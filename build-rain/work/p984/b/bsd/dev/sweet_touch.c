/*
 * SWEET SE7 FIFO I2C / GT9896 Yellowstone / Focaltech 54:52 bring-up.
 * topaz (SM6225): FocalTech FT5452 at 0x38 on QUP0 SE2, IRQ GPIO80, reset
 * GPIO86, AVDD enable GPIO36 (no regulator), values from topaz-woa
 * TopazTouch. Call once after the scheduler is running; all work runs on one
 * thread. Requires ABL-loaded GENI I2C firmware.
 * No firmware/config upload or direct PMIC writes. Only IC data write:
 * zero to GT9896's event status after reading a report. See touch2/report.md.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <arm/caches_internal.h>
#include <pexpert/device_tree.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
/* Mach MIG declaration is not included by BSD's exported thread.h. */
extern kern_return_t thread_terminate(thread_t);
extern int sweet_rpmh_vreg_enable(const char *res, unsigned mv);
extern void sweet_hid_touch(int finger, int down, int x, int y);
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
extern bool topaz_smmu_identity(uint32_t sid, uint32_t mask, const char *name);  /* TopazModem.cpp */
void sweet_touch_init(void);

#define TOUCH_SE_PA      0x04a88000u /* QUP0 SE2 */
#define TOUCH_GCC_PA     0x01400000u
#define TOUCH_WEST_PA    0x00500000u /* TLMM 0x400000 + WEST tile: GPIO 6/7/80/86 */
#define TOUCH_EAST_PA    0x00d00000u /* TLMM 0x400000 + EAST tile: GPIO 36 */
#define TOUCH_GCC_VOTE   0x7900cu    /* QUP0 wrapper + SE2 vote, bits 0x13c0 */
#define TOUCH_RCG       0x1f3a8u    /* QUP0 S2 CMD; CFG +4, CBCR -4 (gcc-khaje.c) */
#define TOUCH_PIN_RESET  86u
#define TOUCH_PIN_IRQ    80u
#define TOUCH_PIN_AVDD   36u
#define TOUCH_GPIO_CFG(n) ((uint32_t)(n) * 0x1000u)
#define TOUCH_GPIO_IO(n)  (TOUCH_GPIO_CFG(n) + 4u)
#define TOUCH_MAX        10u
#define TOUCH_GOODIX     1
#define TOUCH_FTS        2
/* GT9896 is Yellowstone in Xiaomi sweet-r-oss, not the Berlin protocol. */
#define GOODIX_PID_REG   0x4022u
#define GOODIX_EVENT_REG 0x4180u
#define GOODIX_HEAD_LEN  8u
#define GOODIX_POINT_LEN 8u

#define SE_STATUS       0x040u
#define SE_MCLK         0x048u
#define SE_FW           0x068u
#define SE_CLK_SEL      0x07cu
#define SE_DMA          0x258u
#define SE_TX_LEN       0x26cu
#define SE_RX_LEN       0x270u
#define SE_SCL          0x278u
#define SE_CMD          0x600u
#define SE_CTRL         0x604u
#define SE_IRQ          0x610u
#define SE_IRQ_EN       0x614u
#define SE_IRQ_CLR      0x618u
#define SE_TX           0x700u
#define SE_RX           0x780u
#define SE_TX_COUNT     0x800u
#define SE_RX_COUNT     0x804u
#define SE_HW_TX        0xe24u
#define SE_HW_RX        0xe28u
#define SE_TX_WM        0x80cu
#define SE_RX_WM_IRQ    (1u << 26)
#define SE_RX_LAST      (1u << 27)
#define SE_TX_WM_IRQ    (1u << 30)
#define SE_DONE         (1u << 0)
#define SE_NACK         (1u << 10)
#define SE_ERRORS       (0x3eu | (1u << 9) | (1u << 10) | (1u << 12) | \
                         (1u << 13) | (3u << 24) | (3u << 28))
#define SE_EVENTS       (SE_DONE | SE_ERRORS | SE_RX_WM_IRQ | SE_RX_LAST | SE_TX_WM_IRQ)
/* geni_se_config_packing(8, 4, true, true, true): start bits 7/15/23/31,
 * MSB-to-LSB, length 8, STOP only on vector 3. FIFO bytes remain LSB first. */
#define SE_PACK0        0x0007f8feu
#define SE_PACK1        0x000ffefeu

static struct {
	vm_offset_t se, gcc, west, east;
	bool started, dead, probing;
	int kind, irq_hint;
	uint32_t irq_status, irq_last, active;
	unsigned sent, got;
	int x[TOUCH_MAX], y[TOUCH_MAX];
	/* sweettouchdiag: last step reached and the values seen on the way */
	int diag;
	const char *stage;
	uint32_t d_vote, d_cfg, d_cbcr, d_fw, d_status, d_hwtx, d_hwrx;
	int d_fe1, d_fe2, d_ge;
	uint8_t d_fid1, d_fid2;
	/* sweettouchdiag=2: polling statistics */
	unsigned s_frames, s_errs, s_down, s_up, s_maxn;
	int s_lasterr, s_lastx, s_lasty;
	uint8_t s_raw[12];
} st;
static int touch_sleep_chan;
static int rain_touch_on;      /* boot-arg raintouch=1: Novatek SPI probe (rain) */

static uint32_t
touch_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
touch_wr(vm_offset_t base, uint32_t off, uint32_t val)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = val;
	__asm__ volatile("dsb osh" ::: "memory");
}

static void
touch_wait(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&touch_sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

static bool
touch_wait_clear(vm_offset_t base, uint32_t off, uint32_t mask)
{
	for (unsigned i = 0; i < 5000; ++i) {
		if (!(touch_rd(base, off) & mask)) {
			return true;
		}
		delay(1);
	}
	return false;
}

/* topaz: GPIO 36 is on the EAST tile, the touch pins otherwise on WEST. */
static vm_offset_t
touch_tile(unsigned pin)
{
	return pin == TOUCH_PIN_AVDD ? st.east : st.west;
}

static void
touch_gpio(unsigned pin, unsigned mux, unsigned pull, unsigned drive, bool out, bool high)
{
	vm_offset_t tile = touch_tile(pin);
	uint32_t off = TOUCH_GPIO_CFG(pin);
	uint32_t cfg = touch_rd(tile, off);
	if (out) {
		/* Set the desired latch before enabling output (reset is active low). */
		touch_wr(tile, TOUCH_GPIO_IO(pin),
		    (touch_rd(tile, TOUCH_GPIO_IO(pin)) & ~2u) | (high ? 2u : 0u));
	}
	cfg &= ~0x3ffu;
	cfg |= (mux << 2) | pull | (drive << 6) | (out ? 0x200u : 0u);
	touch_wr(tile, off, cfg);
}

static bool
touch_clocks(void)
{
	/*
	 * topaz (TopazTouch hwio.c): OR the QUP0 wrapper/SE2 votes into the
	 * shared vote register (other bits belong to other clients), XO parent
	 * with divider 1 = 19.2 MHz, no PLL or voltage change.
	 */
	touch_wr(st.gcc, TOUCH_GCC_VOTE, touch_rd(st.gcc, TOUCH_GCC_VOTE) | 0x13c0u);
	printf("SWEET TOUCH: gcc vote=%#x rcg=%#x cfg=%#x cbcr=%#x\n",
	    touch_rd(st.gcc, TOUCH_GCC_VOTE), touch_rd(st.gcc, TOUCH_RCG),
	    touch_rd(st.gcc, TOUCH_RCG + 4u), touch_rd(st.gcc, TOUCH_RCG - 4u));
	st.d_vote = touch_rd(st.gcc, TOUCH_GCC_VOTE);
	touch_wr(st.gcc, TOUCH_RCG + 4u, 1u);
	touch_wr(st.gcc, TOUCH_RCG, touch_rd(st.gcc, TOUCH_RCG) | 1u);
	if (!touch_wait_clear(st.gcc, TOUCH_RCG, 1u)) {
		printf("SWEET TOUCH: RCG update timeout\n");
		return false;
	}
	st.d_cfg = touch_rd(st.gcc, TOUCH_RCG + 4u);
	st.d_cbcr = touch_rd(st.gcc, TOUCH_RCG - 4u);
	if (!touch_wait_clear(st.gcc, TOUCH_RCG - 4u, 1u << 31)) {
		printf("SWEET TOUCH: SE2 branch halted (%#x)\n", touch_rd(st.gcc, TOUCH_RCG - 4u));
		return false;
	}
	return true;
}

/* Called only for a transfer started by this driver, including a stretched TX. */
static void
touch_abort(void)
{
	/* i2c-qcom-geni.c: M_GENI_CMD_CANCEL, then ABORT if cancel does not finish */
	bool done = false;
	touch_wr(st.se, SE_CTRL, 4);
	for (unsigned i = 0; i < 1000; ++i) {
		if (touch_rd(st.se, SE_IRQ) & (1u << 4)) {
			done = true;
			break;
		}
		delay(1);
	}
	if (!done) {
		touch_wr(st.se, SE_CTRL, 2); /* abort M command, release held bus */
		for (unsigned i = 0; i < 1000; ++i) {
			if (touch_rd(st.se, SE_IRQ) & (1u << 5)) {
				done = true;
				break;
			}
			delay(1);
		}
	}
	if (!done || !touch_wait_clear(st.se, SE_STATUS, 1u)) {
		st.dead = true;
		printf("SWEET TOUCH: abort failed status=%#x irq=%#x; stopping\n",
		    touch_rd(st.se, SE_STATUS), touch_rd(st.se, SE_IRQ));
	}
	/* Bound drain even if the hardware reports an impossible FIFO count. */
	for (unsigned i = 0; i < 64 && (touch_rd(st.se, SE_RX_COUNT) & 0x1ffffffu); ++i) {
		(void)touch_rd(st.se, SE_RX);
	}
	touch_wr(st.se, SE_IRQ_CLR, 0xffffffffu);
	if (touch_rd(st.se, SE_TX_COUNT) & 0xfffffffu) {
		st.dead = true;
		printf("SWEET TOUCH: TX FIFO not empty after abort; stopping\n");
	}
}

/* Keep all observed bits for diagnostics, even if a watermark was cleared. */
static uint32_t
touch_irq(void)
{
	st.irq_last = touch_rd(st.se, SE_IRQ);
	st.irq_status |= st.irq_last;
	return st.irq_last;
}

/* One Linux-style FIFO message. TX is at most 3 bytes; RX at most 90. */
static int
touch_msg_run(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx,
    unsigned nr, bool stretch)
{
	if (st.dead || nt > 3 || nr > 90 || (nt && nr) || (!nt && !nr)) {
		return EIO;
	}
	if (!touch_wait_clear(st.se, SE_STATUS, 1u)) {
		st.dead = true; /* no other SE7 owner is supported */
		return EBUSY;
	}
	touch_wr(st.se, SE_IRQ_CLR, 0xffffffffu);
	touch_wr(st.se, SE_TX_WM, nt ? 1u : 0u);
	touch_wr(st.se, nr ? SE_RX_LEN : SE_TX_LEN, nr ? nr : nt);
	touch_wr(st.se, SE_CMD, ((nr ? 2u : 1u) << 27) |
	    ((uint32_t)addr << 9) | (stretch ? 4u : 0u));
	if (nr > 8 && !st.probing) {
		touch_wait(2); /* sleep, do not spin, through most of the transfer */
	}
	int err = ETIMEDOUT;
	for (unsigned i = 0; i < 5000; ++i) {
		uint32_t irq = touch_irq();
		if (irq & SE_ERRORS) {
			err = (irq & SE_NACK) ? ENXIO : EIO;
			/* A completed NACK is normal during address probing. Aborting an
			 * idle sequencer may never produce ABORT_DONE. Wait for completion
			 * and leave a clean idle engine available for the other IC. */
			for (unsigned j = 0; j < 5000 && !(irq & SE_DONE); ++j) {
				delay(1);
				irq = touch_irq();
			}
			if ((irq & SE_DONE) && !(touch_rd(st.se, SE_STATUS) & 1u) &&
			    !(touch_rd(st.se, SE_TX_COUNT) & 0xfffffffu) &&
			    !(touch_rd(st.se, SE_RX_COUNT) & 0x1ffffffu)) {
				touch_wr(st.se, SE_IRQ_CLR, irq);
				return err;
			}
			break;
		}
		if (nt && !st.sent && (irq & SE_TX_WM_IRQ)) {
			uint32_t word = 0;
			for (unsigned j = 0; j < nt; ++j) {
				word |= (uint32_t)tx[j] << (8u * j);
			}
			touch_wr(st.se, SE_TX, word);
			st.sent = nt;
			touch_wr(st.se, SE_TX_WM, 0);
		}
		uint32_t count = touch_rd(st.se, SE_RX_COUNT) & 0x1ffffffu;
		if (count > 64 || count > (nr - st.got + 3u) / 4u) {
			err = EIO;
			break;
		}
		while (count--) {
			uint32_t word = touch_rd(st.se, SE_RX);
			for (unsigned j = 0; j < 4 && st.got < nr; ++j) {
				rx[st.got++] = (uint8_t)(word >> (8u * j));
			}
		}
		/* DONE is checked after draining RX, so final short words aren't lost. */
		if (irq & SE_DONE) {
			/* Re-sample after the final drain; a late error must beat DONE. */
			irq = touch_irq();
			if (irq & SE_ERRORS) {
				continue;
			}
			touch_wr(st.se, SE_IRQ_CLR, irq);
			if (st.got == nr && st.sent == nt) {
				return 0;
			}
			err = EIO;
			break;
		}
		/* Preserve DONE/errors if they arrive during this iteration. */
		touch_wr(st.se, SE_IRQ_CLR, irq & (SE_RX_WM_IRQ | SE_RX_LAST | SE_TX_WM_IRQ));
		delay(1);
	}
	touch_abort();
	return err;
}

static int
touch_msg(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx,
    unsigned nr, bool stretch)
{
	st.irq_status = st.irq_last = 0;
	st.sent = st.got = 0;
	int err = touch_msg_run(addr, tx, nt, rx, nr, stretch);
	if (st.probing) {
		printf("SWEET TOUCH: probe xfer addr=%#x %s len=%u stretch=%u err=%d irq=%#x seen=%#x tx=%u rx=%u status=%#x\n",
		    addr, nr ? "read" : "write", nr ? nr : nt, stretch ? 1u : 0u,
		    err, st.irq_last, st.irq_status, st.sent, st.got,
		    touch_rd(st.se, SE_STATUS));
	}
	return err;
}

static int
touch_read(uint8_t addr, uint16_t reg, bool wide, uint8_t *buf, unsigned len)
{
	uint8_t ptr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
	int err = touch_msg(addr, wide ? ptr : ptr + 1, wide ? 2u : 1u,
	    NULL, 0, true);
	if (err) {
		return err;
	}
	/* STOP_STRETCH on the pointer write retains the bus for a repeated START. */
	return touch_msg(addr, NULL, 0, buf, len, false);
}

static int
touch_goodix_ack(void)
{
	const uint8_t ack[3] = {GOODIX_EVENT_REG >> 8, GOODIX_EVENT_REG & 0xffu, 0};
	return touch_msg(0x5d, ack, sizeof(ack), NULL, 0, false);
}

/* Raw read only: 0x8240 is the vendor's generic/older VERSION candidate,
 * not a second accepted GT9896 identity. Never try a four-byte Berlin pointer:
 * on this 16-bit IC the extra bytes would be register data writes. */
static int
touch_probe_info(uint16_t reg, uint8_t *buf)
{
	memset(buf, 0xa5, 32);
	int err = touch_read(0x5d, reg, true, buf, 32);
	printf("SWEET TOUCH: probe info reg=%#x len=32 err=%d irq=%#x seen=%#x\n",
	    reg, err, st.irq_last, st.irq_status);
	if (!err) {
		for (unsigned off = 0; off < 32; off += 16) {
			char line[49];
			for (unsigned j = 0; j < 16; ++j) {
				(void)snprintf(line + j * 3u, 4, "%02x ", buf[off + j]);
			}
			printf("SWEET TOUCH: probe raw %#x: %s\n", reg + off, line);
		}
	}
	return err;
}

static unsigned
touch_be16(const uint8_t *p)
{
	return ((unsigned)p[0] << 8) | p[1];
}

static bool
touch_checksum(const uint8_t *buf, unsigned len)
{
	if (len < 2) {
		return false;
	}
	unsigned sum = 0;
	for (unsigned i = 0; i + 2 < len; ++i) {
		sum += buf[i];
	}
	return (sum & 0xffffu) == touch_be16(buf + len - 2);
}

static void
touch_report(uint32_t mask, const int *x, const int *y)
{
	for (unsigned id = 0; id < TOUCH_MAX; ++id) {
		uint32_t bit = 1u << id;
		if (mask & bit) {
			if (!(st.active & bit) || x[id] != st.x[id] || y[id] != st.y[id]) {
				sweet_hid_touch((int)id, 1, x[id], y[id]);
				st.s_down++;
				st.s_lastx = x[id];
				st.s_lasty = y[id];
			}
			st.x[id] = x[id];
			st.y[id] = y[id];
		} else if (st.active & bit) {
			sweet_hid_touch((int)id, 0, st.x[id], st.y[id]);
			st.s_up++;
		}
	}
	st.active = mask;
}

static int
touch_goodix(void)
{
	uint8_t buf[90];
	int x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};
	uint32_t mask = 0;
	int err = touch_read(0x5d, GOODIX_EVENT_REG, true, buf, GOODIX_HEAD_LEN);
	if (err) {
		return err;
	}
	if (!(buf[0] & 0x80u)) {
		/* Do not acknowledge firmware/config requests that we cannot service. */
		if (buf[0] & 0x40u) {
			return ENOTSUP;
		}
		return 0; /* no new touch frame: retain current contacts */
	}
	if (!touch_checksum(buf, GOODIX_HEAD_LEN)) {
		return EIO; /* header untrusted: no write to the IC */
	}
	unsigned n = buf[2] & 15u;
	if (n > TOUCH_MAX) {
		(void)touch_goodix_ack();
		return EIO;
	}
	if (n) {
		err = touch_read(0x5d, GOODIX_EVENT_REG + GOODIX_HEAD_LEN, true,
		    buf + GOODIX_HEAD_LEN, n * GOODIX_POINT_LEN + 2u);
		if (err) {
			return err;
		}
		if (!touch_checksum(buf + GOODIX_HEAD_LEN, n * GOODIX_POINT_LEN + 2u)) {
			(void)touch_goodix_ack();
			return EIO;
		}
	}
	for (unsigned i = 0; i < n; ++i) {
		const uint8_t *p = buf + GOODIX_HEAD_LEN + i * GOODIX_POINT_LEN;
		unsigned id = p[0] >> 4;
		unsigned px = touch_be16(p + 2), py = touch_be16(p + 4);
		/* Low nibble 1/3 denotes pen data in the vendor parser. */
		if ((p[0] & 15u) == 1u || (p[0] & 15u) == 3u || id >= TOUCH_MAX ||
		    (mask & (1u << id)) || px > 1079 || py > 2399) {
			(void)touch_goodix_ack();
			return EIO;
		}
		mask |= 1u << id;
		x[id] = (int)px;
		y[id] = (int)py;
	}
	err = touch_goodix_ack();
	if (!err) {
		touch_report(mask, x, y);
	}
	return err;
}

static int
touch_focaltech(void)
{
	uint8_t buf[63];
	int x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};
	uint32_t mask = 0, seen = 0;
	int err = touch_read(0x38, 0, false, buf, sizeof(buf));
	if (err) {
		return err;
	}
	unsigned n = buf[2] & 15u, active = 0;
	if (n || !st.s_frames) {
		memcpy(st.s_raw, buf, sizeof(st.s_raw)); /* keep the last frame with contacts */
	}
	st.s_frames++;
	if (n > st.s_maxn) {
		st.s_maxn = n;
	}
	if (n == 0 || (n > TOUCH_MAX && buf[3] == 0xffu)) {
		/*
		 * topaz: an empty report releases every contact, like Linux
		 * focaltech and TopazTouch. With no contact the FT5452 (firmware
		 * as updated by HyperOS 2.0.208) reports TD_STATUS 0xff and 0xff
		 * records; TD_STATUS 0 must not be parsed as slots either: its
		 * records can carry a stale contact slot that resurrects a lifted
		 * finger (TopazTouch v0.13 fixed exactly that stuck-finger case),
		 * and an all-zero frame would become a phantom finger 0 at (0,0).
		 */
		touch_report(0, x, y);
		return 0;
	}
	if (n > TOUCH_MAX) {
		/* topaz: show what the IC actually sends (rate-limited) */
		static uint64_t last_dump;
		uint64_t now = mach_absolute_time(), gap;
		nanoseconds_to_absolutetime(NSEC_PER_SEC, &gap);
		if (now - last_dump > gap) {
			last_dump = now;
			printf("SWEET TOUCH: bad frame n=%u raw %02x %02x %02x | %02x %02x %02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x %02x\n",
			    n, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8],
			    buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
		}
		return EIO;
	}
	for (unsigned i = 0; i < TOUCH_MAX; ++i) {
		const uint8_t *p = buf + 3u + 6u * i;
		unsigned id = p[2] >> 4, event = p[0] >> 6;
		if (id >= TOUCH_MAX) {
			break; /* vendor's unused-record sentinel */
		}
		if (seen & (1u << id)) {
			return EIO;
		}
		seen |= 1u << id;
		if (event == 1u) {
			continue; /* UP record */
		}
		if (event == 3u) {
			continue; /* topaz: "no event" slot (TopazTouch) */
		}
		unsigned px = ((unsigned)(p[0] & 15u) << 8) | p[1];
		unsigned py = ((unsigned)(p[2] & 15u) << 8) | p[3];
		/* topaz: raw 0..1349 x 0..2999 (1.25x the panel) + 2-bit fractions */
		if (px > 1349 || py > 2999) {
			return EIO;
		}
		px = (px * 4u + (p[4] >> 6)) / 5u;
		py = (py * 4u + ((p[4] >> 4) & 3u)) / 5u;
		px = px > 1079 ? 1079 : px;
		py = py > 2399 ? 2399 : py;
		mask |= 1u << id;
		x[id] = (int)px;
		y[id] = (int)py;
		++active;
	}
	(void)active;   /* topaz: byte 2 does not always match the records */
	touch_report(mask, x, y);
	return 0;
}

static bool
touch_se_init(void)
{
	uint32_t fw = touch_rd(st.se, SE_FW);
	uint32_t tx = touch_rd(st.se, SE_HW_TX), rx = touch_rd(st.se, SE_HW_RX);
	st.d_fw = fw;
	st.d_status = touch_rd(st.se, SE_STATUS);
	st.d_hwtx = tx;
	st.d_hwrx = rx;
	printf("SWEET TOUCH: se2 fw=%#x proto=%u status=%#x tx=%#x rx=%#x ios=%#x\n",
	    fw, (fw >> 8) & 0xffu, touch_rd(st.se, SE_STATUS), tx, rx,
	    touch_rd(st.se, 0x908));
	if (((fw >> 8) & 0xffu) != 3u || (touch_rd(st.se, 0x64) & 1u) ||
	    ((tx >> 24) & 0x3fu) != 32u || ((rx >> 24) & 0x3fu) != 32u ||
	    ((tx >> 16) & 0x3fu) < 2u || ((rx >> 16) & 0x3fu) < 2u) {
		printf("SWEET TOUCH: need ABL-loaded I2C firmware and 32-bit FIFOs; probes skipped\n");
		return false;
	}
	if (touch_rd(st.se, SE_STATUS) & 0x1001u) {
		printf("SWEET TOUCH: SE2 already busy; probes skipped\n");
		return false;
	}
	touch_wr(st.se, 0xe18, 0); /* no GSI events */
	touch_wr(st.se, 0xe1c, 0); /* mask external IRQ routing; poll local status */
	touch_wr(st.se, 0x644, 0);
	touch_wr(st.se, SE_IRQ_CLR, 0xffffffffu);
	touch_wr(st.se, 0x648, 0xffffffffu);
	touch_wr(st.se, SE_DMA, 0);
	touch_wr(st.se, 0x28, touch_rd(st.se, 0x28) | 0x7fu);
	touch_wr(st.se, 0xe30, touch_rd(st.se, 0xe30) | 0x0fu);
	touch_wr(st.se, 0x24, 0x7f);
	touch_wr(st.se, 0x20, 1);
	touch_wr(st.se, SE_CLK_SEL, 0);
	touch_wr(st.se, SE_MCLK, (2u << 4) | 1u);
	touch_wr(st.se, SE_SCL, (5u << 20) | (11u << 10) | 22u); /* 400 kHz */
	/* Linux 8-bit I2C packing: MSB on the wire, LSB byte first in FIFO. */
	touch_wr(st.se, 0x254, 0);
	touch_wr(st.se, 0x260, SE_PACK0);
	touch_wr(st.se, 0x264, SE_PACK1);
	touch_wr(st.se, 0x284, SE_PACK0);
	touch_wr(st.se, 0x288, SE_PACK1);
	touch_wr(st.se, SE_TX_WM, 0);
	touch_wr(st.se, 0x810, 0);
	touch_wr(st.se, 0x814, ((rx >> 16) & 0x3fu) - 1u);
	touch_wr(st.se, SE_IRQ_EN, SE_EVENTS);
	printf("SWEET TOUCH: packing tx=%#x/%#x rx=%#x/%#x gran=%#x\n",
	    touch_rd(st.se, 0x260), touch_rd(st.se, 0x264),
	    touch_rd(st.se, 0x284), touch_rd(st.se, 0x288), touch_rd(st.se, 0x254));
	if (touch_rd(st.se, 0x260) != SE_PACK0 || touch_rd(st.se, 0x264) != SE_PACK1 ||
	    touch_rd(st.se, 0x284) != SE_PACK0 || touch_rd(st.se, 0x288) != SE_PACK1 ||
	    touch_rd(st.se, 0x254) != 0) {
		printf("SWEET TOUCH: packing readback mismatch; probes skipped\n");
		return false;
	}
	if ((touch_rd(st.se, SE_TX_COUNT) & 0xfffffffu) ||
	    (touch_rd(st.se, SE_RX_COUNT) & 0x1ffffffu)) {
		printf("SWEET TOUCH: inherited FIFO data; probes skipped\n");
		return false;
	}
	return true;
}

/*
 * rain (Redmi 10C, BOE nt36525b panel): Novatek NT36525B TDDI touch on QUP0
 * SE0 in SPI mode. Stage A only: SE0 clocks, SPI FIFO mode, a software
 * bootloader reset of the touch MCU and a read of its 6 chip-ID bytes. No
 * hardware reset pin (TDDI: shared with the display), no firmware.
 * Boot-arg raintouch=1; ends in a panic whose text carries every reading.
 */
#define RAIN_SE0_PA       0x04a80000u /* QUP0 SE0 spi@4a80000 */
#define RAIN_GCC_VOTE_SE0 0x7c0u      /* QUP0 wrapper bits 6..9 + S0 bit 10 */
#define RAIN_RCG_SE0      0x1f148u    /* QUP0 S0 CMD; CFG +4, CBCR -4 (S2 0x1f3a8 - 2*0x130) */
#define RAIN_SPI_CPHA     0x224u
#define RAIN_SPI_LOOPBACK 0x22cu
#define RAIN_SPI_CPOL     0x230u
#define RAIN_SPI_DEMUXINV 0x24cu
#define RAIN_SPI_DEMUXSEL 0x250u
#define RAIN_SPI_TRANSCFG 0x25cu
#define RAIN_SPI_WORDLEN  0x268u
#define RAIN_SPI_PREPOST  0x274u
#define RAIN_SPI_DELAYS   0x278u
#define RAIN_OP_TX        1u          /* SPI_TX_ONLY */
#define RAIN_OP_FULL      3u          /* SPI_FULL_DUPLEX */
#define RAIN_NVT_ENGRST   0x7fff80u   /* SPI engine reset, written before anything else */
#define RAIN_NVT_RST      0x3f0feu    /* DT novatek,swrst-n8-addr */
#define RAIN_NVT_RDFAST   0x3f310u    /* DT novatek,spi-rd-fast-addr */
#define RAIN_NVT_TRIM     0x3f004u
#define RAIN_NVT_TRIM_OLD 0x1f64eu

static struct {
	vm_offset_t se;
	uint32_t vote, rcgcfg, cbcr, fw, hwtx, hwrx, status;
	uint32_t pin_before[4], pin_after[4];
	int err;
	unsigned xfers;
	uint8_t id[6], id_old[6];
	const char *stage;
	/* 0963: engine state at the first failed command, SPI config read back */
	bool snapped;
	uint32_t f_op, f_n, f_irq, f_status, f_txfs, f_rxfs, f_ios, f_sirq;
	uint32_t c_mclk, c_clksel, c_word, c_cpha, c_cpol, c_demux, c_trans, c_cgc, c_dma, c_pack0;
	/* 0964: clock tree evidence and a forced-root retry */
	uint32_t k_cmd, k_b138, k_b144, k_se2cmd, k_se2cfg, k_cmd2;
	int retry_err;
	/* 0965: interface flags, read before any SPI command */
	uint32_t ifdis, fws, proto_s, seqcfg;
} rn;

static void
rain_snap(uint32_t op, unsigned n)
{
	if (rn.snapped) {
		return;
	}
	rn.snapped = true;
	rn.f_op = op;
	rn.f_n = n;
	rn.f_irq = touch_rd(rn.se, SE_IRQ);
	rn.f_status = touch_rd(rn.se, SE_STATUS);
	rn.f_txfs = touch_rd(rn.se, SE_TX_COUNT);
	rn.f_rxfs = touch_rd(rn.se, SE_RX_COUNT);
	rn.f_ios = touch_rd(rn.se, 0x908);         /* GENI_IOS: pin levels seen by the SE */
	rn.f_sirq = touch_rd(rn.se, 0x640);        /* S engine IRQ status */
}

/*
 * rain 0966: GPI (QUP GSI) DMA for SE0. On rain SE0 has FIFO_IF_DISABLE
 * set, so SPI only works through GPI instance 0 of the QUP0 GPI engine
 * (DT qcom,gpi-dma@4a00000, ee offset 0x10000, SE0 dmas: GPII 0, ch 0 TX /
 * ch 1 RX, protocol 1). Polled: no interrupts are used.
 * GPII 0 register block = 0x4a00000 - 0x10000 + 0x20000.
 */
#define RG_PA          0x04a10000u
#define RG_CH(k)       (0x0000u + 0x80u * (k))   /* channel context */
#define RG_EV          0x1000u                   /* event ring context */
#define RG_CH_DB(k)    (0x2000u + 0x8u * (k))
#define RG_EV_DB       0x2100u
#define RG_CH_CMD      0x3008u
#define RG_EV_CMD      0x3010u
#define RG_TYPE_IRQ    0x3080u
#define RG_SRC_CH      0x3090u
#define RG_SRC_EV      0x3094u
#define RG_SRC_CH_CLR  0x30a0u
#define RG_SRC_EV_CLR  0x30a4u
#define RG_IEOB_CLR    0x30c0u
#define RG_GLOB_STTS   0x3100u
#define RG_GPII_STTS   0x3118u
#define RG_INTSET      0x3180u
#define RG_ERRLOG      0x3200u
#define RG_QOS(k)      (0x005cu + 0x80u * (k))
#define RG_SCR(k, i)   (0x0060u + 0x80u * (k) + 4u * (i))
/* context words */
#define RC_CFG         0x00u
#define RC_LEN         0x04u
#define RC_BASE_LO     0x08u
#define RC_BASE_HI     0x0cu
#define RC_RP_LO       0x10u
#define RC_WP_LO       0x18u
#define RC_INTMOD      0x20u
#define RC_MSI_LO      0x28u
#define RC_MSI_HI      0x2cu
#define RC_RPU_LO      0x30u
#define RC_RPU_HI      0x34u
#define RG_STATE(v)    (((v) >> 20) & 0xfu)      /* 1 allocated, 2 started */
/* ring entries (16 bytes) */
#define RT_DMA         0x10u
#define RT_GO          0x20u
#define RT_CONFIG0     0x22u
#define RT_CHAIN       (1u << 0)
#define RT_IEOT        (1u << 9)
#define RT_LINK        (1u << 11)      /* duplex GO: tie in the RX channel */
#define RG_NTRE        64u
#define RG_NEV         128u
/* memory layout in one 16 KiB contiguous block */
#define RM_TX          0x0000u
#define RM_RX          0x0400u
#define RM_EV          0x0800u
#define RM_TXBUF       0x1000u     /* up to RM_TXMAX bytes */
#define RM_RXBUF       0x2400u     /* up to RM_RXMAX bytes */
#define RM_TXMAX       0x1100u
#define RM_RXMAX       0x400u
#define RM_SIZE        0x4000u

struct rain_tre { uint32_t w[4]; };

/* SPI CONFIG0: 8-bit words (encoded bits-4), packing on, mode 0, clock source 0 (XO) / div. */
static void
rain_tre_config(struct rain_tre *t, uint32_t div, bool loopback)
{
	t->w[0] = (8u - 4u) | (loopback ? (1u << 8) : 0u) | (1u << 24) | (1u << 25);
	t->w[1] = 0;
	t->w[2] = (div & 0xfffu) | (0u << 16);
	t->w[3] = (RT_CONFIG0 << 16) | RT_CHAIN;
}

/* GO: SPI command (1 TX, 3 duplex), chip select 0, no fragmentation (CS drops after). */
static void
rain_tre_go(struct rain_tre *t, uint32_t cmd, uint32_t rx_len)
{
	t->w[0] = cmd & 0x1fu;
	t->w[1] = 0;
	t->w[2] = rx_len & 0xffffffu;
	t->w[3] = (RT_GO << 16) | RT_CHAIN | (cmd == 3u ? RT_LINK : 0u);
}

static void
rain_tre_dma(struct rain_tre *t, uint64_t pa, uint32_t len, bool ieot)
{
	t->w[0] = (uint32_t)pa;
	t->w[1] = (uint32_t)(pa >> 32);
	t->w[2] = len & 0xffffffu;
	t->w[3] = (RT_DMA << 16) | (ieot ? RT_IEOT : 0u);   /* only the TX side signals end of transfer */
}

static struct {
	vm_offset_t g;                  /* GPII 0 registers */
	vm_offset_t mem;                /* contiguous block, kernel VA */
	uint64_t pa;                    /* its physical address */
	unsigned tx_wp, rx_wp, ev_rp;   /* in elements */
	uint32_t pre_ch0, pre_ch1, pre_ev, st_ev, st_ch0, st_ch1, st_go0, st_go1;
	/* first failed transfer */
	bool snapped;
	uint32_t f_ch0, f_ch1, f_rp0, f_wp0, f_evrp, f_type, f_glob, f_gpii, f_err, f_srcch;
	uint32_t ev_w2, ev_w3, nev;     /* last event seen */
	uint32_t rx_w3, nimm;           /* last RX-channel event; immediate-data events seen */
	uint32_t rx_w0, rx_w2;          /* its TRE pointer (low) and length|code */
	vm_offset_t uc;                 /* uncached alias of mem: DMA data without the CPU cache */
	uint8_t cached[8];              /* last duplex RX as seen through the cached mapping */
	const char *gstage;
	bool loopback;                  /* 0975: CONFIG0 loopback for the RX path test */
	uint8_t lb[8];
} rg;

static uint32_t rg_rd(uint32_t off) { return touch_rd(rg.g, off); }
/* 0967: on-screen breadcrumbs with a pause, and raintouchstop=N to stop after step N */
static unsigned rain_stop, rain_step, rain_pause_ms = 3000;
static int rain_smmu = -1;      /* 0970: result of the SID 0xf6 identity bank */
/*
 * 0971: apps SMMU (MMU-500 at 0xc600000, 4 KiB pages, 64 global pages) fault capture
 * around the first doorbell. sCR0 GFRE|GFIE are cleared first so a global fault is
 * recorded instead of raised; the probe then reports the faulting SID and address.
 */
static struct {
	vm_offset_t s;
	int cb;
	uint32_t cr0_was, cr0_now, gfsr, gfsynr0, gfsynr1, gfsynr2, gfar_lo, gfar_hi, cbfsr, cbfar_lo, cbfar_hi, cbfsynr0;
	bool seen;
} sm;

/* C9: the QUP0 wrapper's own stream (DT qupv3_id_0 iommus = <&apps_smmu 0xe3 0x0>) and any CB fault */
static struct {
	uint32_t nvalid, slot, smr, s2cr, sctlr;    /* bootloader's SMR state for SID 0xe3 */
	bool e3;                                    /* identity bank installed for 0xe3 */
	int fcb;                                    /* first context bank with a fault, -1 none */
	uint32_t ffsr, ffar, fsyn, fsid;
} r9 = { .slot = 0xffu, .fcb = -1 };

static void
rain_smmu_pre(void)
{
	vm_offset_t s = ml_io_map(0x0c600000u, 0x80000);
	if (!s) {
		return;
	}
	for (uint32_t i = 0; i < 50; i++) {
		uint32_t smr = touch_rd(s, 0x800u + 4u * i);
		if (!(smr >> 31)) {
			continue;
		}
		r9.nvalid++;
		uint32_t id = smr & 0x7fffu, m = (smr >> 16) & 0x7fffu;
		if (((id ^ 0xe3u) & ~m & 0x7fffu) == 0 && r9.slot == 0xffu) {
			r9.slot = i;
			r9.smr = smr;
			r9.s2cr = touch_rd(s, 0xc00u + 4u * i);
			if (((r9.s2cr >> 16) & 3u) == 0) {      /* translate: that bank's SCTLR */
				r9.sctlr = touch_rd(s, 0x40000u + 0x1000u * (r9.s2cr & 0x3fu));
			}
		}
	}
}

static void
rain_smmu_cbscan(void)
{
	if (!sm.s) {
		return;
	}
	for (uint32_t cb = 0; cb < 48; cb++) {
		uint32_t fsr = touch_rd(sm.s, 0x40000u + 0x1000u * cb + 0x58u);
		if (fsr & 0xc00001feu) {
			r9.fcb = (int)cb;
			r9.ffsr = fsr;
			r9.ffar = touch_rd(sm.s, 0x40000u + 0x1000u * cb + 0x60u);
			r9.fsyn = touch_rd(sm.s, 0x40000u + 0x1000u * cb + 0x68u);
			r9.fsid = touch_rd(sm.s, 0x1000u + 0x400u + 4u * cb) & 0xffffu;   /* CBFRSYNRA */
			return;
		}
	}
}

static void
rain_smmu_arm(void)
{
	sm.cb = -1;
	sm.s = ml_io_map(0x0c600000u, 0x80000);
	if (!sm.s) {
		return;
	}
	for (uint32_t i = 0; i < 128; i++) {      /* the slot topaz_smmu_identity filled */
		if (touch_rd(sm.s, 0x800u + 4u * i) == 0x800000f6u) {
			sm.cb = (int)(touch_rd(sm.s, 0xc00u + 4u * i) & 0x3fu);
			break;
		}
	}
	sm.cr0_was = touch_rd(sm.s, 0x0);
	uint32_t raise = 0;
	(void)PE_parse_boot_argn("raintouchgfault", &raise, sizeof(raise));
	if (!raise) {
		touch_wr(sm.s, 0x0, sm.cr0_was & ~0x6u);    /* GFRE, GFIE off */
	}
	sm.cr0_now = touch_rd(sm.s, 0x0);
	touch_wr(sm.s, 0x48, 0xffffffffu);          /* sGFSR: clear */
	if (sm.cb >= 0) {
		touch_wr(sm.s, 0x40000u + 0x1000u * (uint32_t)sm.cb + 0x58u, 0xffffffffu);   /* CB FSR */
	}
}

/* true once the SMMU has recorded any fault */
static bool
rain_smmu_check(void)
{
	if (!sm.s || sm.seen) {
		return sm.seen;
	}
	/* fault bits only: sGFSR 8:0 + MULTI; CB FSR 8:1 + SS + MULTI (bits 10:9 are FORMAT) */
	uint32_t gfsr = touch_rd(sm.s, 0x48);
	uint32_t cbfsr = sm.cb >= 0 ? touch_rd(sm.s, 0x40000u + 0x1000u * (uint32_t)sm.cb + 0x58u) : 0;
	if (!(gfsr & 0x800001ffu) && !(cbfsr & 0xc00001feu)) {
		return false;
	}
	sm.seen = true;
	sm.gfsr = gfsr;
	sm.gfsynr0 = touch_rd(sm.s, 0x50);
	sm.gfsynr1 = touch_rd(sm.s, 0x54);  /* bits 15:0 = faulting SID */
	sm.gfsynr2 = touch_rd(sm.s, 0x58);
	sm.gfar_lo = touch_rd(sm.s, 0x40);
	sm.gfar_hi = touch_rd(sm.s, 0x44);
	sm.cbfsr = cbfsr;
	if (sm.cb >= 0) {
		vm_offset_t cbp = 0x40000u + 0x1000u * (uint32_t)sm.cb;
		sm.cbfar_lo = touch_rd(sm.s, cbp + 0x60u);
		sm.cbfar_hi = touch_rd(sm.s, cbp + 0x64u);
		sm.cbfsynr0 = touch_rd(sm.s, cbp + 0x68u);
	}
	return true;
}
static bool
rain_bc(unsigned n, const char *what)
{
	rain_step = n;
	printf("RAIN B: %u %s\n", n, what);
	if (rain_pause_ms) {
		touch_wait(rain_pause_ms);
	}
	return rain_stop != 0 && n > rain_stop;   /* true: stop before doing step n */
}
static void rg_wr(uint32_t off, uint32_t v) { touch_wr(rg.g, off, v); }

/* Poll a context's state field after a command; clear the per-channel/event IRQ status. */
static bool
rain_gpi_wait(uint32_t ctx, uint32_t want)
{
	for (unsigned i = 0; i < 100000; i++) {
		if (RG_STATE(rg_rd(ctx + RC_CFG)) == want) {
			rg_wr(RG_SRC_CH_CLR, rg_rd(RG_SRC_CH));
			rg_wr(RG_SRC_EV_CLR, rg_rd(RG_SRC_EV));
			return true;
		}
		delay(1);
	}
	return false;
}

static void
rain_gpi_ring_ctx(uint32_t ctx, uint32_t cfg, uint32_t off, uint32_t len, uint32_t db)
{
	rg_wr(ctx + RC_CFG, cfg);
	rg_wr(ctx + RC_LEN, len);
	rg_wr(ctx + RC_BASE_LO, (uint32_t)(rg.pa + off));
	rg_wr(ctx + RC_BASE_HI, (uint32_t)((rg.pa + off) >> 32));
	rg_wr(db + 4u, (uint32_t)((rg.pa + off) >> 32));      /* doorbell high word */
}

static int
rain_gpi_setup(void)
{
	rg.gstage = "gpi map";
	(void)PE_parse_boot_argn("raintouchstop", &rain_stop, sizeof(rain_stop));
	(void)PE_parse_boot_argn("raintouchpause", &rain_pause_ms, sizeof(rain_pause_ms));
	if (rain_bc(1, "map + alloc")) {
		return -40;
	}
	rg.g = ml_io_map(RG_PA, 0x4000);
	if (!rg.g) {
		return -20;
	}
	if (kmem_alloc_contig(kernel_map, &rg.mem, RM_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return -21;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)rg.mem);
	if (!pn) {
		return -22;
	}
	rg.pa = ((uint64_t)pn << PAGE_SHIFT) | (rg.mem & PAGE_MASK);
	flush_dcache64((addr64_t)rg.pa, RM_SIZE, TRUE);
	rg.uc = ml_io_map((vm_offset_t)rg.pa, RM_SIZE);

	rg.pre_ch0 = rg_rd(RG_CH(0) + RC_CFG);
	rg.pre_ch1 = rg_rd(RG_CH(1) + RC_CFG);
	rg.pre_ev = rg_rd(RG_EV + RC_CFG);
	if (RG_STATE(rg.pre_ch0) || RG_STATE(rg.pre_ch1) || RG_STATE(rg.pre_ev)) {
		return -23;                     /* someone else owns GPII 0: touch nothing */
	}

	/* SE0 into GPI mode (qcom-geni-se: select_gpi_mode), one write per step */
	if (rain_bc(2, "se0 irq off")) {
		return -40;
	}
	touch_wr(rn.se, 0xe1c, 0);
	touch_wr(rn.se, SE_IRQ_EN, 0);
	if (rain_bc(3, "se0 dma mode")) {
		return -40;
	}
	touch_wr(rn.se, SE_DMA, 1);
	if (rain_bc(4, "se0 gsi events")) {
		return -40;
	}
	touch_wr(rn.se, 0xe18, touch_rd(rn.se, 0xe18) | 0xfu);
	if (rain_bc(5, "gpi intset")) {
		return -40;
	}
	rg_wr(RG_INTSET, 1);

	rg.gstage = "ev alloc";
	if (rain_bc(6, "ev alloc cmd")) {
		return -40;
	}
	rg_wr(RG_EV_CMD, (0u << 24) | 0u);
	if (!rain_gpi_wait(RG_EV, 1)) {
		rg.st_ev = rg_rd(RG_EV + RC_CFG);
		return -24;
	}
	rg.st_ev = rg_rd(RG_EV + RC_CFG);
	if (rain_bc(7, "ev ctx + doorbell")) {
		return -40;
	}
	rain_gpi_ring_ctx(RG_EV, (16u << 24) | (1u << 16) | 2u, RM_EV, RG_NEV * 16u, RG_EV_DB);
	rg_wr(RG_EV + RC_INTMOD, 0);
	rg_wr(RG_EV + RC_MSI_LO, 0);
	rg_wr(RG_EV + RC_MSI_HI, 0);
	rg_wr(RG_EV + RC_RPU_LO, 0);
	rg_wr(RG_EV + RC_RPU_HI, 0);
	__asm__ volatile("dsb sy" ::: "memory");
	rg_wr(RG_EV_DB, (uint32_t)(rg.pa + RM_EV + (RG_NEV - 1u) * 16u));  /* whole ring to GSI */

	for (uint32_t k = 0; k < 2; k++) {
		rg.gstage = k ? "ch1 alloc" : "ch0 alloc";
		if (rain_bc(8u + k, rg.gstage)) {
			return -40;
		}
		rg_wr(RG_CH_CMD, (0u << 24) | k);
		bool alloc_ok = rain_gpi_wait(RG_CH(k), 1);
		if (k) {
			rg.st_ch1 = rg_rd(RG_CH(k) + RC_CFG);
		} else {
			rg.st_ch0 = rg_rd(RG_CH(k) + RC_CFG);
		}
		if (!alloc_ok) {
			return -25 - (int)k;
		}
		/* el size 16, event ring 0, dir (ch0 out = 1, ch1 in = 0), protocol GPI */
		rain_gpi_ring_ctx(RG_CH(k), (16u << 24) | (0u << 14) | ((k ? 0u : 1u) << 3) | 2u,
		    k ? RM_RX : RM_TX, RG_NTRE * 16u, RG_CH_DB(k));
		rg_wr(RG_SCR(k, 0), ((k ^ 1u) << 16) | (1u << 4) | 0u);  /* pair, SPI, SE 0 */
		rg_wr(RG_SCR(k, 1), 0);
		rg_wr(RG_SCR(k, 2), 0);
		rg_wr(RG_SCR(k, 3), 0);
		rg_wr(RG_QOS(k), 1);
	}
	__asm__ volatile("dsb sy" ::: "memory");
	for (uint32_t k = 0; k < 2; k++) {
		rg.gstage = k ? "ch1 start" : "ch0 start";
		if (rain_bc(10u + k, rg.gstage)) {
			return -40;
		}
		rg_wr(RG_CH_CMD, (1u << 24) | k);
		bool ok = rain_gpi_wait(RG_CH(k), 2);
		if (k) {
			rg.st_go1 = rg_rd(RG_CH(k) + RC_CFG);
		} else {
			rg.st_go0 = rg_rd(RG_CH(k) + RC_CFG);
		}
		if (!ok) {
			return -27 - (int)k;
		}
	}
	rg.gstage = "gpi ready";
	return 0;
}

static void
rain_gpi_snap(void)
{
	if (rg.snapped) {
		return;
	}
	rg.snapped = true;
	rg.f_ch0 = rg_rd(RG_CH(0) + RC_CFG);
	rg.f_ch1 = rg_rd(RG_CH(1) + RC_CFG);
	rg.f_rp0 = rg_rd(RG_CH(0) + RC_RP_LO);
	rg.f_wp0 = rg_rd(RG_CH(0) + RC_WP_LO);
	rg.f_evrp = rg_rd(RG_EV + RC_RP_LO);
	rg.f_type = rg_rd(RG_TYPE_IRQ);
	rg.f_glob = rg_rd(RG_GLOB_STTS);
	rg.f_gpii = rg_rd(RG_GPII_STTS);
	rg.f_err = rg_rd(RG_ERRLOG);
	rg.f_srcch = rg_rd(RG_SRC_CH);
}

/* One SPI command over GPI. cmd 1 = TX only, 3 = full duplex (rx gets n bytes). */
static int
rain_gpi_xfer(uint32_t cmd, const uint8_t *tx, uint8_t *rx, unsigned n)
{
	if (n == 0 || n > (cmd == 3u ? RM_RXMAX : RM_TXMAX) || !rg.uc) {
		return -30;
	}
	uint8_t *txb = (uint8_t *)(rg.mem + RM_TXBUF);
	struct rain_tre *txr = (struct rain_tre *)(rg.mem + RM_TX);
	struct rain_tre *rxr = (struct rain_tre *)(rg.mem + RM_RX);
	memcpy(txb, tx, n);
	if (cmd == 3u) {
		memset((void *)(rg.mem + RM_RXBUF), 0xa5, n);  /* untouched bytes read back as a5 */
		rain_tre_dma(&rxr[rg.rx_wp % RG_NTRE], rg.pa + RM_RXBUF, n, false);
	}
	rain_tre_config(&txr[rg.tx_wp % RG_NTRE], 16, rg.loopback);          /* 19.2 MHz / 16 */
	rain_tre_go(&txr[(rg.tx_wp + 1u) % RG_NTRE], cmd, cmd == 3u ? n : 0);
	rain_tre_dma(&txr[(rg.tx_wp + 2u) % RG_NTRE], rg.pa + RM_TXBUF, n, true);
	clean_dcache64((addr64_t)rg.pa, RM_EV, TRUE);              /* both TRE rings */
	clean_dcache64((addr64_t)(rg.pa + RM_TXBUF), RM_TXMAX, TRUE);
	flush_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);
	__asm__ volatile("dsb sy" ::: "memory");
	if (rn.xfers == 0 && rain_smmu < 0) {
		if (rain_bc(12, "smmu identity sid f6")) {
			return -40;
		}
		/* DT: qcom,gpi-dma@4a00000 iommus = <&apps_smmu 0xf6 0x0> */
		rain_smmu_pre();
		rain_smmu = topaz_smmu_identity(0xf6u, 0u, "GPI0") ? 1 : 0;
		r9.e3 = topaz_smmu_identity(0xe3u, 0u, "QUP0");
		if (!rain_smmu) {
			return -41;          /* no identity bank: never ring the doorbell */
		}
		rain_smmu_arm();
	}
	if (rn.xfers == 0 && rain_bc(13, "first doorbell (DMA)")) {
		return -40;
	}
	if (cmd == 3u) {
		rg.rx_wp = (rg.rx_wp + 1u) % RG_NTRE;
		rg_wr(RG_CH_DB(1), (uint32_t)(rg.pa + RM_RX + rg.rx_wp * 16u));
	}
	rg.tx_wp = (rg.tx_wp + 3u) % RG_NTRE;
	rg_wr(RG_CH_DB(0), (uint32_t)(rg.pa + RM_TX + rg.tx_wp * 16u));

	/* done = TX transfer-complete event (type 0x22, chid 0); duplex also needs the RX one (chid 1):
	 * the RX ring pointer moves when the TRE is fetched, before the data lands */
	unsigned need = 1u, got = 0;
	bool rx_done = cmd != 3u;
	volatile struct rain_tre *ev = (volatile struct rain_tre *)(rg.uc + RM_EV);   /* uncached: entries are zeroed after use */
	for (unsigned spin = 0; spin < 200000 && (got < need || !rx_done); spin++) {
		if (rain_smmu_check()) {
			break;
		}
		while (ev[rg.ev_rp].w[3] != 0) {
			uint32_t w3 = ev[rg.ev_rp].w[3];
			rg.ev_w2 = ev[rg.ev_rp].w[2];
			rg.ev_w3 = w3;
			rg.nev++;
			if (((w3 >> 16) & 0xffu) == 0x22u && (w3 >> 24) == 0u) {
				got++;
			}
			if ((w3 >> 24) == 1u) {
				rg.rx_w3 = w3;
				rg.rx_w0 = ev[rg.ev_rp].w[0];
				rg.rx_w2 = ev[rg.ev_rp].w[2];
				rx_done = true;
				/* IMMEDIATE_DATA (0x30): up to 8 RX bytes ride in the event, length in w2[3:0] */
				if (((w3 >> 16) & 0xffu) == 0x30u) {
					unsigned len = ev[rg.ev_rp].w[2] & 0xfu;
					uint32_t imm[2] = { ev[rg.ev_rp].w[0], ev[rg.ev_rp].w[1] };
					memcpy((void *)(rg.mem + RM_RXBUF), imm, len > 8u ? 8u : len);
					clean_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);
					rg.nimm++;
				}
			}
			ev[rg.ev_rp].w[3] = 0;
			rg.ev_rp = (rg.ev_rp + 1u) % RG_NEV;
		}
		if (got < need || !rx_done) {
			delay(1);
		}
	}
	/* hand the consumed events back; clear end-of-block status */
	rg_wr(RG_EV_DB, (uint32_t)(rg.pa + RM_EV + ((rg.ev_rp + RG_NEV - 1u) % RG_NEV) * 16u));
	rg_wr(RG_IEOB_CLR, 0xffffffffu);
	if (got < need || !rx_done) {
		rain_gpi_snap();
		return sm.seen ? -32 : -31;     /* -32: the SMMU recorded a fault */
	}
	if (cmd == 3u) {
		flush_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);
		memcpy(rg.cached, (const void *)(rg.mem + RM_RXBUF), 8);
		if (rg.uc) {
			for (unsigned i = 0; i < n; i++) {   /* byte reads: device memory, no memcpy */
				rx[i] = ((volatile uint8_t *)(rg.uc + RM_RXBUF))[i];
			}
		} else {
			memcpy(rx, (const void *)(rg.mem + RM_RXBUF), n);
		}
	}
	rn.xfers++;
	return 0;
}

/* One SPI command of n (<= 32) bytes, chip select held for the whole command. */
static int
rain_spi(uint32_t op, const uint8_t *tx, uint8_t *rx, unsigned n)
{
	vm_offset_t se = rn.se;
	if (rg.mem) {
		return rain_gpi_xfer(op == RAIN_OP_FULL ? 3u : 1u, tx, rx, n);
	}
	if (n == 0 || n > 32) {
		return -1;
	}
	if (touch_rd(se, SE_STATUS) & 1u) {
		return -2;                      /* M engine busy */
	}
	touch_wr(se, SE_IRQ_CLR, 0xffffffffu);
	touch_wr(se, SE_TX_LEN, n);         /* 8-bit words */
	touch_wr(se, SE_RX_LEN, op == RAIN_OP_FULL ? n : 0);
	touch_wr(se, SE_CMD, op << 27);
	for (unsigned i = 0; i < n; i += 4) {
		uint32_t w = 0;
		for (unsigned b = 0; b < 4 && i + b < n; b++) {
			w |= (uint32_t)tx[i + b] << (8 * b);   /* FIFO bytes LSB first */
		}
		touch_wr(se, SE_TX, w);
	}
	unsigned got = 0;
	for (unsigned spin = 0; spin < 200000; spin++) {
		uint32_t irq = touch_rd(se, SE_IRQ);
		while (op == RAIN_OP_FULL && got < n && (touch_rd(se, SE_RX_COUNT) & 0x1ffffffu)) {
			uint32_t w = touch_rd(se, SE_RX);
			for (unsigned b = 0; b < 4 && got < n; b++) {
				rx[got++] = (uint8_t)(w >> (8 * b));
			}
		}
		if (irq & SE_ERRORS & ~SE_NACK) {
			rain_snap(op, n);
			touch_wr(se, SE_IRQ_CLR, 0xffffffffu);
			return -3;
		}
		if ((irq & SE_DONE) && (op != RAIN_OP_FULL || got >= n)) {
			touch_wr(se, SE_IRQ_CLR, 0xffffffffu);
			rn.xfers++;
			return 0;
		}
		delay(1);
	}
	rain_snap(op, n);
	touch_wr(se, SE_CTRL, 4);           /* M cancel so the engine is idle again */
	delay(1000);
	return -4;                          /* timeout */
}

/* Novatek SPI: page select (0xFF, addr[22:15], addr[14:7]); write = addr[6:0]|0x80, data. */
static int
rain_nvt_write(uint32_t addr, uint8_t val)
{
	uint8_t page[3] = { 0xff, (uint8_t)(addr >> 15), (uint8_t)(addr >> 7) };
	uint8_t wr[2] = { (uint8_t)((addr & 0x7fu) | 0x80u), val };
	int e = rain_spi(RAIN_OP_TX, page, NULL, 3);
	return e ? e : rain_spi(RAIN_OP_TX, wr, NULL, 2);
}

/* read = addr[6:0] with bit 7 clear; the chip answers after one dummy byte. */
static int
rain_nvt_read(uint32_t addr, uint8_t *out, unsigned n)
{
	uint8_t page[3] = { 0xff, (uint8_t)(addr >> 15), (uint8_t)(addr >> 7) };
	static uint8_t tx[RM_RXMAX], rx[RM_RXMAX];
	bzero(tx, sizeof(tx));
	if (n + 2 > sizeof(tx)) {
		return -1;
	}
	int e = rain_spi(RAIN_OP_TX, page, NULL, 3);
	if (e) {
		return e;
	}
	tx[0] = (uint8_t)(addr & 0x7fu);
	e = rain_spi(RAIN_OP_FULL, tx, rx, n + 2);
	if (!e) {
		memcpy(out, rx + 2, n);
	}
	return e;
}

/* ---- C10: NT36525B firmware download (Xiaomi nt36xxx_fw_update.c, hw_crc = 1, NT36672A map) ---- */
#define NVT_EVENT_BUF    0x21c00u
#define NVT_BOOT_RDY     0x3f10du
#define NVT_ILM_LEN      0x3f118u
#define NVT_DLM_LEN      0x3f130u
#define NVT_ILM_DES      0x3f128u
#define NVT_DLM_DES      0x3f12cu
#define NVT_G_ILM_CRC    0x3f100u
#define NVT_G_DLM_CRC    0x3f104u
#define NVT_R_ILM_CRC    0x3f120u
#define NVT_R_DLM_CRC    0x3f124u
#define NVT_BLD_CRC_EN   0x3f30eu
#define NVT_CRC_FLAG     0x3f133u
#define NVT_CHUNK        4096u
#define NVT_MAXPART      24u

static struct {
	uint32_t fwpa, fwlen, nparts, tries, wrote;
	const volatile uint8_t *fw;           /* mapped firmware image, kept for reloads */
	uint32_t reloads, reload_fail;
	int err;
	uint8_t rst[5], crcflag;
	uint32_t rilm, rdlm;
	uint8_t info[12];
	uint32_t polls, downs, fx, fy, lx, ly;
	uint8_t raw[7];
	/* raintouchpanic=3: 3 s of statistics from the first contact */
	uint32_t t3_frames, t3_max, t3_ids, t3_empty;
	uint8_t t3_first[18];             /* first frame: up to 3 valid records */
	uint32_t t3_x0, t3_y0, t3_x1, t3_y1;   /* finger id 1: first and last panel position */
} nf;

static int
rain_nvt_page(uint32_t addr)
{
	uint8_t page[3] = { 0xff, (uint8_t)(addr >> 15), (uint8_t)(addr >> 7) };
	return rain_spi(RAIN_OP_TX, page, NULL, 3);
}

/* page select, then addr[6:0]|0x80 followed by data (len <= RM_TXMAX - 1) */
static int
rain_nvt_wbytes(uint32_t addr, const uint8_t *data, unsigned len, bool device_src)
{
	static uint8_t buf[RM_TXMAX];
	int e = rain_nvt_page(addr);
	if (e) {
		return e;
	}
	buf[0] = (uint8_t)((addr & 0x7fu) | 0x80u);
	for (unsigned i = 0; i < len; i++) {      /* byte copy: the firmware sits in device-mapped memory */
		buf[1 + i] = device_src ? ((const volatile uint8_t *)data)[i] : data[i];
	}
	return rain_spi(RAIN_OP_TX, buf, NULL, len + 1u);
}

static uint32_t
le32(const volatile uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int
rain_nvt_download(const volatile uint8_t *fw, uint32_t fwlen)
{
	uint32_t sram[NVT_MAXPART], size[NVT_MAXPART], bin[NVT_MAXPART], crc[2];
	uint32_t end = le32(fw), np = 0;
	if (fwlen < 0x100u || end > fwlen || (fw[0x20] & 2u)) {
		return -50;                               /* cascade header: not this chip */
	}
	for (uint32_t i = 0; i < 2; i++) {            /* ILM, DLM */
		bin[np] = le32(fw + 12u * i);
		sram[np] = le32(fw + 4u + 12u * i);
		size[np] = le32(fw + 8u + 12u * i);
		crc[i] = le32(fw + 0x18u + 4u * i);
		np++;
	}
	for (uint32_t pos = 0x30; pos < end && np < NVT_MAXPART; pos += 0x10) {   /* info sections */
		sram[np] = le32(fw + pos);
		size[np] = le32(fw + pos + 4u);
		bin[np] = le32(fw + pos + 8u);
		np++;
	}
	uint32_t novly = (fw[0x28] & 0x10u) ? (fw[0x28] & 0x0fu) : 0;
	for (uint32_t k = 0; k < novly && np < NVT_MAXPART; k++) {             /* overlays, listed at DLM start */
		uint32_t pos = bin[1] + 0x10u * k;
		if (pos + 12u > fwlen) {
			return -51;
		}
		sram[np] = le32(fw + pos);
		size[np] = le32(fw + pos + 4u);
		bin[np] = le32(fw + pos + 8u);
		np++;
	}
	nf.nparts = np;
	for (uint32_t i = 0; i < np; i++) {
		if (size[i] && bin[i] + size[i] + 1u > fwlen) {
			return -52;
		}
	}

	int e = 0;
	for (nf.tries = 1; nf.tries <= 3; nf.tries++) {
		nf.wrote = 0;
		/* bootloader reset */
		e = rain_nvt_write(RAIN_NVT_RST, 0x69);
		touch_wait(5);
		e = e ? e : rain_nvt_write(RAIN_NVT_RDFAST, 0x00);
		/* hw crc banks: destination (3 bytes), length (2 bytes), golden crc (4 bytes) */
		for (uint32_t i = 0; i < 2 && !e; i++) {
			uint8_t des[3] = { (uint8_t)sram[i], (uint8_t)(sram[i] >> 8), (uint8_t)(sram[i] >> 16) };
			uint8_t len[2] = { (uint8_t)size[i], (uint8_t)(size[i] >> 8) };
			uint8_t c[4] = { (uint8_t)crc[i], (uint8_t)(crc[i] >> 8), (uint8_t)(crc[i] >> 16), (uint8_t)(crc[i] >> 24) };
			e = rain_nvt_wbytes(i ? NVT_DLM_DES : NVT_ILM_DES, des, 3, false);
			e = e ? e : rain_nvt_wbytes(i ? NVT_DLM_LEN : NVT_ILM_LEN, len, 2, false);
			e = e ? e : rain_nvt_wbytes(i ? NVT_G_DLM_CRC : NVT_G_ILM_CRC, c, 4, false);
		}
		/* every non-empty partition: size + 1 bytes from bin to sram */
		for (uint32_t i = 0; i < np && !e; i++) {
			uint32_t left = size[i] ? size[i] + 1u : 0, a = sram[i], b = bin[i];
			while (left && !e) {
				uint32_t n = left < NVT_CHUNK ? left : NVT_CHUNK;
				e = rain_nvt_wbytes(a, (const uint8_t *)(fw + b), n, true);
				a += n;
				b += n;
				left -= n;
				nf.wrote += n;
			}
		}
		if (e) {
			return e;
		}
		/* enable bld crc, clear reset status, enable fw crc, boot ready */
		uint8_t v = 0;
		e = rain_nvt_read(NVT_BLD_CRC_EN, &v, 1);
		e = e ? e : rain_nvt_write(NVT_BLD_CRC_EN, (uint8_t)(v | 0x80u));
		e = e ? e : rain_nvt_write(NVT_EVENT_BUF | 0x60u, 0x00);
		e = e ? e : rain_nvt_write(NVT_EVENT_BUF | 0x50u, 0xae);
		e = e ? e : rain_nvt_write(NVT_BOOT_RDY, 1);
		if (e) {
			return e;
		}
		touch_wait(5);
		/* reset state 0xA0..0xAF = firmware up */
		for (unsigned r = 0; r <= 10; r++) {
			e = rain_nvt_read(NVT_EVENT_BUF | 0x60u, nf.rst, 5);
			if (!e && nf.rst[0] >= 0xa0u && nf.rst[0] <= 0xafu) {
				return 0;
			}
			touch_wait(10);
		}
		(void)rain_nvt_read(NVT_CRC_FLAG, &nf.crcflag, 1);
		uint8_t c4[4] = { 0 };
		(void)rain_nvt_read(NVT_R_ILM_CRC, c4, 4);
		nf.rilm = le32(c4);
		(void)rain_nvt_read(NVT_R_DLM_CRC, c4, 4);
		nf.rdlm = le32(c4);
		e = -53;
	}
	return e;
}

static void
rain_nvt_bringup(void)
{
	DTEntry mm;
	void const *prop = NULL;
	unsigned int psize = 0;
	rn.stage = "nvt fw find";
	if (SecureDTLookupEntry(NULL, "/chosen/memory-map", &mm) != kSuccess ||
	    SecureDTGetProperty(mm, "TouchFW", &prop, &psize) != kSuccess || psize < 16) {
		rn.err = -54;                             /* payload built without --blob TouchFW= */
		return;
	}
	nf.fwpa = (uint32_t)((const uint64_t *)prop)[0];
	nf.fwlen = (uint32_t)((const uint64_t *)prop)[1];
	flush_dcache64((addr64_t)((const uint64_t *)prop)[0], nf.fwlen, TRUE);   /* the loader's copy may still be in cache */
	vm_offset_t fw = ml_io_map((vm_offset_t)((const uint64_t *)prop)[0], (vm_size_t)nf.fwlen);
	if (!fw) {
		rn.err = -55;
		return;
	}
	rn.stage = "nvt fw download";
	nf.fw = (const volatile uint8_t *)fw;
	nf.err = rain_nvt_download(nf.fw, nf.fwlen);
	if (nf.err) {
		rn.err = nf.err;
		return;
	}
	rn.stage = "nvt fw info";
	touch_wait(50);
	uint8_t info[40];
	if (!rain_nvt_read(NVT_EVENT_BUF | 0x78u, info, 38)) {
		memcpy(nf.info, info, sizeof(nf.info));   /* ver, ~ver, x_num, y_num, xmax(2), ymax(2) ... */
	}

}

/*
 * C11: the driver loop. 10 fingers of 6 bytes at EVENT_BUF: id (1..10) << 3 | status
 * (1 enter, 2 move), x[11:4], y[11:4], x[3:0] << 4 | y[3:0]. Panel pixels 720x1650.
 * iOS's 1125x2436 canvas is drawn into 720x1559 at y = 45 (sweetfb_fit); sweet_hid_touch
 * wants 1080x2400 panel pixels and adds the canvas origin (20, 18) itself (SweetHID.cpp).
 */
static void rain_touch_report(void);

/*
 * The NT36525B is display driver and touch controller in one, and its touch firmware
 * lives in RAM: a panel reset or sleep (iOS taking the display, screen off/on) drops it.
 * Xiaomi's driver reloads on every resume. Here: once a second read the firmware info
 * (version + ~version = 0xff) and reload after two bad readings in a row.
 */
/*
 * Set by the framebuffer driver around DCS sleep in / sleep out. Sleep stops the touch
 * scan although the firmware info stays readable, so the liveness check alone misses it:
 * no polling while the panel is off, a fresh download when it comes back.
 */
static volatile int rain_panel_off, rain_panel_woke;

void rain_touch_panel(int on);

void
rain_touch_panel(int on)
{
	rain_panel_off = !on;
	if (on) {
		rain_panel_woke = 1;
	}
}

static bool
rain_nvt_alive(void)
{
	uint8_t v[2] = { 0, 0 };
	return !rain_nvt_read(NVT_EVENT_BUF | 0x78u, v, 2) && (uint8_t)(v[0] + v[1]) == 0xffu && v[0] != 0xffu && v[0] != 0;
}

static void
rain_nvt_reload(void)
{
	touch_report(0, st.x, st.y);                  /* lift every finger first */
	(void)rain_nvt_write(RAIN_NVT_ENGRST, 0x5a);
	touch_wait(1);
	int e = rain_nvt_download(nf.fw, nf.fwlen);
	nf.reloads++;
	nf.reload_fail += e != 0;
	printf("RAIN TOUCH: firmware reload %u err=%d\n", nf.reloads, e);
	touch_wait(50);
}

/* stop (raintouchpanic): 2 = panic at the first finger, 3 = panic 3 s after it with statistics */
static void
rain_touch_run(unsigned stop)
{
	uint8_t pd[60];
	unsigned errs = 0, tick = 0, dead = 0;
	rn.stage = "nvt run";
	printf("RAIN TOUCH: firmware %02x up, %ux%u, polling every 10 ms\n", nf.info[0],
	    ((uint32_t)nf.info[4] << 8) | nf.info[5], ((uint32_t)nf.info[6] << 8) | nf.info[7]);
	for (;;) {
		int x[TOUCH_MAX] = { 0 }, y[TOUCH_MAX] = { 0 };
		uint32_t mask = 0;
		if (rain_panel_off) {
			touch_report(0, st.x, st.y);
			touch_wait(50);
			continue;
		}
		if (rain_panel_woke) {
			rain_panel_woke = 0;
			touch_wait(150);                        /* display on follows sleep out by 120 ms */
			rain_nvt_reload();
			dead = 0;
		}
		if (++tick % 100u == 0) {
			dead = rain_nvt_alive() ? 0 : dead + 1;
			if (dead >= 2) {
				rain_nvt_reload();
				dead = 0;
			}
		}
		if (rain_nvt_read(NVT_EVENT_BUF, pd, sizeof(pd))) {
			if (++errs == 10) {
				touch_report(0, st.x, st.y);    /* release after persistent bad reads */
			}
			if (errs > 1000) {
				printf("RAIN TOUCH: giving up after %u read errors\n", errs);
				return;
			}
			touch_wait(10);
			continue;
		}
		errs = 0;
		nf.polls++;
		unsigned nvalid = 0;
		for (unsigned i = 0; i < TOUCH_MAX; i++) {
			const uint8_t *f = pd + 6u * i;
			unsigned id = f[0] >> 3, status = f[0] & 7u;
			unsigned px = ((unsigned)f[1] << 4) | (f[3] >> 4);
			unsigned py = ((unsigned)f[2] << 4) | (f[3] & 0x0fu);
			if (id < 1 || id > TOUCH_MAX || (status != 1 && status != 2) || px > 719 || py > 1649) {
				continue;
			}
			int hx = (int)(px * 1125u / 720u) - 20;
			int hy = (py < 45 ? 0 : (int)((py - 45u) * 2436u / 1559u)) - 18;
			x[id - 1] = hx < 0 ? 0 : (hx > 1079 ? 1079 : hx);
			y[id - 1] = hy < 0 ? 0 : (hy > 2399 ? 2399 : hy);
			if (!nf.t3_frames && nvalid < 3) {
				memcpy(nf.t3_first + 6u * nvalid, f, 6);
			}
			nvalid++;
			nf.t3_ids |= 1u << id;
			if (id == 1) {
				if (!nf.t3_frames) {
					nf.t3_x0 = px;
					nf.t3_y0 = py;
				}
				nf.t3_x1 = px;
				nf.t3_y1 = py;
			}
			mask |= 1u << (id - 1);
			if (!nf.downs++) {
				nf.fx = px;
				nf.fy = py;
				memcpy(nf.raw, f, 6);
				nf.raw[6] = (uint8_t)i;
			}
			nf.lx = (uint32_t)x[id - 1];
			nf.ly = (uint32_t)y[id - 1];
		}
		if (stop == 2 && mask) {
			rain_touch_report();
		}
		if (nf.downs) {
			nf.t3_frames++;
			nf.t3_empty += !mask;
			nf.t3_max = nvalid > nf.t3_max ? nvalid : nf.t3_max;
		}
		touch_report(mask, x, y);
		if (stop == 3 && nf.t3_frames >= 300) {
			rain_touch_report();
		}
		touch_wait(10);
	}
}

static void
rain_touch_probe(void)
{
	rn.stage = "map";
	vm_offset_t gcc = ml_io_map(TOUCH_GCC_PA, 0x80000);
	vm_offset_t west = ml_io_map(TOUCH_WEST_PA, 0x58000);
	rn.se = ml_io_map(RAIN_SE0_PA, 0x4000);
	if (!gcc || !west || !rn.se) {
		rn.err = -10;
		return;
	}
	st.gcc = gcc;
	st.west = west;

	rn.stage = "clocks";
	touch_wr(gcc, TOUCH_GCC_VOTE, touch_rd(gcc, TOUCH_GCC_VOTE) | RAIN_GCC_VOTE_SE0);
	rn.vote = touch_rd(gcc, TOUCH_GCC_VOTE);
	touch_wr(gcc, RAIN_RCG_SE0 + 4u, 1u);               /* XO, divider 1: 19.2 MHz */
	touch_wr(gcc, RAIN_RCG_SE0, touch_rd(gcc, RAIN_RCG_SE0) | 1u);
	if (!touch_wait_clear(gcc, RAIN_RCG_SE0, 1u)) {
		rn.err = -11;
		return;
	}
	rn.rcgcfg = touch_rd(gcc, RAIN_RCG_SE0 + 4u);
	rn.cbcr = touch_rd(gcc, RAIN_RCG_SE0 - 4u);
	if (!touch_wait_clear(gcc, RAIN_RCG_SE0 - 4u, 1u << 31)) {
		rn.err = -12;                   /* SE0 branch clock off */
		return;
	}

	rn.stage = "se0 fw";
	rn.fw = touch_rd(rn.se, SE_FW);
	rn.hwtx = touch_rd(rn.se, SE_HW_TX);
	rn.hwrx = touch_rd(rn.se, SE_HW_RX);
	rn.status = touch_rd(rn.se, SE_STATUS);
	rn.ifdis = touch_rd(rn.se, 0x64);        /* GENI_IF_DISABLE_RO: bit 0 FIFO_IF_DISABLE */
	rn.fws = touch_rd(rn.se, 0x6c);          /* GENI_S_FW_REVISION_RO */
	rn.seqcfg = touch_rd(rn.se, 0x20);       /* GENI_FORCE_DEFAULT as left by ABL */
	if (((rn.fw >> 8) & 0xffu) != 1u) {
		rn.err = -13;                   /* not SPI firmware: touch nothing else */
		return;
	}
	if (rn.status & 0x1001u) {
		rn.err = -14;                   /* engine busy */
		return;
	}
	if (rn.ifdis & 1u) {
		/* FIFO interface off: GPI DMA only */
		int ge = rain_gpi_setup();
		if (ge) {
			rn.err = ge;
			rn.stage = rg.gstage;
			return;
		}
	}

	rn.stage = "pins";
	for (unsigned p = 0; p < 4; p++) {
		uint32_t cfg = touch_rd(west, TOUCH_GPIO_CFG(p));
		rn.pin_before[p] = cfg;
		if (((cfg >> 2) & 0xfu) != 1u) {
			touch_gpio(p, 1, 0, 2, false, false);   /* qup0, no pull, 6 mA */
		}
		rn.pin_after[p] = touch_rd(west, TOUCH_GPIO_CFG(p));
	}

	rn.stage = "se0 spi init";
	if (!rg.mem) {
	touch_wr(rn.se, 0xe18, 0);          /* no GSI */
	touch_wr(rn.se, 0xe1c, 0);
	touch_wr(rn.se, 0x644, 0);
	touch_wr(rn.se, SE_IRQ_CLR, 0xffffffffu);
	touch_wr(rn.se, 0x648, 0xffffffffu);
	touch_wr(rn.se, SE_DMA, 0);         /* FIFO mode */
	touch_wr(rn.se, 0x28, touch_rd(rn.se, 0x28) | 0x7fu);
	touch_wr(rn.se, 0xe30, touch_rd(rn.se, 0xe30) | 0x0fu);
	touch_wr(rn.se, 0x24, 0x7f);
	touch_wr(rn.se, 0x20, 1);
	touch_wr(rn.se, SE_CLK_SEL, 0);
	touch_wr(rn.se, SE_MCLK, (16u << 4) | 1u);         /* 19.2 MHz / 16 = 1.2 MHz */
	touch_wr(rn.se, RAIN_SPI_CPHA, 0);                 /* SPI mode 0 */
	touch_wr(rn.se, RAIN_SPI_CPOL, 0);
	touch_wr(rn.se, RAIN_SPI_LOOPBACK, 0);
	touch_wr(rn.se, RAIN_SPI_DEMUXSEL, 0);             /* CS0 */
	touch_wr(rn.se, RAIN_SPI_DEMUXINV, 0);
	touch_wr(rn.se, RAIN_SPI_TRANSCFG, 0);             /* CS held for a command */
	touch_wr(rn.se, RAIN_SPI_WORDLEN, 8u - 4u);        /* 8-bit words */
	touch_wr(rn.se, RAIN_SPI_PREPOST, 0);
	touch_wr(rn.se, RAIN_SPI_DELAYS, 0);
	touch_wr(rn.se, 0x254, 0);
	touch_wr(rn.se, 0x260, SE_PACK0);
	touch_wr(rn.se, 0x264, SE_PACK1);
	touch_wr(rn.se, 0x284, SE_PACK0);
	touch_wr(rn.se, 0x288, SE_PACK1);
	touch_wr(rn.se, SE_TX_WM, 0);
	touch_wr(rn.se, 0x810, 0);
	touch_wr(rn.se, 0x814, ((rn.hwrx >> 16) & 0x3fu) - 1u);
	touch_wr(rn.se, SE_IRQ_EN, SE_EVENTS);
	}
	rn.c_mclk = touch_rd(rn.se, SE_MCLK);
	rn.c_clksel = touch_rd(rn.se, SE_CLK_SEL);
	rn.c_word = touch_rd(rn.se, RAIN_SPI_WORDLEN);
	rn.c_cpha = touch_rd(rn.se, RAIN_SPI_CPHA);
	rn.c_cpol = touch_rd(rn.se, RAIN_SPI_CPOL);
	rn.c_demux = touch_rd(rn.se, RAIN_SPI_DEMUXSEL);
	rn.c_trans = touch_rd(rn.se, RAIN_SPI_TRANSCFG);
	rn.c_cgc = touch_rd(rn.se, 0x28);
	rn.c_dma = touch_rd(rn.se, SE_DMA);
	rn.c_pack0 = touch_rd(rn.se, 0x260);
	rn.cbcr = touch_rd(st.gcc, RAIN_RCG_SE0 - 4u);   /* branch state after the wait */
	rn.k_cmd = touch_rd(st.gcc, RAIN_RCG_SE0);          /* bit 31 ROOT_OFF, bit 1 ROOT_EN */
	rn.k_b138 = touch_rd(st.gcc, 0x1f138u);            /* candidate SE0 branch (halt) register */
	rn.k_b144 = touch_rd(st.gcc, 0x1f144u);
	rn.k_se2cmd = touch_rd(st.gcc, TOUCH_RCG);         /* topaz SE2, known good, read only */
	rn.k_se2cfg = touch_rd(st.gcc, TOUCH_RCG + 4u);

	if (rg.mem) {
		static const uint8_t pat[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
		rn.stage = "loopback";
		rg.loopback = true;
		(void)rain_spi(RAIN_OP_FULL, pat, rg.lb, sizeof(pat));
		rg.loopback = false;
	}
	rn.stage = "nvt eng reset";
	int e = rain_nvt_write(RAIN_NVT_ENGRST, 0x5a);
	if (e) {
		rn.err = e;
		return;
	}
	touch_wait(1);
	rn.stage = "nvt reset";
	e = rain_nvt_write(RAIN_NVT_RST, 0x69);
	if (e == -4) {
		rn.stage = "retry root_en";
		touch_wr(st.gcc, RAIN_RCG_SE0, touch_rd(st.gcc, RAIN_RCG_SE0) | 2u);   /* ROOT_EN */
		delay(100);
		rn.k_cmd2 = touch_rd(st.gcc, RAIN_RCG_SE0);
		e = rain_nvt_write(RAIN_NVT_RST, 0x69);
		rn.retry_err = e;
		rn.stage = "nvt reset";
	}
	touch_wait(5);
	if (!e) {
		e = rain_nvt_write(RAIN_NVT_RDFAST, 0x00);
	}
	if (e) {
		rn.err = e;
		return;
	}
	rn.stage = "nvt id";
	int e1 = rain_nvt_read(RAIN_NVT_TRIM, rn.id, 6);
	int e2 = rain_nvt_read(RAIN_NVT_TRIM_OLD, rn.id_old, 6);
	rn.err = e1 ? e1 : e2;
	if (!rn.err && rn.id[0] == 0x0bu && rn.id[3] == 0x25u && rn.id[4] == 0x65u && rn.id[5] == 0x03u) {
		rain_nvt_bringup();
		if (rn.err) {
			return;
		}
	}
	rn.stage = "done";
}

/*
 * 0969: raintouchfb=1 ends the probe by rebooting into fastboot instead of
 * panicking, so the outcome is readable with the panel off or covered by
 * iOS: fastboot = stop point reached, other reboot = crash, none = hang.
 * Same method as the shim: IMEM restart reason "bootloader", then PSCI
 * SYSTEM_RESET, then PS_HOLD low as a fallback.
 */
static void
rain_reboot_fastboot(void)
{
	vm_offset_t imem = ml_io_map(0x0c125000u, 0x1000);   /* qcom,msm-imem restart_reason@65c */
	vm_offset_t pshold = ml_io_map(0x0440b000u, 0x1000);
	if (imem) {
		touch_wr(imem, 0x65cu, 0x77665500u);
	}
	__asm__ volatile("dsb sy" ::: "memory");
	register uint64_t x0 __asm__("x0") = 0x84000009u;    /* PSCI SYSTEM_RESET */
	__asm__ volatile("smc #0" : "+r"(x0) : : "x1", "x2", "x3", "memory");
	if (pshold) {
		touch_wr(pshold, 0, 0);
	}
	for (;;) {
		__asm__ volatile("wfe");
	}
}

static void
rain_touch_report(void)
{
	uint32_t fb = 0;
	(void)PE_parse_boot_argn("raintouchfb", &fb, sizeof(fb));
	if (fb) {
		printf("RAIN TOUCH: step %u err=%d, rebooting to fastboot\n", rain_step, rn.err);
		rain_reboot_fastboot();
	}
	rain_smmu_cbscan();
	panic("RAIN TOUCH C14: step %u '%s' err=%d xf=%u smmu=%d cb=%d | SMMU cr0 %#x->%#x gfsr=%#x gfsynr0=%#x gfsynr1(sid)=%#x gfsynr2=%#x gfar=%#x%08x cbfsr=%#x cbfar=%#x%08x cbfsynr0=%#x | ifdis=%#x | GPI pre ch0=%#x ch1=%#x ev=%#x | ev=%#x ch0=%#x ch1=%#x "
	    "go0=%#x go1=%#x | FAIL ch0=%#x ch1=%#x rp0=%#x wp0=%#x evrp=%#x type=%#x glob=%#x gpii=%#x "
	    "err=%#x src=%#x | ev n=%u w2=%#x w3=%#x rx=%#x %#x %#x imm=%u uc=%d pa=%#llx | cached %02x %02x %02x %02x | id %02x %02x %02x %02x %02x %02x | old %02x %02x %02x %02x %02x %02x | lb %02x %02x %02x %02x %02x %02x %02x %02x | pins %x %x %x %x | e3 nv=%u slot=%u smr=%#x s2cr=%#x sctlr=%#x id=%d | cbf %d fsr=%#x far=%#x syn=%#x sid=%#x | FW pa=%#x len=%#x parts=%u try=%u wrote=%#x err=%d rst %02x %02x crcflag=%#x rilm=%#x rdlm=%#x | INFO %02x %02x x%u y%u max %u %u | TOUCH polls=%u downs=%u panel %u,%u hid %u,%u raw %02x %02x %02x %02x %02x %02x %02x | reloads=%u fail=%u | T3 frames=%u empty=%u maxfingers=%u ids=%#x id1 %u,%u -> %u,%u hid down=%u up=%u | first %02x %02x %02x %02x %02x %02x / %02x %02x %02x %02x %02x %02x / %02x %02x %02x %02x %02x %02x",
	    rain_step, rn.stage ? rn.stage : "?", rn.err, rn.xfers, rain_smmu, sm.cb,
	    sm.cr0_was, sm.cr0_now, sm.gfsr, sm.gfsynr0, sm.gfsynr1, sm.gfsynr2, sm.gfar_hi, sm.gfar_lo,
	    sm.cbfsr, sm.cbfar_hi, sm.cbfar_lo, sm.cbfsynr0, rn.ifdis,
	    rg.pre_ch0, rg.pre_ch1, rg.pre_ev, rg.st_ev, rg.st_ch0, rg.st_ch1, rg.st_go0, rg.st_go1,
	    rg.f_ch0, rg.f_ch1, rg.f_rp0, rg.f_wp0, rg.f_evrp, rg.f_type, rg.f_glob, rg.f_gpii,
	    rg.f_err, rg.f_srcch, rg.nev, rg.ev_w2, rg.ev_w3, rg.rx_w3, rg.rx_w0, rg.rx_w2, rg.nimm, rg.uc != 0, (unsigned long long)rg.pa,
	    rg.cached[0], rg.cached[1], rg.cached[2], rg.cached[3],
	    rn.id[0], rn.id[1], rn.id[2], rn.id[3], rn.id[4], rn.id[5],
	    rn.id_old[0], rn.id_old[1], rn.id_old[2], rn.id_old[3], rn.id_old[4], rn.id_old[5],
	    rg.lb[0], rg.lb[1], rg.lb[2], rg.lb[3], rg.lb[4], rg.lb[5], rg.lb[6], rg.lb[7],
	    rn.pin_after[0], rn.pin_after[1], rn.pin_after[2], rn.pin_after[3],
	    r9.nvalid, r9.slot, r9.smr, r9.s2cr, r9.sctlr, r9.e3, r9.fcb, r9.ffsr, r9.ffar, r9.fsyn, r9.fsid,
	    nf.fwpa, nf.fwlen, nf.nparts, nf.tries, nf.wrote, nf.err, nf.rst[0], nf.rst[1], nf.crcflag, nf.rilm, nf.rdlm,
	    nf.info[0], nf.info[1], nf.info[2], nf.info[3], ((uint32_t)nf.info[4] << 8) | nf.info[5], ((uint32_t)nf.info[6] << 8) | nf.info[7],
	    nf.polls, nf.downs, nf.fx, nf.fy, nf.lx, nf.ly,
	    nf.raw[0], nf.raw[1], nf.raw[2], nf.raw[3], nf.raw[4], nf.raw[5], nf.raw[6],
	    nf.reloads, nf.reload_fail, nf.t3_frames, nf.t3_empty, nf.t3_max, nf.t3_ids, nf.t3_x0, nf.t3_y0, nf.t3_x1, nf.t3_y1, st.s_down, st.s_up,
	    nf.t3_first[0], nf.t3_first[1], nf.t3_first[2], nf.t3_first[3], nf.t3_first[4], nf.t3_first[5],
	    nf.t3_first[6], nf.t3_first[7], nf.t3_first[8], nf.t3_first[9], nf.t3_first[10], nf.t3_first[11],
	    nf.t3_first[12], nf.t3_first[13], nf.t3_first[14], nf.t3_first[15], nf.t3_first[16], nf.t3_first[17]);
}

static void
touch_thread(void *arg, wait_result_t result)
{
	unsigned delay_s = 45;
	(void)arg;
	(void)result;
	/*
	 * Start late (boot-arg sweettouchdelay, seconds): if the bus access
	 * below hangs the SoC, the persistent log and the GUI got there first.
	 */
	(void)PE_parse_boot_argn("sweettouchdelay", &delay_s, sizeof(delay_s));
	printf("SWEET TOUCH: starting in %u s\n", delay_s);
	touch_wait(delay_s * 1000u);
	if (rain_touch_on) {
		uint32_t stop = 0;
		(void)PE_parse_boot_argn("raintouchpanic", &stop, sizeof(stop));
		rain_touch_probe();
		if (rn.err || !nf.wrote || stop == 1) {
			rain_touch_report();    /* panics with every reading */
		}
		rain_touch_run(stop);
		goto out;
	}
	st.stage = "map";
	printf("SWEET TOUCH: mapping GCC/TLMM/SE2\n");
	st.gcc = ml_io_map(TOUCH_GCC_PA, 0x80000);
	st.west = ml_io_map(TOUCH_WEST_PA, 0x58000);
	st.east = ml_io_map(TOUCH_EAST_PA, 0x25000);
	st.se = ml_io_map(TOUCH_SE_PA, 0x4000);
	if (!st.gcc || !st.west || !st.east || !st.se) {
		printf("SWEET TOUCH: MMIO mapping failed\n");
		goto out;
	}
	st.stage = "clocks";
	if (!touch_clocks()) {
		goto out;
	}
	st.stage = "se2 init";
	if (!touch_se_init()) {
		goto out;
	}
	st.stage = "power/reset";
	printf("SWEET TOUCH: TLMM reset=%#x irq=%#x avdd=%#x\n",
	    touch_rd(st.west, TOUCH_GPIO_IO(TOUCH_PIN_RESET)),
	    touch_rd(st.west, TOUCH_GPIO_IO(TOUCH_PIN_IRQ)),
	    touch_rd(st.east, TOUCH_GPIO_IO(TOUCH_PIN_AVDD)));
	/* TopazTouch touch.c: IRQ input, AVDD on, I2C pins, then the reset pulse. */
	touch_gpio(TOUCH_PIN_IRQ, 0, 3, 3, false, false);       /* CTL 0xc3 */
	touch_gpio(TOUCH_PIN_AVDD, 0, 3, 3, true, true);        /* CTL 0x2c3, high */
	touch_gpio(6, 1, 0, 0, false, false);                   /* qup2 SDA, CTL 0x4 */
	touch_gpio(7, 1, 0, 0, false, false);                   /* qup2 SCL */
	touch_gpio(TOUCH_PIN_RESET, 0, 3, 3, true, false);      /* CTL 0x2c3, low */
	touch_wait(20);
	touch_gpio(TOUCH_PIN_RESET, 0, 3, 3, true, true);
	touch_wait(250);
	st.stage = "probe";
	uint8_t gid[32], legacy[32], fid1 = 0, fid2 = 0;
	st.probing = true;
	int ge = touch_probe_info(GOODIX_PID_REG, gid);
	printf("SWEET TOUCH: probe 0x5d err=%d pid=%02x%02x%02x%02x fw=%02x.%02x.%02x.%02x irq=%#x\n",
	    ge, gid[0], gid[1], gid[2], gid[3], gid[8], gid[9], gid[10], gid[11], st.irq_status);
	(void)touch_probe_info(0x8240, legacy);
	int fe1 = touch_read(0x38, 0xa3, false, &fid1, 1);
	int fe2 = touch_read(0x38, 0x9f, false, &fid2, 1);
	printf("SWEET TOUCH: probe 0x38 err=%d/%d chip-id=%02x:%02x irq=%#x\n",
	    fe1, fe2, fid1, fid2, st.irq_status);
	{
		uint8_t fw = 0, vendor = 0, mode = 0, pmode = 0, lib[2] = {0, 0};
		int e1 = touch_read(0x38, 0xa6, false, &fw, 1);
		int e2 = touch_read(0x38, 0xa8, false, &vendor, 1);
		int e3 = touch_read(0x38, 0x00, false, &mode, 1);
		int e4 = touch_read(0x38, 0xa5, false, &pmode, 1);
		int e5 = touch_read(0x38, 0xa1, false, lib, 2);
		printf("SWEET TOUCH: ft fw=%02x/%d vendor=%02x/%d devmode=%02x/%d pmode=%02x/%d lib=%02x%02x/%d\n",
		    fw, e1, vendor, e2, mode, e3, pmode, e4, lib[0], lib[1], e5);
	}
	st.probing = false;
	st.d_ge = ge;
	st.d_fe1 = fe1;
	st.d_fe2 = fe2;
	st.d_fid1 = fid1;
	st.d_fid2 = fid2;
	bool goodix = !ge && gid[0] == '9' && gid[1] == '8' && gid[2] == '9' && gid[3] == '6';
	bool fts = !fe1 && !fe2 && fid1 == 0x54 && fid2 == 0x52;
	if (st.dead || goodix == fts) {
		printf("SWEET TOUCH: no unique supported IC (goodix=%d fts=%d dead=%d); stopping\n",
		    goodix, fts, st.dead);
		goto out;
	}
	st.kind = goodix ? TOUCH_GOODIX : TOUCH_FTS;
	st.stage = "polling";
	printf("SWEET TOUCH: selected %s, 1080x2400, poll=10ms irq-hint=%d\n",
	    goodix ? "GT9896 Yellowstone" : "Focaltech 54:52", st.irq_hint);
	unsigned errors = 0, ticks = 0, polls = 0;
	while (!st.dead) {
		if (st.diag == 2 && ++polls == 3000) {  /* ~30 s at 10 ms */
			panic("SWEET TOUCH stats: frames=%u errs=%u lasterr=%d maxn=%u hid down=%u up=%u last=%d,%d | "
			    "raw %02x %02x %02x | %02x %02x %02x %02x %02x %02x | %02x %02x %02x",
			    st.s_frames, st.s_errs, st.s_lasterr, st.s_maxn, st.s_down, st.s_up,
			    st.s_lastx, st.s_lasty, st.s_raw[0], st.s_raw[1], st.s_raw[2],
			    st.s_raw[3], st.s_raw[4], st.s_raw[5], st.s_raw[6], st.s_raw[7],
			    st.s_raw[8], st.s_raw[9], st.s_raw[10], st.s_raw[11]);
		}
		/* Optional IRQ hint still reads every 100ms, and always while down. */
		if (!st.irq_hint || st.active ||
		    !(touch_rd(st.west, TOUCH_GPIO_IO(TOUCH_PIN_IRQ)) & 1u) ||
		    (++ticks % 10u) == 0) {
			int err = st.kind == TOUCH_GOODIX ? touch_goodix() : touch_focaltech();
			if (err) {
				st.s_errs++;
				st.s_lasterr = err;
				if ((errors++ % 100u) == 0) {
					printf("SWEET TOUCH: report err=%d irq=%#x\n", err, st.irq_status);
				}
				if (errors == 10) {
					touch_report(0, st.x, st.y); /* release after persistent bad reads */
				}
			} else {
				errors = 0;
			}
		}
		touch_wait(10);
	}
	touch_report(0, st.x, st.y);
out:
	if (st.diag) {
		panic("SWEET TOUCH diag: stopped at '%s' dead=%d | gcc vote=%#x cfg=%#x cbcr=%#x | "
		    "se2 fw=%#x status=%#x hwtx=%#x hwrx=%#x | goodix err=%d | ft 0x38 err=%d/%d id=%02x:%02x | irq=%#x",
		    st.stage ? st.stage : "?", st.dead, st.d_vote, st.d_cfg, st.d_cbcr,
		    st.d_fw, st.d_status, st.d_hwtx, st.d_hwrx, st.d_ge,
		    st.d_fe1, st.d_fe2, st.d_fid1, st.d_fid2, st.irq_status);
	}
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

void
sweet_touch_init(void)
{
	int enabled = 1;
	thread_t thread;
	if (st.started) {
		return;
	}
	st.started = true;
	(void)PE_parse_boot_argn("sweettouch", &enabled, sizeof(enabled));
	st.irq_hint = 1; /* topaz: read on IRQ GPIO80 low, while touching, else every 100 ms */
	(void)PE_parse_boot_argn("sweettouchirq", &st.irq_hint, sizeof(st.irq_hint));
	(void)PE_parse_boot_argn("sweettouchdiag", &st.diag, sizeof(st.diag));
	(void)PE_parse_boot_argn("raintouch", &rain_touch_on, sizeof(rain_touch_on));
	if (!enabled && !rain_touch_on) {
		printf("SWEET TOUCH: disabled (sweettouch=0)\n");
		return;
	}
	kern_return_t kr = kernel_thread_start(touch_thread, NULL, &thread);
	if (kr == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("SWEET TOUCH: kernel_thread_start failed %d\n", kr);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
