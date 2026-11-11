/*
 * topaz: polled FIFO I2C on QUP0 SE1 (battery bus: SM5602 fuel gauge 0x71,
 * bq2589x-class charger 0x6a). Values from MakrSas/topaz-woa TopazBattery
 * (hw.h, hwio.c) and Linux i2c-qcom-geni.c. Read-only use by SweetBattery.
 * The SE1 RCG is not touched: DFS level 0 already runs from XO (TopazBattery).
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/locks.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
int sweet_i2c1_init(void);
int sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len);
int sweet_i2c1_write(uint8_t addr, uint8_t reg, uint8_t val);

#define I2C1_SE_PA      0x04a84000u
#define I2C1_GCC_PA     0x01400000u
#define I2C1_WEST_PA    0x00500000u     /* TLMM WEST tile: GPIO 4/5 */
#define I2C1_GCC_VOTE   0x7900cu        /* QUP0 wrapper + SE1 vote, bits 0xbc0 */
#define I2C1_CBCR       0x1f274u        /* qupv3_wrap0_s1 branch */

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
#define SE_TX_WM        0x80cu
#define SE_HW_RX        0xe28u
#define SE_DONE         (1u << 0)
#define SE_NACK         (1u << 10)
#define SE_TX_WM_IRQ    (1u << 30)
#define SE_ERRORS       (0x3eu | (1u << 9) | (1u << 10) | (1u << 12) | (1u << 13))
#define SE_PACK0        0x0007f8feu
#define SE_PACK1        0x000ffefeu

static vm_offset_t i2c1_se, i2c1_gcc, i2c1_west;
static bool i2c1_ready;
static int i2c1_engine(void);
static void sweet_i2c_scan(void);
static LCK_GRP_DECLARE(sweet_i2c1_grp, "sweet_i2c1");
static LCK_MTX_DECLARE(sweet_i2c1_lock, &sweet_i2c1_grp);

static uint32_t
rd(vm_offset_t b, uint32_t o)
{
	uint32_t v = *(volatile uint32_t *)(b + o);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
wr(vm_offset_t b, uint32_t o, uint32_t v)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(b + o) = v;
	__asm__ volatile("dsb osh" ::: "memory");
}

static bool
wait_clear(vm_offset_t b, uint32_t o, uint32_t m)
{
	for (unsigned i = 0; i < 5000; ++i) {
		if (!(rd(b, o) & m)) {
			return true;
		}
		delay(1);
	}
	return false;
}

static void
i2c1_abort(void)
{
	wr(i2c1_se, SE_CTRL, 4);                /* cancel */
	for (unsigned i = 0; i < 1000 && !(rd(i2c1_se, SE_IRQ) & (1u << 4)); ++i) {
		delay(1);
	}
	if (!(rd(i2c1_se, SE_IRQ) & (1u << 4))) {
		wr(i2c1_se, SE_CTRL, 2);        /* abort */
		for (unsigned i = 0; i < 1000 && !(rd(i2c1_se, SE_IRQ) & (1u << 5)); ++i) {
			delay(1);
		}
	}
	for (unsigned i = 0; i < 64 && (rd(i2c1_se, SE_RX_COUNT) & 0x1ffffffu); ++i) {
		(void)rd(i2c1_se, SE_RX);
	}
	wr(i2c1_se, SE_IRQ_CLR, 0xffffffffu);
}

/* One message: write nt (<= 3) bytes or read nr (<= 64) bytes. */
static int
i2c1_msg(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx, unsigned nr, bool stretch)
{
	unsigned sent = 0, got = 0;
	if (!wait_clear(i2c1_se, SE_STATUS, 1u)) {
		return EBUSY;
	}
	wr(i2c1_se, SE_IRQ_CLR, 0xffffffffu);
	wr(i2c1_se, SE_TX_WM, nt ? 1u : 0u);
	wr(i2c1_se, nr ? SE_RX_LEN : SE_TX_LEN, nr ? nr : nt);
	wr(i2c1_se, SE_CMD, ((nr ? 2u : 1u) << 27) | ((uint32_t)addr << 9) | (stretch ? 4u : 0u));
	for (unsigned i = 0; i < 20000; ++i) {
		uint32_t irq = rd(i2c1_se, SE_IRQ);
		if (irq & SE_ERRORS) {
			int err = (irq & SE_NACK) ? ENXIO : EIO;
			for (unsigned j = 0; j < 2000 && !(irq & SE_DONE); ++j) {
				delay(1);
				irq = rd(i2c1_se, SE_IRQ);
			}
			if (!(irq & SE_DONE) || (rd(i2c1_se, SE_STATUS) & 1u)) {
				i2c1_abort();
			}
			wr(i2c1_se, SE_IRQ_CLR, 0xffffffffu);
			return err;
		}
		if (nt && !sent && (irq & SE_TX_WM_IRQ)) {
			uint32_t word = 0;
			for (unsigned j = 0; j < nt; ++j) {
				word |= (uint32_t)tx[j] << (8u * j);
			}
			wr(i2c1_se, SE_TX, word);
			sent = nt;
			wr(i2c1_se, SE_TX_WM, 0);
		}
		uint32_t count = rd(i2c1_se, SE_RX_COUNT) & 0x1ffffffu;
		if (count > 16) {
			break;
		}
		while (count--) {
			uint32_t word = rd(i2c1_se, SE_RX);
			for (unsigned j = 0; j < 4 && got < nr; ++j) {
				rx[got++] = (uint8_t)(word >> (8u * j));
			}
		}
		if (irq & SE_DONE) {
			wr(i2c1_se, SE_IRQ_CLR, 0xffffffffu);
			return (got == nr && sent == nt) ? 0 : EIO;
		}
		wr(i2c1_se, SE_IRQ_CLR, irq & ~SE_DONE);
		delay(1);
	}
	i2c1_abort();
	return ETIMEDOUT;
}

int
sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len)
{
	if (!i2c1_ready || len == 0 || len > 64) {
		return ENXIO;
	}
	lck_mtx_lock(&sweet_i2c1_lock);
	int err = i2c1_msg(addr, &reg, 1, NULL, 0, true);
	if (!err) {
		err = i2c1_msg(addr, NULL, 0, buf, len, false);
	}
	lck_mtx_unlock(&sweet_i2c1_lock);
	return err;
}

/* One register write (TopazAudio: the sia8159 speaker amp at 0x2b). */
int
sweet_i2c1_write(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = { reg, val };

	if (!i2c1_ready) {
		return ENXIO;
	}
	lck_mtx_lock(&sweet_i2c1_lock);
	int err = i2c1_msg(addr, tx, 2, NULL, 0, false);
	lck_mtx_unlock(&sweet_i2c1_lock);
	return err;
}

int
sweet_i2c1_init(void)
{
	if (i2c1_ready) {
		return 0;
	}
	i2c1_gcc = ml_io_map(I2C1_GCC_PA, 0x80000);
	i2c1_west = ml_io_map(I2C1_WEST_PA, 0x8000);   /* rain: up to GPIO7 for the SE2 scan */
	i2c1_se = ml_io_map(I2C1_SE_PA, 0x4000);
	if (!i2c1_gcc || !i2c1_west || !i2c1_se) {
		return ENOMEM;
	}
	/* OR our votes in: other bits of the shared register belong to others */
	wr(i2c1_gcc, I2C1_GCC_VOTE, rd(i2c1_gcc, I2C1_GCC_VOTE) | 0xbc0u);
	if (!wait_clear(i2c1_gcc, I2C1_CBCR, 1u << 31)) {
		printf("SWEET I2C1: SE1 clock off (cbcr %#x)\n", rd(i2c1_gcc, I2C1_CBCR));
		return EIO;
	}
	/* GPIO4/5: function 1 (qup1), 2 mA, no pull (TopazBattery CTL 0x4) */
	wr(i2c1_west, 0x4000, (rd(i2c1_west, 0x4000) & ~0x3ffu) | 0x4u);
	wr(i2c1_west, 0x5000, (rd(i2c1_west, 0x5000) & ~0x3ffu) | 0x4u);

	int e = i2c1_engine();
	if (e) {
		return e;
	}
	i2c1_ready = true;
	printf("SWEET I2C1: SE1 ready (fw %#x vote %#x)\n", rd(i2c1_se, SE_FW), rd(i2c1_gcc, I2C1_GCC_VOTE));
	uint32_t scan = 0;
	if (PE_parse_boot_argn("raini2cscan", &scan, sizeof(scan)) && scan) {
		sweet_i2c_scan();
	}
	return 0;
}

/* GENI I2C engine set-up on i2c1_se (clocks and pins already on). */
static int
i2c1_engine(void)
{
	uint32_t fw = rd(i2c1_se, SE_FW), rxp = rd(i2c1_se, SE_HW_RX);
	if (((fw >> 8) & 0xffu) != 3u || (rd(i2c1_se, SE_STATUS) & 0x1001u)) {
		printf("SWEET I2C1: not an idle I2C engine (fw %#x status %#x)\n",
		    fw, rd(i2c1_se, SE_STATUS));
		return EIO;
	}
	wr(i2c1_se, 0xe18, 0);
	wr(i2c1_se, 0xe1c, 0);
	wr(i2c1_se, 0x644, 0);
	wr(i2c1_se, SE_IRQ_CLR, 0xffffffffu);
	wr(i2c1_se, 0x648, 0xffffffffu);
	wr(i2c1_se, SE_DMA, 0);
	wr(i2c1_se, 0x28, rd(i2c1_se, 0x28) | 0x7fu);
	wr(i2c1_se, 0xe30, rd(i2c1_se, 0xe30) | 0x0fu);
	wr(i2c1_se, 0x24, 0x7f);
	wr(i2c1_se, 0x20, 1);
	wr(i2c1_se, SE_CLK_SEL, 0);
	wr(i2c1_se, SE_MCLK, (2u << 4) | 1u);                  /* 400 kHz from 19.2 MHz */
	wr(i2c1_se, SE_SCL, (5u << 20) | (11u << 10) | 22u);
	wr(i2c1_se, 0x254, 0);
	wr(i2c1_se, 0x260, SE_PACK0);
	wr(i2c1_se, 0x264, SE_PACK1);
	wr(i2c1_se, 0x284, SE_PACK0);
	wr(i2c1_se, 0x288, SE_PACK1);
	wr(i2c1_se, SE_TX_WM, 0);
	wr(i2c1_se, 0x810, 0);
	wr(i2c1_se, 0x814, ((rxp >> 16) & 0x3fu) - 1u);
	wr(i2c1_se, SE_IRQ_EN, SE_DONE | SE_ERRORS | (3u << 26) | SE_TX_WM_IRQ);
	return 0;
}

/*
 * rain (boot-arg raini2cscan=1): which addresses answer on QUP0 SE1 (gpio4/5) and SE2
 * (gpio6/7, DT: smb1351 charger 0x55, fusb303 0x21, wusb3801 0x60)? Read-only: one
 * register-0 read per address. The fuel gauge is not at topaz's SE1 0x71 here.
 */
static void
i2c_scan_bus(const char *name)
{
	char line[256];
	unsigned n = 0, len = 0;
	line[0] = 0;
	for (uint8_t a = 0x08; a < 0x78; a++) {
		uint8_t b = 0;
		lck_mtx_lock(&sweet_i2c1_lock);
		uint8_t reg = 0;
		int err = i2c1_msg(a, &reg, 1, NULL, 0, true);
		if (!err) {
			err = i2c1_msg(a, NULL, 0, &b, 1, false);
		}
		lck_mtx_unlock(&sweet_i2c1_lock);
		if (!err && len + 8 < sizeof(line)) {
			len += (unsigned)snprintf(line + len, sizeof(line) - len, " %02x(%02x)", a, b);
			n++;
		}
	}
	printf("RAIN I2C: %s: %u answer:%s\n", name, n, line);
}

static void
sweet_i2c_scan(void)
{
	i2c_scan_bus("SE1 gpio4/5");
	vm_offset_t se1 = i2c1_se, se2 = ml_io_map(0x04a88000u, 0x4000);
	if (!se2) {
		return;
	}
	wr(i2c1_gcc, I2C1_GCC_VOTE, rd(i2c1_gcc, I2C1_GCC_VOTE) | (1u << 12));      /* qupv3_wrap0_s2 (gcc-khaje.c) */
	if (!wait_clear(i2c1_gcc, 0x1f3a4u, 1u << 31)) {
		printf("RAIN I2C: SE2 clock off (cbcr %#x)\n", rd(i2c1_gcc, 0x1f3a4u));
		return;
	}
	/* GPIO6/7: function 1 (qup2), 2 mA, no pull */
	wr(i2c1_west, 0x6000, (rd(i2c1_west, 0x6000) & ~0x3ffu) | 0x4u);
	wr(i2c1_west, 0x7000, (rd(i2c1_west, 0x7000) & ~0x3ffu) | 0x4u);
	lck_mtx_lock(&sweet_i2c1_lock);
	i2c1_se = se2;
	int e = i2c1_engine();
	lck_mtx_unlock(&sweet_i2c1_lock);
	if (!e) {
		i2c_scan_bus("SE2 gpio6/7");
	}
	lck_mtx_lock(&sweet_i2c1_lock);
	i2c1_se = se1;
	lck_mtx_unlock(&sweet_i2c1_lock);
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
