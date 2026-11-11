/*
 * topaz (SM6225): boot the modem DSP (MPSS). It runs the WCN3988 WLAN firmware, so this is
 * Wi-Fi step 1. Port of MakrSas/topaz-woa drivers/TopazModem/ModemPas.c + Smem.c, the way
 * Linux qcom_mdt_load() + qcom_q6v5_pas do it:
 *   TrustZone PAS init_image (.mdt metadata) -> mem_setup -> copy the .bNN segments into
 *   the carveout (0x4AB00000, 0x6900000) -> auth_and_reset -> wait for the modem's SMP2P
 *   "slave-kernel" READY bit (SMEM item 435) and answer its IPA power query (item 428).
 *
 * Boot-arg modem=1. Firmware from the root filesystem, /usr/share/firmware/topaz/modem/
 * (stock HyperOS 2.0.208 NON-HLOS: it must match this phone's EFS partitions).
 * Rules (WIFI_RISKS.md): buffers TZ/the modem see are below 0xF0000000 (own pool, not the
 * GPU's 0x90000000-0xD0000000), memory shared with the modem is write-combined (Normal NC:
 * no cache upkeep, unaligned access ok), the carveout is mapped only while copying, the
 * modem must start cold, polling only (no interrupts), no EFS writes.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <libkern/crc.h>
#include <sys/param.h>
#include <sys/errno.h>

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define LOG(fmt, ...)       IOLog("TopazModem: " fmt "\n", ##__VA_ARGS__)

#define PAS_ID_MODEM        4
#define SCM_FN(svc, cmd)    (0x42000000u | ((uint32_t)(svc) << 8) | (uint32_t)(cmd))
#define SCM_SVC_PIL         0x02
#define PIL_INIT_IMAGE      0x01
#define PIL_MEM_SETUP       0x02
#define PIL_AUTH_RESET      0x05
#define PIL_SHUTDOWN        0x06
#define PIL_IS_SUPPORTED    0x07
#define SCM_ARG_RW          2

#define PT_LOAD             1
#define MDT_TYPE_MASK       (7u << 24)
#define MDT_TYPE_HASH       (2u << 24)
#define MDT_RELOCATABLE     (1u << 27)

/* middle DDR bank, RAM the kernel does not manage; below 0xF0000000 for TZ/the modem */
#define MODEM_POOL_BASE     0xE0000000ull
#define MODEM_POOL_SIZE     0x01000000ull       /* 16 MB: metadata, SCM args, rmtfs, CE rings, WMI memory */

#define SMEM_PA             0x46000000ull
#define SMEM_SIZE           0x00200000u
#define SMEM_GLOBAL_HOST    0xFFFE
#define TCSR_MUTEX_PA       0x00340000ull       /* qcom,tcsr-mutex, stride 0x1000 */
#define SMEM_HWLOCK         3
#define HWLOCK_APPS_ID      1
#define APCS_PA             0x0F111000ull
#define APCS_IPC            8                   /* apcs_glb mailbox */
#define MPSS_SMP2P_BIT      14

#define ITEM_SMP2P_OUT      428                 /* apps -> modem */
#define ITEM_SMP2P_IN       435                 /* modem -> apps */
#define ITEM_MSS_CRASH      421
#define SMP2P_MAGIC         0x504D5324u         /* "$SMP" */
#define SMP2P_MAX_ENTRY     16

struct elf32_ehdr {
	uint8_t  e_ident[16];
	uint16_t e_type, e_machine;
	uint32_t e_version, e_entry, e_phoff, e_shoff, e_flags;
	uint16_t e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx;
} __attribute__((packed));
struct elf32_phdr {
	uint32_t p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align;
} __attribute__((packed));

struct smem_pt_entry { uint32_t offset, size, flags; uint16_t host0, host1; uint32_t cacheline, rsvd[7]; } __attribute__((packed));
struct smem_pt { uint8_t magic[4]; uint32_t version, num_entries, rsvd[5]; } __attribute__((packed));
struct smem_part_hdr { uint8_t magic[4]; uint16_t host0, host1; uint32_t size, free_uncached, free_cached, rsvd[3]; } __attribute__((packed));
struct smem_priv_entry { uint16_t canary, item; uint32_t size; uint16_t pad_data, pad_hdr; uint32_t rsvd; } __attribute__((packed));
struct smp2p_hdr { uint32_t magic; uint8_t version, features[3]; uint16_t local_pid, remote_pid, total, valid; uint32_t flags; } __attribute__((packed));
struct smp2p_entry { char name[16]; uint32_t value; } __attribute__((packed));

static uint8_t *smem;                           /* write-combined */
static volatile uint8_t *apcs, *tcsr_mutex;     /* device */
static uint8_t *pool;
static uint64_t pool_used;
/*
 * TopazAudio (adsp=1) shares SMEM, the APCS doorbell, SCM, the pool and the firmware reader
 * (topaz_* below). The TCSR hwlock only tells apps from the remote processors apart, so the
 * two kernel threads also take this lock around SMEM allocation and the pool.
 */
static IOLock *soc_lock;

static inline void mb(void) { __asm__ volatile ("dsb sy" ::: "memory"); }

/* ---------------------------------------------------------------- mappings */

/* Large or RAM regions: an IOMemoryDescriptor mapping (ml_io_map space is small). */
static IOMemoryMap *
map_phys(uint64_t pa, uint64_t size, bool wc)
{
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionInOut);
	IOMemoryMap *m;

	if (md == NULL) {
		return NULL;
	}
	m = md->map(wc ? kIOMapWriteCombineCache : kIOMapInhibitCache);
	md->release();
	return m;
}

static void *
pool_alloc(uint64_t size, uint64_t *pa)
{
	size = (size + 0xfff) & ~0xfffull;
	IOLockLock(soc_lock);
	if (pool == NULL || pool_used + size > MODEM_POOL_SIZE) {
		IOLockUnlock(soc_lock);
		return NULL;
	}
	*pa = MODEM_POOL_BASE + pool_used;
	void *va = pool + pool_used;
	pool_used += size;
	IOLockUnlock(soc_lock);
	bzero(va, size);                            /* Normal NC: DC ZVA is fine */
	return va;
}

/* ---------------------------------------------------------------- TrustZone */

static uint64_t
smc(uint64_t a[8])
{
	register uint64_t x0 __asm__("x0") = a[0], x1 __asm__("x1") = a[1], x2 __asm__("x2") = a[2],
	    x3 __asm__("x3") = a[3], x4 __asm__("x4") = a[4], x5 __asm__("x5") = a[5], x6 __asm__("x6") = a[6],
	    x7 __asm__("x7") = a[7];
	__asm__ volatile ("smc #0"
            : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4), "+r"(x5), "+r"(x6), "+r"(x7)
            :
            : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	a[0] = x0; a[1] = x1; a[2] = x2; a[3] = x3; a[4] = x4; a[5] = x5; a[6] = x6; a[7] = x7;
	return x0;
}

static unsigned scm_resumes;

static uint64_t
scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1)
{
	uint64_t args[8] = { fn, arginfo, a, b, c, 0, 0, 0 };
	unsigned n;

	smc(args);
	/* long calls return INTERRUPTED (1): resume with x0 = 1 and the session from x6 */
	for (n = 0; args[0] == 1 && n < 100000; n++) {
		uint64_t again[8] = { 1, arginfo, a, b, c, 0, args[6], 0 };
		memcpy(args, again, sizeof(args));
		smc(args);
	}
	scm_resumes = n;
	if (res1 != NULL) {
		*res1 = args[1];
	}
	return args[0];
}

/* ---------------------------------------------------------------- firmware files */

/*
 * The firmware is read from the phone's stock modem partition (boot-arg modem_part=, default
 * modem_a), read-only through sweet_ufs (bsd/dev/sweet_ufs.c): nothing has to be put on
 * userdata. Where each file lies in that FAT16 volume was worked out on the host from the
 * same NON-HLOS.bin (HyperOS 2.0.208; every file is one contiguous run); the CRC32 tells
 * whether the phone's partition is that image.
 */
extern "C" int sweet_ufs_read_part(const char *want, uint64_t off, void *dst, uint64_t len);

static char fw_part[32] = "modem_a";
static const struct { const char *name; uint32_t off, size, crc; } fw_files[] = {
	{ "modem.b00", 0x1e1d000, 0x414, 0xaacde80c },
	{ "modem.b01", 0x1e21000, 0x1f78, 0x80555ebe },
	{ "modem.b02", 0x1e25000, 0x2930, 0x0858b3a0 },
	{ "modem.b03", 0x1e29000, 0x2518, 0x93a2467e },
	{ "modem.b04", 0x1e2d000, 0x4a3ac, 0xbf85c65c },
	{ "modem.b05", 0x1e79000, 0x119d8, 0xf3637bd9 },
	{ "modem.b06", 0x1e8d000, 0x100c, 0x89d24f27 },
	{ "modem.b07", 0x1e91000, 0x90b4, 0xefeddcd9 },
	{ "modem.b08", 0x1e9d000, 0x4f054, 0xeb2e2bdb },
	{ "modem.b09", 0x1eed000, 0x143ee8, 0xc8344d45 },
	{ "modem.b10", 0x2031000, 0x40000, 0x7e9db699 },
	{ "modem.b11", 0x2071000, 0x540, 0x0030dad4 },
	{ "modem.b12", 0x2075000, 0xe2210, 0x7d37ba08 },
	{ "modem.b14", 0x2159000, 0x2d, 0x7a4002c7 },
	{ "modem.b15", 0x215d000, 0x7c, 0x959b6a58 },
	{ "modem.b16", 0x2161000, 0x9c1a0, 0xd902be24 },
	{ "modem.b17", 0x2201000, 0x61110, 0xd2182ca7 },
	{ "modem.b18", 0x2265000, 0x2d29a9, 0x672e3b04 },
	{ "modem.b19", 0x2539000, 0x2d94cc, 0x916bcc13 },
	{ "modem.b22", 0x2815000, 0x10ec0, 0x444b4456 },
	{ "modem.b23", 0x2829000, 0x1722040, 0x3c3bc2a3 },
	{ "modem.b24", 0x3f4d000, 0xa07900, 0x4001a993 },
	{ "modem.b25", 0x4955000, 0x77a45, 0xda7bfa33 },
	{ "modem.b26", 0x49cd000, 0x15104, 0x33437cee },
	{ "modem.b27", 0x49e5000, 0x1378, 0x161303ae },
	{ "modem.b28", 0x49e9000, 0xb0, 0x1e475201 },
	{ "modem.b29", 0x49ed000, 0x7d52c, 0x7d6f01a3 },
	{ "modem.mdt", 0x4a6d000, 0x238c, 0x8742d22b },
};

static int
fw_find(const char *name)
{
	for (unsigned i = 0; i < sizeof(fw_files) / sizeof(fw_files[0]); i++) {
		if (strcmp(fw_files[i].name, name) == 0) {
			return (int)i;
		}
	}
	return -1;
}

static int64_t
fw_size(const char *name)
{
	int i = fw_find(name);
	return i < 0 ? -1 : (int64_t)fw_files[i].size;
}

/* Read the whole file name into dst (max bytes at most). Returns bytes read or -1. */
static int64_t
fw_read(const char *name, void *dst, uint64_t max)
{
	int i = fw_find(name), err;

	if (i < 0 || fw_files[i].size > max) {
		LOG("%s: not in the table or too big (%lld)", name, fw_size(name));
		return -1;
	}
	err = sweet_ufs_read_part(fw_part, fw_files[i].off, dst, fw_files[i].size);
	if (err != 0) {
		LOG("%s: read from %s failed: %d", name, fw_part, err);
		return -1;
	}
	uint32_t crc = crc32(0, dst, fw_files[i].size);
	if (crc != fw_files[i].crc) {
		/* TrustZone checks the signature anyway: say so and go on */
		LOG("%s: crc32 %08x, expected %08x: %s is not the image this was made for", name, crc, fw_files[i].crc, fw_part);
	}
	return fw_files[i].size;
}

/* ---------------------------------------------------------------- SMEM */

static struct smem_part_hdr *
smem_partition(uint16_t a, uint16_t b)
{
	struct smem_pt *pt = (struct smem_pt *)(smem + SMEM_SIZE - 0x1000);
	struct smem_pt_entry *e = (struct smem_pt_entry *)(pt + 1);

	if (memcmp(pt->magic, "$TOC", 4) != 0 || pt->num_entries > 64) {
		return NULL;
	}
	for (uint32_t i = 0; i < pt->num_entries; i++) {
		if (e[i].offset != 0 && e[i].size != 0 && e[i].offset < SMEM_SIZE &&
		    ((e[i].host0 == a && e[i].host1 == b) || (e[i].host0 == b && e[i].host1 == a))) {
			return (struct smem_part_hdr *)(smem + e[i].offset);
		}
	}
	return NULL;
}

static void *
smem_get(struct smem_part_hdr *p, uint16_t item, uint32_t *size)
{
	uint8_t *e, *end;

	if (p == NULL || memcmp(p->magic, "$PRT", 4) != 0 || p->free_uncached > p->size) {
		return NULL;
	}
	e = (uint8_t *)p + sizeof(*p);
	end = (uint8_t *)p + p->free_uncached;
	while (e + sizeof(struct smem_priv_entry) <= end) {
		struct smem_priv_entry *h = (struct smem_priv_entry *)e;
		if (h->canary != 0xA5A5) {
			return NULL;
		}
		if (h->item == item) {
			if (size != NULL) {
				*size = h->size - h->pad_data;
			}
			return e + sizeof(*h) + h->pad_hdr;
		}
		e += sizeof(*h) + h->pad_hdr + h->size;
	}
	return NULL;
}

static bool
smem_lock(void)
{
	volatile uint32_t *reg = (volatile uint32_t *)(tcsr_mutex + SMEM_HWLOCK * 0x1000);

	for (int n = 0; n < 100000; n++) {
		*reg = HWLOCK_APPS_ID;
		mb();
		if (*reg == HWLOCK_APPS_ID) {
			return true;
		}
		IODelay(10);
	}
	LOG("SMEM hwlock busy: owner %u", *reg);
	return false;
}

static void
smem_unlock(void)
{
	mb();
	*(volatile uint32_t *)(tcsr_mutex + SMEM_HWLOCK * 0x1000) = 0;
	mb();
}

/* qcom_smem_alloc_private(): append an uncached entry under hwlock 3 */
static void *
smem_alloc(struct smem_part_hdr *p, uint16_t item, uint32_t size)
{
	uint32_t aligned = (size + 7) & ~7u, need = sizeof(struct smem_priv_entry) + aligned;
	void *v = smem_get(p, item, NULL);

	if (v != NULL || p == NULL) {
		return v;
	}
	IOLockLock(soc_lock);
	if (!smem_lock()) {
		IOLockUnlock(soc_lock);
		return NULL;
	}
	v = smem_get(p, item, NULL);
	if (v == NULL && p->free_uncached + need <= p->free_cached) {
		struct smem_priv_entry *h = (struct smem_priv_entry *)((uint8_t *)p + p->free_uncached);
		h->canary = 0xA5A5;
		h->item = item;
		h->size = aligned;
		h->pad_data = (uint16_t)(aligned - size);
		h->pad_hdr = 0;
		h->rsvd = 0;
		v = h + 1;
		bzero(v, aligned);
		mb();
		p->free_uncached += need;
		mb();
	} else if (v == NULL) {
		LOG("SMEM partition full: %x..%x need %x", p->free_uncached, p->free_cached, need);
	}
	smem_unlock();
	IOLockUnlock(soc_lock);
	return v;
}

/* ---------------------------------------------------------------- SMP2P */

static void
smp2p_kick(void)
{
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << MPSS_SMP2P_BIT;
	mb();
}

static struct smp2p_hdr *
smp2p_create_out(struct smem_part_hdr *p)
{
	struct smp2p_hdr *o = (struct smp2p_hdr *)smem_alloc(p, ITEM_SMP2P_OUT,
	    sizeof(struct smp2p_hdr) + SMP2P_MAX_ENTRY * sizeof(struct smp2p_entry));

	if (o == NULL || o->version != 0) {
		return o;
	}
	o->magic = SMP2P_MAGIC;
	o->local_pid = 0;
	o->remote_pid = 1;
	o->total = SMP2P_MAX_ENTRY;
	o->valid = 0;
	o->features[0] = 1;                         /* SSR_ACK */
	mb();
	o->version = 1;                             /* validates the item */
	smp2p_kick();
	return o;
}

static volatile uint32_t *
smp2p_find(struct smp2p_hdr *h, const char *name)
{
	struct smp2p_entry *e = (struct smp2p_entry *)(h + 1);

	for (uint32_t i = 0; i < h->valid && i < SMP2P_MAX_ENTRY; i++) {
		if (strncmp(e[i].name, name, 16) == 0) {
			return &e[i].value;
		}
	}
	return NULL;
}

static volatile uint32_t *
smp2p_add_out(struct smp2p_hdr *o, const char *name, uint32_t value)
{
	struct smp2p_entry *e = (struct smp2p_entry *)(o + 1);
	volatile uint32_t *v = smp2p_find(o, name);

	if (v != NULL || o->valid >= o->total || o->valid >= SMP2P_MAX_ENTRY) {
		return v;
	}
	bzero(e[o->valid].name, sizeof(e[o->valid].name));
	strlcpy(e[o->valid].name, name, sizeof(e[o->valid].name));
	e[o->valid].value = value;
	mb();
	o->valid++;
	smp2p_kick();
	return &e[o->valid - 1].value;
}

static bool svc_rx(uint32_t src_node, uint32_t src_port, uint32_t dst_port, const uint8_t *data, uint32_t len);
static void svc_announce(void);
static void svc_summary(void);
static void wlfw_arrive(uint32_t node, uint32_t port);
static bool wifi_enabled(void);
static bool ce_start(void);
static void radio_poll(unsigned ms);
static void w_rx_native(const uint8_t *f, uint32_t len);
static void htt_tx_compl(const uint8_t *p, uint32_t len);
static void w_assoc_mgmt_rx(const uint8_t *f, uint32_t len);
static void w_join_event(uint32_t id, const uint8_t *t, uint32_t len);
extern "C" void read_random(void *buffer, unsigned int numBytes);

/* ---------------------------------------------------------------- GLINK over SMEM */

/*
 * After READY the modem talks GLINK through two SMEM FIFOs (items 478 descriptor, 479 our TX,
 * 480 its TX), doorbell APCS bit 12. One channel, "IPCRTR", carries QRTR. Port of
 * topaz-woa drivers/TopazModem/Glink.c, polled from the watch thread (no interrupts).
 */
#define SMEM_GLINK_DESC     478
#define SMEM_GLINK_FIFO_TX  479
#define SMEM_GLINK_FIFO_RX  480
#define GLINK_TX_SIZE       0x4000u
#define GLINK_IRQ_BIT       12
#define GLINK_FEATURES      1                   /* INTENT_REUSE */

#define CMD_VERSION         0
#define CMD_VERSION_ACK     1
#define CMD_OPEN            2
#define CMD_CLOSE           3
#define CMD_OPEN_ACK        4
#define CMD_INTENT          5
#define CMD_RX_DONE         6
#define CMD_RX_INTENT_REQ   7
#define CMD_RX_INTENT_REQ_ACK 8
#define CMD_TX_DATA         9
#define CMD_CLOSE_ACK       11
#define CMD_TX_DATA_CONT    12
#define CMD_READ_NOTIF      13
#define CMD_RX_DONE_W_REUSE 14
#define CMD_SIGNALS         15

#define CHAN_NAME           "IPCRTR"
#define CHAN_LCID           1
#define OUR_INTENTS         8
#define OUR_INTENT_SIZE     0x4400u

#define QRTR_NODE_APPS      1
#define QRTR_PORT_CTRL      0xFFFFFFFEu
#define QRTR_NODE_BCAST     0xFFFFFFFFu
#define QRTR_TYPE_DATA      1
#define QRTR_TYPE_HELLO     2
#define QRTR_TYPE_BYE       3
#define QRTR_TYPE_NEW_SERVER 4
#define QRTR_TYPE_DEL_SERVER 5
#define QRTR_TYPE_DEL_CLIENT 6
#define QRTR_TYPE_RESUME_TX 7
#define QMI_SVC_WLFW        0x45

struct glink_msg { uint16_t cmd, p1; uint32_t p2; } __attribute__((packed));
struct glink_data_hdr { struct glink_msg m; uint32_t chunk, left; } __attribute__((packed));
struct qrtr_hdr_v1 { uint32_t version, type, src_node, src_port, confirm_rx, size, dst_node, dst_port; } __attribute__((packed));
struct qrtr_hdr_v2 { uint8_t version, type, flags, optlen; uint32_t size; uint16_t src_node, src_port, dst_node, dst_port; } __attribute__((packed));
struct qrtr_ctrl { uint32_t cmd, a, b, c, d; } __attribute__((packed));

static volatile uint32_t *g_desc;               /* [0] tx tail [1] tx head [2] rx tail [3] rx head */
static uint8_t *g_tx, *g_rx;
static uint32_t g_rx_len, g_features = GLINK_FEATURES, g_rcid, g_next_intent = 1;
static bool g_remote_open, g_open_sent, g_open_acked, g_intents_sent, g_intent_req_sent;
static struct { uint32_t id, size; bool used; } g_ri[256];
static unsigned g_ri_count;
static uint8_t g_asm[OUR_INTENT_SIZE];
static uint32_t g_asm_len;
static uint32_t q_ver = 1, q_modem_node = QRTR_NODE_BCAST;
static bool q_hello_sent, q_hello_rx, q_announced;
static struct { uint32_t svc, inst, node, port; } q_srv[64];
static unsigned q_srv_count, g_rx_msgs, g_tx_msgs, q_resumes;
static unsigned g_ms;                           /* watch time, for the log */

static uint32_t
rx_avail(void)
{
	uint32_t h = g_desc[3], t = g_desc[2];
	return h >= t ? h - t : g_rx_len - t + h;
}

static void
rx_peek(uint32_t off, void *dst, uint32_t n)
{
	uint32_t t = g_desc[2] + off;
	if (t >= g_rx_len) {
		t -= g_rx_len;
	}
	uint32_t n1 = MIN(n, g_rx_len - t);
	memcpy(dst, g_rx + t, n1);
	if (n > n1) {
		memcpy((uint8_t *)dst + n1, g_rx, n - n1);
	}
}

static void
rx_advance(uint32_t n)
{
	uint32_t t = g_desc[2] + n;
	if (t >= g_rx_len) {
		t -= g_rx_len;
	}
	mb();
	g_desc[2] = t;
}

static uint32_t
tx_put(uint32_t head, const void *src, uint32_t n)
{
	uint32_t n1 = MIN(n, GLINK_TX_SIZE - head);
	memcpy(g_tx + head, src, n1);
	if (n > n1) {
		memcpy(g_tx, (const uint8_t *)src + n1, n - n1);
	}
	head += n;
	return head >= GLINK_TX_SIZE ? head - GLINK_TX_SIZE : head;
}

static bool
tx_write(const void *hdr, uint32_t hlen, const void *data, uint32_t dlen)
{
	uint32_t head = 0, tail = 0, avail = 0;
	int n;

	for (n = 0; n < 1000; n++) {
		head = g_desc[1];
		tail = g_desc[0];
		avail = tail <= head ? GLINK_TX_SIZE - head + tail : tail - head;
		avail = avail < 8 ? 0 : avail - 8;
		if (avail >= ((hlen + dlen + 7) & ~7u)) {
			break;
		}
		IODelay(100);
	}
	if (n == 1000) {
		LOG("glink TX FIFO full (head %x tail %x)", head, tail);
		return false;
	}
	head = tx_put(head, hdr, hlen);
	if (dlen != 0) {
		head = tx_put(head, data, dlen);
	}
	head = (head + 7) & ~7u;
	if (head >= GLINK_TX_SIZE) {
		head -= GLINK_TX_SIZE;
	}
	mb();
	g_desc[1] = head;
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << GLINK_IRQ_BIT;
	mb();
	g_tx_msgs++;
	return true;
}

static void
send_cmd(uint16_t cmd, uint16_t p1, uint32_t p2)
{
	struct glink_msg m = { cmd, p1, p2 };
	tx_write(&m, sizeof(m), NULL, 0);
}

static void
send_open(void)
{
	struct { struct glink_msg m; char name[8]; } __attribute__((packed)) req = {};
	req.m.cmd = CMD_OPEN;
	req.m.p1 = CHAN_LCID;
	req.m.p2 = sizeof(CHAN_NAME);
	memcpy(req.name, CHAN_NAME, sizeof(CHAN_NAME));
	tx_write(&req, sizeof(req), NULL, 0);
	g_open_sent = true;
}

static void
advertise_intents(uint32_t count, uint32_t size)
{
	struct { struct glink_msg m; uint32_t pair[2 * OUR_INTENTS]; } __attribute__((packed)) m = {};
	m.m.cmd = CMD_INTENT;
	m.m.p1 = CHAN_LCID;
	m.m.p2 = count;
	for (uint32_t i = 0; i < count && i < OUR_INTENTS; i++) {
		m.pair[2 * i] = size;
		m.pair[2 * i + 1] = g_next_intent++;
	}
	tx_write(&m, sizeof(struct glink_msg) + 8 * count, NULL, 0);
}

static bool
chan_up(void)
{
	return g_remote_open && g_open_acked;
}

static bool
glink_try_send(const void *data, uint32_t len)
{
	unsigned i;
	for (i = 0; i < g_ri_count; i++) {
		if (!g_ri[i].used && g_ri[i].size >= len) {
			break;
		}
	}
	if (i == g_ri_count) {
		if (!g_intent_req_sent) {
			send_cmd(CMD_RX_INTENT_REQ, CHAN_LCID, len);
			g_intent_req_sent = true;
		}
		return false;
	}
	struct glink_data_hdr h = { { CMD_TX_DATA, CHAN_LCID, g_ri[i].id }, len, 0 };
	g_ri[i].used = true;
	return tx_write(&h, sizeof(h), data, len);
}

/* packets wait here until the modem has a free RX intent for them */
#define TXQ_LEN 64
static struct { void *p; uint32_t len; } g_txq[TXQ_LEN];
static unsigned g_txq_head, g_txq_tail, g_txq_drops;

static void
txq_pump(void)
{
	while (g_txq_head != g_txq_tail &&
	    glink_try_send(g_txq[g_txq_head % TXQ_LEN].p, g_txq[g_txq_head % TXQ_LEN].len)) {
		IOFree(g_txq[g_txq_head % TXQ_LEN].p, g_txq[g_txq_head % TXQ_LEN].len);
		g_txq_head++;
	}
}

static void
glink_send(const void *data, uint32_t len)
{
	void *p;
	if (g_txq_tail - g_txq_head >= TXQ_LEN || (p = IOMalloc(len)) == NULL) {
		g_txq_drops++;
		return;
	}
	memcpy(p, data, len);
	g_txq[g_txq_tail % TXQ_LEN].p = p;
	g_txq[g_txq_tail % TXQ_LEN].len = len;
	g_txq_tail++;
	txq_pump();
}

/* ---------------------------------------------------------------- QRTR */

static void
qrtr_send(uint32_t type, uint32_t src_port, uint32_t dst_node, uint32_t dst_port, const void *payload, uint32_t len)
{
	uint32_t cap = (uint32_t)sizeof(struct qrtr_hdr_v1) + len + 4;
	uint8_t *buf = (uint8_t *)IOMalloc(cap);
	uint32_t n;

	if (buf == NULL) {
		return;
	}
	bzero(buf, cap);
	if (q_ver == 2) {
		struct qrtr_hdr_v2 *h = (struct qrtr_hdr_v2 *)buf;
		h->version = 2;
		h->type = (uint8_t)type;
		h->flags = type == QRTR_TYPE_DATA ? 1 : 0;
		h->size = len;
		h->src_node = QRTR_NODE_APPS;
		h->src_port = (uint16_t)src_port;
		h->dst_node = (uint16_t)dst_node;
		h->dst_port = (uint16_t)dst_port;
		n = sizeof(*h);
	} else {
		struct qrtr_hdr_v1 *h = (struct qrtr_hdr_v1 *)buf;
		h->version = 1;
		h->type = type;
		h->src_node = QRTR_NODE_APPS;
		h->src_port = src_port;
		h->confirm_rx = type == QRTR_TYPE_DATA ? 1 : 0;   /* flow control: ask for RESUME_TX */
		h->size = len;
		h->dst_node = dst_node;
		h->dst_port = dst_port;
		n = sizeof(*h);
	}
	memcpy(buf + n, payload, len);
	glink_send(buf, (n + len + 3) & ~3u);
	IOFree(buf, cap);
}

static const char *
svc_name(uint32_t svc)
{
	switch (svc) {
	case 0x45: return "WLFW";
	case 0x0E: return "rmtfs";
	case 0x40: return "servreg-loc";
	case 0x42: return "servreg-notif";
	case 0x2B: return "ssctl";
	case 0x31: return "ipa";
	case 0x01: return "wds";
	case 0x02: return "dms";
	case 0x03: return "nas";
	case 0x0B: return "uim";
	case 0x1A: return "wda";
	default:   return "";
	}
}

static void
qrtr_rx(const uint8_t *p, uint32_t len)
{
	uint32_t type, src_node, src_port, dst_port, size, hlen, confirm;

	if (len >= sizeof(struct qrtr_hdr_v1) && *(const uint32_t *)p == 1) {
		const struct qrtr_hdr_v1 *h = (const struct qrtr_hdr_v1 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h); confirm = h->confirm_rx;
		q_ver = 1;
	} else if (len >= sizeof(struct qrtr_hdr_v2) && p[0] == 2) {
		const struct qrtr_hdr_v2 *h = (const struct qrtr_hdr_v2 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port;
		dst_port = h->dst_port; size = h->size; hlen = sizeof(*h) + h->optlen * 4;
		src_port = src_port == 0xFFFE ? QRTR_PORT_CTRL : src_port;
		dst_port = dst_port == 0xFFFE ? QRTR_PORT_CTRL : dst_port;
		confirm = h->flags & 1;
		q_ver = 2;
	} else {
		LOG("t=%u ms: qrtr: bad packet, %u bytes", g_ms, len);
		return;
	}
	const struct qrtr_ctrl *c = (const struct qrtr_ctrl *)(p + hlen);

	switch (type) {
	case QRTR_TYPE_HELLO:
		q_modem_node = src_node;
		q_hello_rx = true;
		q_hello_sent = false;                   /* (re)answer with our HELLO */
		LOG("t=%u ms: qrtr v%u HELLO from node %u", g_ms, q_ver, src_node);
		break;
	case QRTR_TYPE_NEW_SERVER:
		if (q_srv_count < sizeof(q_srv) / sizeof(q_srv[0])) {
			q_srv[q_srv_count].svc = c->a;
			q_srv[q_srv_count].inst = c->b;
			q_srv[q_srv_count].node = c->c;
			q_srv[q_srv_count].port = c->d;
			q_srv_count++;
		}
		if (c->a == QMI_SVC_WLFW) {
			LOG("t=%u ms: *** WLFW service 0x45 up: inst %x node %u port %x ***", g_ms, c->b, c->c, c->d);
			wlfw_arrive(c->c, c->d);
		} else if (c->a != 0) {
			LOG("t=%u ms: service %x:%x port %x %s", g_ms, c->a, c->b, c->d, svc_name(c->a));
		}
		break;
	case QRTR_TYPE_DATA:
		if (confirm) {
			/* the full 20-byte control packet (Linux qrtr_send_resume_tx): 12 bytes stalls the modem */
			struct qrtr_ctrl r = { QRTR_TYPE_RESUME_TX, QRTR_NODE_APPS, dst_port, 0, 0 };
			qrtr_send(QRTR_TYPE_RESUME_TX, dst_port, src_node, src_port, &r, sizeof(r));
		}
		if (hlen + size <= len) {
			(void)svc_rx(src_node, src_port, dst_port, p + hlen, size);
		}
		break;
	case QRTR_TYPE_RESUME_TX:
		q_resumes++;
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------- GLINK RX */

static bool
rx_one(void)
{
	struct glink_msg m;
	uint32_t avail = rx_avail(), n;

	if (avail < sizeof(m)) {
		return false;
	}
	rx_peek(0, &m, sizeof(m));
	g_rx_msgs++;
	switch (m.cmd) {
	case CMD_VERSION:
		rx_advance(8);
		LOG("t=%u ms: glink VERSION %u features %x", g_ms, m.p1, m.p2);
		if (m.p1 != 0) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION_ACK, 1, g_features);
		}
		break;
	case CMD_VERSION_ACK:
		rx_advance(8);
		LOG("t=%u ms: glink VERSION_ACK %u features %x", g_ms, m.p1, m.p2);
		if (m.p1 != 0 && m.p2 != g_features) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION, 1, g_features);
		}
		break;
	case CMD_OPEN: {
		char name[33] = {};
		n = (8 + m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		rx_peek(8, name, MIN(m.p2, 32u));
		rx_advance(n);
		LOG("t=%u ms: glink modem OPEN \"%s\" rcid %u", g_ms, name, m.p1);
		if (strcmp(name, CHAN_NAME) == 0) {
			g_rcid = m.p1;
			g_remote_open = true;
			send_cmd(CMD_OPEN_ACK, (uint16_t)g_rcid, 0);
			if (!g_open_sent) {
				send_open();
			}
		}
		break;
	}
	case CMD_OPEN_ACK:
		rx_advance(8);
		LOG("t=%u ms: glink OPEN_ACK lcid %u", g_ms, m.p1);
		if (m.p1 == CHAN_LCID) {
			g_open_acked = true;
		}
		break;
	case CMD_CLOSE:
		rx_advance(8);
		LOG("t=%u ms: glink modem CLOSE rcid %u", g_ms, m.p1);
		send_cmd(CMD_CLOSE_ACK, m.p1, 0);
		if (m.p1 == g_rcid) {
			g_remote_open = false;
		}
		break;
	case CMD_INTENT:
		n = (8 + 8 * m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		for (uint32_t i = 0; i < m.p2; i++) {
			uint32_t pair[2];
			rx_peek(8 + 8 * i, pair, 8);
			if (m.p1 == g_rcid && g_ri_count < sizeof(g_ri) / sizeof(g_ri[0])) {
				g_ri[g_ri_count].size = pair[0];
				g_ri[g_ri_count].id = pair[1];
				g_ri[g_ri_count].used = false;
				g_ri_count++;
			}
		}
		rx_advance(n);
		g_intent_req_sent = false;
		break;
	case CMD_RX_DONE:
	case CMD_RX_DONE_W_REUSE:
		rx_advance(8);
		for (unsigned i = 0; i < g_ri_count; i++) {
			if (g_ri[i].id == m.p2) {
				if (m.cmd == CMD_RX_DONE_W_REUSE) {
					g_ri[i].used = false;
				} else {
					g_ri[i] = g_ri[--g_ri_count];
				}
				break;
			}
		}
		break;
	case CMD_RX_INTENT_REQ:
		rx_advance(8);
		advertise_intents(1, MAX(m.p2, OUR_INTENT_SIZE));
		send_cmd(CMD_RX_INTENT_REQ_ACK, CHAN_LCID, 1);
		break;
	case CMD_RX_INTENT_REQ_ACK:
		rx_advance(8);
		if (m.p2 == 0) {
			g_intent_req_sent = false;      /* ask again on the next pump */
		}
		break;
	case CMD_CLOSE_ACK:
	case CMD_SIGNALS:
		rx_advance(8);
		break;
	case CMD_READ_NOTIF:
		rx_advance(8);
		*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << GLINK_IRQ_BIT;
		mb();
		break;
	case CMD_TX_DATA:
	case CMD_TX_DATA_CONT: {
		uint32_t hdr[2];
		if (avail < 16) {
			return false;
		}
		rx_peek(8, hdr, 8);                     /* chunk_size, left_size */
		n = (16 + hdr[0] + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		if (m.cmd == CMD_TX_DATA) {
			g_asm_len = 0;
		}
		if (g_asm_len + hdr[0] <= sizeof(g_asm)) {
			rx_peek(16, g_asm + g_asm_len, hdr[0]);
			g_asm_len += hdr[0];
		}
		rx_advance(n);
		if (hdr[1] == 0) {
			if (m.p1 == g_rcid) {
				qrtr_rx(g_asm, g_asm_len);
			}
			if (g_features & 1) {
				send_cmd(CMD_RX_DONE_W_REUSE, CHAN_LCID, m.p2);
			} else {
				send_cmd(CMD_RX_DONE, CHAN_LCID, m.p2);
				advertise_intents(1, OUR_INTENT_SIZE);
			}
		}
		break;
	}
	default:
		LOG("t=%u ms: glink unknown cmd %u (%x %x), RX stopped", g_ms, m.cmd, m.p1, m.p2);
		g_rx = NULL;
		return false;
	}
	return true;
}

static bool
glink_start(struct smem_part_hdr *part)
{
	bool existed = smem_get(part, SMEM_GLINK_DESC, NULL) != NULL;

	g_desc = (volatile uint32_t *)smem_alloc(part, SMEM_GLINK_DESC, 32);
	g_tx = (uint8_t *)smem_alloc(part, SMEM_GLINK_FIFO_TX, GLINK_TX_SIZE);
	if (g_desc == NULL || g_tx == NULL) {
		LOG("glink: SMEM alloc failed (desc %p tx %p)", g_desc, g_tx);
		return false;
	}
	LOG("glink: desc %s, our TX FIFO %u bytes", existed ? "made by the modem" : "ours", GLINK_TX_SIZE);
	g_desc[2] = 0;                              /* rx tail */
	g_desc[1] = 0;                              /* tx head */
	mb();
	send_cmd(CMD_VERSION, 1, g_features);
	return true;
}

/* one pass, every millisecond from the watch loop */
static void
glink_poll(struct smem_part_hdr *part, unsigned ms)
{
	g_ms = ms;
	if (g_rx == NULL && g_rx_msgs == 0) {
		uint32_t sz = 0;
		g_rx = (uint8_t *)smem_get(part, SMEM_GLINK_FIFO_RX, &sz);
		if (g_rx != NULL) {
			g_rx_len = sz;
			LOG("t=%u ms: modem RX FIFO, %u bytes", ms, sz);
		}
	}
	for (int i = 0; i < 64 && g_rx != NULL && rx_one(); i++) {
	}
	txq_pump();
	if (chan_up() && !g_intents_sent) {
		advertise_intents(OUR_INTENTS, OUR_INTENT_SIZE);
		g_intents_sent = true;
		LOG("t=%u ms: IPCRTR open both ways, %u intents sent", ms, OUR_INTENTS);
	}
	if (chan_up() && !q_hello_sent) {
		struct qrtr_ctrl c = { QRTR_TYPE_HELLO, 0, 0, 0, 0 };
		qrtr_send(QRTR_TYPE_HELLO, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
		q_hello_sent = true;
		LOG("t=%u ms: sent qrtr HELLO to node %x", ms, q_modem_node);
		if (q_hello_rx && !q_announced) {
			svc_announce();
			q_announced = true;
		}
	}
	/* nobody opened IPCRTR after 2 s: open it from our side */
	if (ms >= 2000 && !g_open_sent && g_rx != NULL) {
		LOG("t=%u ms: modem did not open IPCRTR, opening it", ms);
		send_open();
	}
}

static void
glink_summary(void)
{
	LOG("glink: rx %u msgs, tx %u, chan %s, modem intents %u, txq %u drops %u; qrtr node %x hello %u, %u services, %u RESUME_TX",
	    g_rx_msgs, g_tx_msgs, chan_up() ? "UP" : "down", g_ri_count, g_txq_tail - g_txq_head, g_txq_drops,
	    q_modem_node, q_hello_rx, q_srv_count, q_resumes);
	svc_summary();
	for (unsigned i = 0; i < q_srv_count; i++) {
		LOG("  service %x:%x node %u port %x %s", q_srv[i].svc, q_srv[i].inst, q_srv[i].node, q_srv[i].port, svc_name(q_srv[i].svc));
	}
}

/* ---------------------------------------------------------------- host services */

/*
 * What the modem expects from the apps processor (linux-msm userspace daemons), port of
 * topaz-woa drivers/TopazModem/ModemSvc.c:
 *  - pd-mapper: QMI servreg-locator 0x40 v0x101, maps "wlan/fw" etc. to msm/modem/wlan_pd;
 *  - tftp (QRTR service 4096): /readonly/firmware/image/... read-only from modem_a,
 *    /readwrite/... kept in RAM for the session (dropped writes starve modem_cfg);
 *  - rmtfs (QMI 14): the modem's EFS, read-only from modemst1/modemst2/fsc/fsg on UFS, into a
 *    3 MB buffer in our pool assigned to the modem through TrustZone. Writes are not persisted.
 */
#define PORT_PDM            0x4001
#define PORT_TFTP           0x4002
#define PORT_RMTFS          0x4003
#define PORT_SESS           0x4100
#define QMI_REQ             0
#define QMI_RESP            2

/* generated by fwtable_all.py from NON-HLOS.bin (HyperOS 2.0.208 modem_a): 657 files, 657 extents */
static const struct { uint32_t off, len; } fw_ext[] = {
	{ 0x25000, 0x4d4 },
	{ 0x29000, 0x2098 },
	{ 0x2d000, 0x14ef4 },
	{ 0x45000, 0x5e9c0 },
	{ 0xa5000, 0x18f388 },
	{ 0x235000, 0x19f628 },
	{ 0x3d5000, 0xb60 },
	{ 0x3d9000, 0x63e4 },
	{ 0x3e1000, 0x6f6c },
	{ 0x3e9000, 0x12dc },
	{ 0x3ed000, 0x104 },
	{ 0x3f1000, 0xf2a0 },
	{ 0x401000, 0x11ae4 },
	{ 0x415000, 0xa4000 },
	{ 0x4b9000, 0x170484 },
	{ 0x62d000, 0x44004 },
	{ 0x675000, 0x2dfb },
	{ 0x679000, 0x1b60 },
	{ 0x67d000, 0x2dc },
	{ 0x681000, 0x1f60 },
	{ 0x685000, 0x6fec },
	{ 0x68d000, 0x3a56c },
	{ 0x6c9000, 0x94000 },
	{ 0x75d000, 0x4db23e },
	{ 0xc39000, 0x15dd6a },
	{ 0xd99000, 0x423c },
	{ 0xda1000, 0x51a74 },
	{ 0xdf5000, 0xb70 },
	{ 0xdf9000, 0xd1f98 },
	{ 0xecd000, 0x50230 },
	{ 0xf21000, 0xe00 },
	{ 0xf25000, 0x13f0 },
	{ 0xf29000, 0x23810 },
	{ 0xf4d000, 0x116400 },
	{ 0x1065000, 0xd4 },
	{ 0x1069000, 0x256c },
	{ 0x15000, 0x193 },
	{ 0x19000, 0x195 },
	{ 0x1d000, 0x22b },
	{ 0x76b1000, 0x66d8 },
	{ 0x76b9000, 0x66d8 },
	{ 0x76c1000, 0x66d8 },
	{ 0x76c9000, 0x66d8 },
	{ 0x76d1000, 0x66d8 },
	{ 0x76d9000, 0x66d8 },
	{ 0x76e1000, 0x66d8 },
	{ 0x76e9000, 0x66d8 },
	{ 0x76f1000, 0x66d8 },
	{ 0x76f9000, 0x66d8 },
	{ 0x7701000, 0x66d8 },
	{ 0x7709000, 0x66d8 },
	{ 0x7711000, 0x66d8 },
	{ 0x7719000, 0x66d8 },
	{ 0x7721000, 0x66d8 },
	{ 0x7729000, 0x66d8 },
	{ 0x7731000, 0x66d8 },
	{ 0x7739000, 0x66d8 },
	{ 0x7741000, 0x66d8 },
	{ 0x7749000, 0x66d8 },
	{ 0x7751000, 0x66d8 },
	{ 0x7759000, 0x66d8 },
	{ 0x7761000, 0x66d8 },
	{ 0x7769000, 0x66d8 },
	{ 0x7771000, 0x66d8 },
	{ 0x7779000, 0x66d8 },
	{ 0x7781000, 0x66d8 },
	{ 0x7789000, 0x66d8 },
	{ 0x7791000, 0x66d8 },
	{ 0x7799000, 0x66d8 },
	{ 0x77a1000, 0x66d8 },
	{ 0x77a9000, 0x66d8 },
	{ 0x77b1000, 0x66d8 },
	{ 0x77b9000, 0x66d8 },
	{ 0x77c1000, 0x66d8 },
	{ 0x77c9000, 0x66d8 },
	{ 0x77d1000, 0x66d8 },
	{ 0x77d9000, 0x66d8 },
	{ 0x77e1000, 0x66d8 },
	{ 0x77e9000, 0x66d8 },
	{ 0x77f1000, 0x66d8 },
	{ 0x77f9000, 0x66d8 },
	{ 0x7801000, 0x66d8 },
	{ 0x7809000, 0x66d8 },
	{ 0x7811000, 0x66d8 },
	{ 0x7819000, 0x66d8 },
	{ 0x7821000, 0x66d8 },
	{ 0x7829000, 0x66d8 },
	{ 0x7831000, 0x66d8 },
	{ 0x7839000, 0x66d8 },
	{ 0x7841000, 0x66d8 },
	{ 0x106d000, 0x134 },
	{ 0x1071000, 0x1b28 },
	{ 0x1075000, 0x97d3 },
	{ 0x1081000, 0x91 },
	{ 0x1085000, 0x2b8 },
	{ 0x1089000, 0x14c },
	{ 0x108d000, 0x98 },
	{ 0x1091000, 0x1cab },
	{ 0x1095000, 0x1c5c },
	{ 0x1099000, 0x1d4 },
	{ 0x109d000, 0x1c18 },
	{ 0x10a1000, 0x39898 },
	{ 0x10dd000, 0x1a07c },
	{ 0x10f9000, 0x44c },
	{ 0x10fd000, 0x20db5c },
	{ 0x130d000, 0x5b60 },
	{ 0x1315000, 0x95378 },
	{ 0x13ad000, 0xa20 },
	{ 0x13b1000, 0xe614 },
	{ 0x13c1000, 0x1dec },
	{ 0x21000, 0x193 },
	{ 0x760d000, 0x9c2df },
	{ 0x13c5000, 0x200 },
	{ 0x13c9000, 0x4e70 },
	{ 0x13d1000, 0xa89d },
	{ 0x13dd000, 0xc4 },
	{ 0x13e1000, 0x391 },
	{ 0x13e5000, 0x1c0 },
	{ 0x13e9000, 0x130 },
	{ 0x13ed000, 0x2231 },
	{ 0x13f1000, 0x5070 },
	{ 0x13f9000, 0x200 },
	{ 0x13fd000, 0x1b28 },
	{ 0x1401000, 0x109ed8 },
	{ 0x150d000, 0x144 },
	{ 0x1511000, 0x103760 },
	{ 0x1615000, 0x468 },
	{ 0x1619000, 0x130 },
	{ 0x161d000, 0x24749 },
	{ 0x1645000, 0x1d28 },
	{ 0x1649000, 0x134 },
	{ 0x164d000, 0x1b28 },
	{ 0x1651000, 0x3b8c68 },
	{ 0x1a0d000, 0x9b },
	{ 0x1a11000, 0x2763a4 },
	{ 0x1c89000, 0x230 },
	{ 0x1c8d000, 0xa8 },
	{ 0x1c91000, 0x268d6 },
	{ 0x1cb9000, 0x1c5c },
	{ 0x1cbd000, 0x134 },
	{ 0x1cc1000, 0x1b28 },
	{ 0x1cc5000, 0x1430b },
	{ 0x1cdd000, 0xd4 },
	{ 0x1ce1000, 0x6bb4 },
	{ 0x1ce9000, 0x430 },
	{ 0x1ced000, 0x98 },
	{ 0x1cf1000, 0x6c49 },
	{ 0x1cf9000, 0x1c5c },
	{ 0x1cfd000, 0x200 },
	{ 0x1d01000, 0x1b28 },
	{ 0x1d05000, 0x541d },
	{ 0x1d0d000, 0xc2 },
	{ 0x1d11000, 0x388 },
	{ 0x1d15000, 0x1c0 },
	{ 0x1d19000, 0x130 },
	{ 0x1d1d000, 0x137d },
	{ 0x1d21000, 0x1d28 },
	{ 0x1d25000, 0x200 },
	{ 0x1d29000, 0x1b28 },
	{ 0x1d2d000, 0xc0f5 },
	{ 0x1d3d000, 0xc5 },
	{ 0x1d41000, 0x4c60 },
	{ 0x1d49000, 0x180 },
	{ 0x1d4d000, 0x130 },
	{ 0x1d51000, 0x10a9 },
	{ 0x1d55000, 0x1d28 },
	{ 0x1d59000, 0x134 },
	{ 0x1d5d000, 0x1b28 },
	{ 0x1d61000, 0x2bb74 },
	{ 0x1d8d000, 0x89 },
	{ 0x1d91000, 0xe47c },
	{ 0x1da1000, 0x188 },
	{ 0x1da5000, 0x98 },
	{ 0x1da9000, 0x68ee },
	{ 0x1db1000, 0x1c5c },
	{ 0x1db5000, 0x200 },
	{ 0x1db9000, 0x1b28 },
	{ 0x1dbd000, 0x1b20a },
	{ 0x1dd9000, 0xbe },
	{ 0x1ddd000, 0xc7c },
	{ 0x1de1000, 0x298 },
	{ 0x1de5000, 0x130 },
	{ 0x1de9000, 0x3931 },
	{ 0x1ded000, 0x1d28 },
	{ 0x1df1000, 0x134 },
	{ 0x1df5000, 0x1b28 },
	{ 0x1df9000, 0xaf99 },
	{ 0x1e05000, 0x8a },
	{ 0x1e09000, 0xa88 },
	{ 0x1e0d000, 0x174 },
	{ 0x1e11000, 0x98 },
	{ 0x1e15000, 0x231b },
	{ 0x1e19000, 0x1c5c },
	{ 0x1e1d000, 0x414 },
	{ 0x1e21000, 0x1f78 },
	{ 0x1e25000, 0x2930 },
	{ 0x1e29000, 0x2518 },
	{ 0x1e2d000, 0x4a3ac },
	{ 0x1e79000, 0x119d8 },
	{ 0x1e8d000, 0x100c },
	{ 0x1e91000, 0x90b4 },
	{ 0x1e9d000, 0x4f054 },
	{ 0x1eed000, 0x143ee8 },
	{ 0x2031000, 0x40000 },
	{ 0x2071000, 0x540 },
	{ 0x2075000, 0xe2210 },
	{ 0x2159000, 0x2d },
	{ 0x215d000, 0x7c },
	{ 0x2161000, 0x9c1a0 },
	{ 0x2201000, 0x61110 },
	{ 0x2265000, 0x2d29a9 },
	{ 0x2539000, 0x2d94cc },
	{ 0x2815000, 0x10ec0 },
	{ 0x2829000, 0x1722040 },
	{ 0x3f4d000, 0xa07900 },
	{ 0x4955000, 0x77a45 },
	{ 0x49cd000, 0x15104 },
	{ 0x49e5000, 0x1378 },
	{ 0x49e9000, 0xb0 },
	{ 0x49ed000, 0x7d52c },
	{ 0x4a6d000, 0x238c },
	{ 0x5351000, 0x588c },
	{ 0x5335000, 0x4404 },
	{ 0x5341000, 0x402c },
	{ 0x5359000, 0x20 },
	{ 0x535d000, 0x232 },
	{ 0x5361000, 0x1 },
	{ 0x537d000, 0xb4ac },
	{ 0x5395000, 0xb838 },
	{ 0x53ad000, 0xb7d8 },
	{ 0x53c1000, 0xbdc8 },
	{ 0x53d9000, 0xb50c },
	{ 0x53f1000, 0xc048 },
	{ 0x5409000, 0xc16c },
	{ 0x5421000, 0xc328 },
	{ 0x5439000, 0xbf28 },
	{ 0x544d000, 0xc418 },
	{ 0x5469000, 0xc2bc },
	{ 0x5481000, 0xbb40 },
	{ 0x5499000, 0xbd1c },
	{ 0x54ad000, 0xbc08 },
	{ 0x54c1000, 0xc464 },
	{ 0x54d9000, 0xbc00 },
	{ 0x5501000, 0xb7cc },
	{ 0x54ed000, 0xb898 },
	{ 0x5515000, 0xeb6c },
	{ 0x552d000, 0xc05c },
	{ 0x5545000, 0xb964 },
	{ 0x5575000, 0xb948 },
	{ 0x555d000, 0xbd34 },
	{ 0x5589000, 0xaff0 },
	{ 0x559d000, 0xbedc },
	{ 0x55b5000, 0xc240 },
	{ 0x55cd000, 0xf490 },
	{ 0x55e5000, 0x1011c },
	{ 0x5601000, 0xb388 },
	{ 0x5615000, 0xb8b8 },
	{ 0x5629000, 0xbc40 },
	{ 0x563d000, 0xb8d8 },
	{ 0x5651000, 0xb80c },
	{ 0x5665000, 0xc2dc },
	{ 0x567d000, 0xb7dc },
	{ 0x5691000, 0xb940 },
	{ 0x56a5000, 0xbcf4 },
	{ 0x56c1000, 0xc4a4 },
	{ 0x56d9000, 0xc794 },
	{ 0x56f1000, 0xc148 },
	{ 0x5711000, 0xd030 },
	{ 0x5725000, 0xe098 },
	{ 0x573d000, 0x6c5c },
	{ 0x5749000, 0xcc88 },
	{ 0x575d000, 0x5eb0 },
	{ 0x5769000, 0x6c64 },
	{ 0x5775000, 0xc664 },
	{ 0x5789000, 0xc7e4 },
	{ 0x579d000, 0x6c64 },
	{ 0x57a9000, 0x5b60 },
	{ 0x57b5000, 0x5e04 },
	{ 0x57f1000, 0x119bc },
	{ 0x57c9000, 0xa698 },
	{ 0x57d9000, 0x11c50 },
	{ 0x580d000, 0x7618 },
	{ 0x5819000, 0xe4c0 },
	{ 0x582d000, 0xe120 },
	{ 0x5841000, 0x9fa0 },
	{ 0x5851000, 0xa060 },
	{ 0x5861000, 0x9cc4 },
	{ 0x5871000, 0xe350 },
	{ 0x5889000, 0xbcc0 },
	{ 0x58a1000, 0x690c },
	{ 0x58ad000, 0xd604 },
	{ 0x58c5000, 0x52f4 },
	{ 0x71a1000, 0xb7f4 },
	{ 0x71b5000, 0xc568 },
	{ 0x71cd000, 0xab94 },
	{ 0x58dd000, 0xc878 },
	{ 0x58f5000, 0xbbc0 },
	{ 0x590d000, 0xbbf0 },
	{ 0x5921000, 0xb824 },
	{ 0x5931000, 0xb90c },
	{ 0x5955000, 0xbbfc },
	{ 0x5945000, 0xc304 },
	{ 0x5965000, 0xbeec },
	{ 0x5975000, 0xbe0c },
	{ 0x5985000, 0xb88c },
	{ 0x599d000, 0xb8f4 },
	{ 0x59c9000, 0xbbe0 },
	{ 0x59d9000, 0xbbf8 },
	{ 0x59f1000, 0xc734 },
	{ 0x59b1000, 0xb854 },
	{ 0x6595000, 0xc7c4 },
	{ 0x5a09000, 0xd034 },
	{ 0x5b31000, 0xc4c8 },
	{ 0x5b45000, 0xbf00 },
	{ 0x5b55000, 0xb448 },
	{ 0x5a25000, 0xcd68 },
	{ 0x5a4d000, 0xcac8 },
	{ 0x5a39000, 0xccfc },
	{ 0x5a61000, 0xd02c },
	{ 0x5a75000, 0xd688 },
	{ 0x5a89000, 0xbff8 },
	{ 0x5a99000, 0xca3c },
	{ 0x5aad000, 0xcdc4 },
	{ 0x5ac1000, 0xc82c },
	{ 0x5ad9000, 0x5b58 },
	{ 0x5ae9000, 0xca60 },
	{ 0x5afd000, 0xca98 },
	{ 0x5b19000, 0xbb48 },
	{ 0x5b69000, 0xc688 },
	{ 0x5b85000, 0xb7d8 },
	{ 0x5b9d000, 0xbdf0 },
	{ 0x5bad000, 0xbfe0 },
	{ 0x5bc1000, 0xbc40 },
	{ 0x5bd9000, 0xb944 },
	{ 0x5be9000, 0xbecc },
	{ 0x5c01000, 0xd440 },
	{ 0x5c1d000, 0xbac8 },
	{ 0x5c35000, 0xba58 },
	{ 0x5c4d000, 0xb954 },
	{ 0x5c5d000, 0xbf58 },
	{ 0x5c6d000, 0xbec4 },
	{ 0x5c7d000, 0xd450 },
	{ 0x5c91000, 0xc0d0 },
	{ 0x5ca5000, 0xbba4 },
	{ 0x5cbd000, 0xc1cc },
	{ 0x5cd5000, 0xc03c },
	{ 0x5cf1000, 0xc018 },
	{ 0x5d25000, 0xbce8 },
	{ 0x5d3d000, 0xc0cc },
	{ 0x5d0d000, 0xb5e4 },
	{ 0x5d55000, 0xc828 },
	{ 0x5df5000, 0xb3fc },
	{ 0x5d71000, 0xb8f4 },
	{ 0x5d89000, 0xc190 },
	{ 0x5d9d000, 0xc94c },
	{ 0x5db9000, 0xbf18 },
	{ 0x5dc9000, 0xbb1c },
	{ 0x5ddd000, 0xb95c },
	{ 0x5e09000, 0xbb00 },
	{ 0x5e1d000, 0xce10 },
	{ 0x5e39000, 0xc678 },
	{ 0x5e4d000, 0xc7e4 },
	{ 0x5e61000, 0xc5dc },
	{ 0x5e7d000, 0xb8b8 },
	{ 0x5e95000, 0xbf04 },
	{ 0x5ea5000, 0xb8fc },
	{ 0x5eb5000, 0xc1d0 },
	{ 0x5ec9000, 0xd5ac },
	{ 0x5edd000, 0x6180 },
	{ 0x5ee9000, 0xb84c },
	{ 0x5ef9000, 0xdc88 },
	{ 0x5f0d000, 0xb8f8 },
	{ 0x5f1d000, 0xc9c0 },
	{ 0x5f31000, 0xc5d0 },
	{ 0x5f45000, 0xcbb4 },
	{ 0x5f59000, 0xaf7c },
	{ 0x5f6d000, 0xcc50 },
	{ 0x5f89000, 0xb8cc },
	{ 0x5f9d000, 0xcf44 },
	{ 0x5fb9000, 0xb9d8 },
	{ 0x5fcd000, 0xbc4c },
	{ 0x5fe1000, 0xb8e8 },
	{ 0x601d000, 0xc568 },
	{ 0x5ff9000, 0xbe88 },
	{ 0x6009000, 0xbd30 },
	{ 0x6035000, 0xc150 },
	{ 0x6051000, 0xbaf8 },
	{ 0x6061000, 0xbcf8 },
	{ 0x6071000, 0xbde0 },
	{ 0x6085000, 0xb8c8 },
	{ 0x609d000, 0xc2d0 },
	{ 0x60b9000, 0xbd74 },
	{ 0x6121000, 0xc7e0 },
	{ 0x6149000, 0xc7dc },
	{ 0x6135000, 0xc800 },
	{ 0x615d000, 0xbf6c },
	{ 0x616d000, 0xb4b8 },
	{ 0x6185000, 0xbff8 },
	{ 0x6195000, 0xc9a0 },
	{ 0x61a9000, 0xbd20 },
	{ 0x61bd000, 0xc8a4 },
	{ 0x61d9000, 0xb288 },
	{ 0x61e9000, 0xbdb0 },
	{ 0x61fd000, 0xb9fc },
	{ 0x620d000, 0xb4a0 },
	{ 0x6225000, 0xbb18 },
	{ 0x623d000, 0xb6c8 },
	{ 0x624d000, 0xc754 },
	{ 0x6261000, 0xb9a8 },
	{ 0x6271000, 0xbda8 },
	{ 0x6281000, 0xb8b8 },
	{ 0x6291000, 0xb524 },
	{ 0x62a1000, 0xbf44 },
	{ 0x62b1000, 0xc224 },
	{ 0x62c5000, 0xbf18 },
	{ 0x62dd000, 0xc478 },
	{ 0x62f1000, 0xc2b0 },
	{ 0x6305000, 0xc7fc },
	{ 0x6319000, 0xc314 },
	{ 0x632d000, 0xcb54 },
	{ 0x6341000, 0xca0c },
	{ 0x6359000, 0xc144 },
	{ 0x60d1000, 0xc8d4 },
	{ 0x60e5000, 0xa828 },
	{ 0x60f5000, 0xa7a8 },
	{ 0x6109000, 0xb924 },
	{ 0x6385000, 0xbb10 },
	{ 0x6371000, 0xb9c4 },
	{ 0x65ad000, 0xc0dc },
	{ 0x6399000, 0xbf50 },
	{ 0x63b1000, 0xbb34 },
	{ 0x63c5000, 0xc2cc },
	{ 0x63e1000, 0xbf60 },
	{ 0x63f1000, 0xb818 },
	{ 0x6401000, 0xc6b4 },
	{ 0x6415000, 0xb91c },
	{ 0x6425000, 0x5f6c },
	{ 0x6435000, 0xc1cc },
	{ 0x6449000, 0xcee0 },
	{ 0x645d000, 0xc6e4 },
	{ 0x6485000, 0xce40 },
	{ 0x6471000, 0xc720 },
	{ 0x6499000, 0xab68 },
	{ 0x64a9000, 0xcca4 },
	{ 0x64bd000, 0xab60 },
	{ 0x64cd000, 0xc118 },
	{ 0x64e1000, 0xc360 },
	{ 0x64f5000, 0xcc98 },
	{ 0x6509000, 0xc384 },
	{ 0x651d000, 0xcec8 },
	{ 0x6539000, 0xbdbc },
	{ 0x654d000, 0xc5e0 },
	{ 0x6565000, 0xb708 },
	{ 0x6579000, 0xc064 },
	{ 0x65c9000, 0xb368 },
	{ 0x65dd000, 0xaee4 },
	{ 0x65f1000, 0xa210 },
	{ 0x6605000, 0x9bf4 },
	{ 0x6621000, 0xc238 },
	{ 0x6635000, 0x393c },
	{ 0x663d000, 0xc638 },
	{ 0x6655000, 0xbd84 },
	{ 0x666d000, 0xbcb4 },
	{ 0x6685000, 0xd9f0 },
	{ 0x6699000, 0xda10 },
	{ 0x66c1000, 0xe430 },
	{ 0x66d5000, 0xebf0 },
	{ 0x66ad000, 0xda28 },
	{ 0x66e9000, 0xda40 },
	{ 0x66fd000, 0xdd08 },
	{ 0x6711000, 0xda34 },
	{ 0x6725000, 0xda2c },
	{ 0x6739000, 0xda30 },
	{ 0x674d000, 0xd9d4 },
	{ 0x6761000, 0xd8b8 },
	{ 0x6775000, 0xeba8 },
	{ 0x6789000, 0xd9cc },
	{ 0x679d000, 0xda2c },
	{ 0x67b1000, 0xd980 },
	{ 0x67cd000, 0xfb3c },
	{ 0x67e1000, 0xefa0 },
	{ 0x67fd000, 0xbc00 },
	{ 0x6829000, 0xb4b8 },
	{ 0x6839000, 0xb7fc },
	{ 0x6849000, 0xb578 },
	{ 0x6859000, 0xb5fc },
	{ 0x6869000, 0xbd6c },
	{ 0x6879000, 0xb508 },
	{ 0x6811000, 0xe110 },
	{ 0x68a9000, 0xbd1c },
	{ 0x68b9000, 0xe0c8 },
	{ 0x68d5000, 0xbb24 },
	{ 0x68e1000, 0xba68 },
	{ 0x6891000, 0xa2c0 },
	{ 0x68f9000, 0xbf00 },
	{ 0x6911000, 0xc10c },
	{ 0x692d000, 0xc114 },
	{ 0x693d000, 0xb014 },
	{ 0x6955000, 0xc7d0 },
	{ 0x696d000, 0xb7f0 },
	{ 0x697d000, 0xb7f0 },
	{ 0x6991000, 0xb79c },
	{ 0x69a5000, 0xb79c },
	{ 0x69b9000, 0xbab4 },
	{ 0x69cd000, 0xb8a0 },
	{ 0x69e5000, 0xb9f0 },
	{ 0x69f9000, 0xb958 },
	{ 0x6a39000, 0xbb10 },
	{ 0x6a11000, 0xbd94 },
	{ 0x6a25000, 0xbd28 },
	{ 0x6a51000, 0xb798 },
	{ 0x6a65000, 0xb8dc },
	{ 0x6a79000, 0xc2fc },
	{ 0x6a91000, 0xb8b0 },
	{ 0x6aa5000, 0xb420 },
	{ 0x6ad5000, 0xbc18 },
	{ 0x6ae9000, 0xb61c },
	{ 0x6afd000, 0xb7e4 },
	{ 0x6abd000, 0xb8d4 },
	{ 0x6b15000, 0xb948 },
	{ 0x6b29000, 0xb928 },
	{ 0x6b41000, 0xd5fc },
	{ 0x6b55000, 0xd4a0 },
	{ 0x6b69000, 0xd5e4 },
	{ 0x6b7d000, 0x61b8 },
	{ 0x6b89000, 0xd490 },
	{ 0x6ba5000, 0xc9c8 },
	{ 0x6bc1000, 0x5454 },
	{ 0x6bd5000, 0xbb7c },
	{ 0x6bed000, 0xbff0 },
	{ 0x6c01000, 0xf0dc },
	{ 0x6c15000, 0xf0d8 },
	{ 0x6c59000, 0xc434 },
	{ 0x6c2d000, 0xd210 },
	{ 0x6c41000, 0xba04 },
	{ 0x6c71000, 0xc850 },
	{ 0x6c89000, 0x116e8 },
	{ 0x6cb1000, 0x132dc },
	{ 0x6ca1000, 0x93f0 },
	{ 0x6cd1000, 0xcec8 },
	{ 0x6d11000, 0xc97c },
	{ 0x6d29000, 0xb79c },
	{ 0x6cf9000, 0xb8d8 },
	{ 0x6ced000, 0xb964 },
	{ 0x6d3d000, 0xbdc8 },
	{ 0x6d51000, 0xc778 },
	{ 0x6d6d000, 0x32cc },
	{ 0x6d81000, 0xbed0 },
	{ 0x7151000, 0xb904 },
	{ 0x6d99000, 0xb850 },
	{ 0x6dad000, 0xbaa4 },
	{ 0x6dd5000, 0xbdec },
	{ 0x6ded000, 0xc99c },
	{ 0x6dc1000, 0xb3dc },
	{ 0x6e49000, 0xca88 },
	{ 0x6e09000, 0xc0b0 },
	{ 0x6e25000, 0xbdd0 },
	{ 0x6e35000, 0xbdd0 },
	{ 0x6e61000, 0xbee4 },
	{ 0x6e79000, 0xbdc0 },
	{ 0x6e91000, 0xb8f0 },
	{ 0x6ea5000, 0xba58 },
	{ 0x6ebd000, 0xc208 },
	{ 0x6f09000, 0xb250 },
	{ 0x6f1d000, 0xbba4 },
	{ 0x6f31000, 0xc720 },
	{ 0x6ed9000, 0xb428 },
	{ 0x6ef1000, 0xbbb8 },
	{ 0x6f4d000, 0xc364 },
	{ 0x6f69000, 0xbb8c },
	{ 0x6f7d000, 0xd70c },
	{ 0x6f99000, 0xc6e4 },
	{ 0x7165000, 0xb950 },
	{ 0x6fc9000, 0xb938 },
	{ 0x6ff1000, 0xc2a4 },
	{ 0x6fe1000, 0xc254 },
	{ 0x6fb5000, 0xb9a4 },
	{ 0x700d000, 0xb9c8 },
	{ 0x7065000, 0xc388 },
	{ 0x7021000, 0xbd54 },
	{ 0x7039000, 0xbcb8 },
	{ 0x707d000, 0xbb04 },
	{ 0x7095000, 0xc010 },
	{ 0x70a9000, 0xc030 },
	{ 0x7051000, 0xa408 },
	{ 0x70c5000, 0xc5f0 },
	{ 0x70e1000, 0xb8f0 },
	{ 0x70f9000, 0xbb30 },
	{ 0x7111000, 0xbdbc },
	{ 0x7125000, 0xb990 },
	{ 0x7139000, 0xbb84 },
	{ 0x7179000, 0xb7b0 },
	{ 0x7189000, 0xb744 },
	{ 0x71d9000, 0x20 },
	{ 0x71dd000, 0x4a2c },
	{ 0x71e5000, 0x378d },
	{ 0x71e9000, 0x3ece },
	{ 0x5309000, 0x2cf },
	{ 0x76ad000, 0x2c9 },
	{ 0x4a71000, 0x134 },
	{ 0x4a75000, 0x1b28 },
	{ 0x4a79000, 0x4c0c0 },
	{ 0x4ac9000, 0x8f },
	{ 0x4acd000, 0x3b80 },
	{ 0x4ad1000, 0x238 },
	{ 0x4ad5000, 0x98 },
	{ 0x4ad9000, 0xb3c3 },
	{ 0x4ae5000, 0x1c5c },
	{ 0x4f29000, 0x3df2b6 },
	{ 0x71ed000, 0x493ba },
	{ 0x4ae9000, 0x134 },
	{ 0x4aed000, 0x1b28 },
	{ 0x4af1000, 0x55b1 },
	{ 0x4af9000, 0x98 },
	{ 0x4afd000, 0x460 },
	{ 0x4b01000, 0xdc },
	{ 0x4b05000, 0x98 },
	{ 0x4b09000, 0x102c },
	{ 0x4b0d000, 0x1c5c },
	{ 0x4b11000, 0x134 },
	{ 0x4b15000, 0x1b28 },
	{ 0x4b19000, 0x43c7c },
	{ 0x4b5d000, 0xcc },
	{ 0x4b61000, 0x1688bc },
	{ 0x4ccd000, 0x4c0 },
	{ 0x4cd1000, 0x98 },
	{ 0x4cd5000, 0xb1a5 },
	{ 0x4ce1000, 0x1c5c },
	{ 0x4ce5000, 0x200 },
	{ 0x4ce9000, 0x1b28 },
	{ 0x4ced000, 0x4ff70 },
	{ 0x4d3d000, 0xf8 },
	{ 0x4d41000, 0x16b250 },
	{ 0x4ead000, 0x980 },
	{ 0x4eb1000, 0x130 },
	{ 0x4eb5000, 0x1452b },
	{ 0x4ecd000, 0x1d28 },
	{ 0x4ed1000, 0x200 },
	{ 0x4ed5000, 0x1b28 },
	{ 0x4ed9000, 0xcecd },
	{ 0x4ee9000, 0xc2 },
	{ 0x4eed000, 0x840 },
	{ 0x4ef1000, 0x288 },
	{ 0x4ef5000, 0x130 },
	{ 0x4ef9000, 0x1b48 },
	{ 0x4efd000, 0x1d28 },
	{ 0x4f01000, 0x134 },
	{ 0x4f05000, 0x1b28 },
	{ 0x4f09000, 0x7d3b },
	{ 0x4f11000, 0xbb },
	{ 0x4f15000, 0x334 },
	{ 0x4f19000, 0x10c },
	{ 0x4f1d000, 0x98 },
	{ 0x4f21000, 0x1510 },
	{ 0x4f25000, 0x1c5c },
	{ 0x7239000, 0x2ecfa },
	{ 0x7269000, 0x3a39c0 },
};
static const struct { const char *path; uint32_t size; uint16_t ext, next; } fw_all[] = {
	{ "image/adsp.b00", 0x4d4, 0, 1 },
	{ "image/adsp.b01", 0x2098, 1, 1 },
	{ "image/adsp.b02", 0x14ef4, 2, 1 },
	{ "image/adsp.b03", 0x5e9c0, 3, 1 },
	{ "image/adsp.b04", 0x18f388, 4, 1 },
	{ "image/adsp.b05", 0x19f628, 5, 1 },
	{ "image/adsp.b06", 0xb60, 6, 1 },
	{ "image/adsp.b07", 0x63e4, 7, 1 },
	{ "image/adsp.b08", 0x6f6c, 8, 1 },
	{ "image/adsp.b09", 0x12dc, 9, 1 },
	{ "image/adsp.b10", 0x104, 10, 1 },
	{ "image/adsp.b12", 0xf2a0, 11, 1 },
	{ "image/adsp.b13", 0x11ae4, 12, 1 },
	{ "image/adsp.b14", 0xa4000, 13, 1 },
	{ "image/adsp.b15", 0x170484, 14, 1 },
	{ "image/adsp.b16", 0x44004, 15, 1 },
	{ "image/adsp.b17", 0x2dfb, 16, 1 },
	{ "image/adsp.b18", 0x1b60, 17, 1 },
	{ "image/adsp.b19", 0x2dc, 18, 1 },
	{ "image/adsp.b20", 0x1f60, 19, 1 },
	{ "image/adsp.b21", 0x6fec, 20, 1 },
	{ "image/adsp.b22", 0x3a56c, 21, 1 },
	{ "image/adsp.b23", 0x94000, 22, 1 },
	{ "image/adsp.b24", 0x4db23e, 23, 1 },
	{ "image/adsp.b25", 0x15dd6a, 24, 1 },
	{ "image/adsp.b26", 0x423c, 25, 1 },
	{ "image/adsp.b27", 0x51a74, 26, 1 },
	{ "image/adsp.b28", 0xb70, 27, 1 },
	{ "image/adsp.b29", 0xd1f98, 28, 1 },
	{ "image/adsp.b30", 0x50230, 29, 1 },
	{ "image/adsp.b31", 0xe00, 30, 1 },
	{ "image/adsp.b32", 0x13f0, 31, 1 },
	{ "image/adsp.b33", 0x23810, 32, 1 },
	{ "image/adsp.b34", 0x116400, 33, 1 },
	{ "image/adsp.b35", 0xd4, 34, 1 },
	{ "image/adsp.mdt", 0x256c, 35, 1 },
	{ "image/adspr.jsn", 0x193, 36, 1 },
	{ "image/adsps.jsn", 0x195, 37, 1 },
	{ "image/adspua.jsn", 0x22b, 38, 1 },
	{ "image/bdm7eu.bin", 0x66d8, 39, 1 },
	{ "image/bdm7eut.bin", 0x66d8, 40, 1 },
	{ "image/bdm7euu.bin", 0x66d8, 41, 1 },
	{ "image/bdm7gl.bin", 0x66d8, 42, 1 },
	{ "image/bdm7glt.bin", 0x66d8, 43, 1 },
	{ "image/bdm7glu.bin", 0x66d8, 44, 1 },
	{ "image/bdm7in.bin", 0x66d8, 45, 1 },
	{ "image/bdm7int.bin", 0x66d8, 46, 1 },
	{ "image/bdm7inu.bin", 0x66d8, 47, 1 },
	{ "image/bdm7mx.bin", 0x66d8, 48, 1 },
	{ "image/bdm7mxt.bin", 0x66d8, 49, 1 },
	{ "image/bdm7mxu.bin", 0x66d8, 50, 1 },
	{ "image/bdwlan.102", 0x66d8, 51, 1 },
	{ "image/bdwlan.104", 0x66d8, 52, 1 },
	{ "image/bdwlan.105", 0x66d8, 53, 1 },
	{ "image/bdwlan.106", 0x66d8, 54, 1 },
	{ "image/bdwlan.107", 0x66d8, 55, 1 },
	{ "image/bdwlan.108", 0x66d8, 56, 1 },
	{ "image/bdwlan.109", 0x66d8, 57, 1 },
	{ "image/bdwlan.10e", 0x66d8, 58, 1 },
	{ "image/bdwlan.10f", 0x66d8, 59, 1 },
	{ "image/bdwlan.111", 0x66d8, 60, 1 },
	{ "image/bdwlan.202", 0x66d8, 61, 1 },
	{ "image/bdwlan.203", 0x66d8, 62, 1 },
	{ "image/bdwlan.204", 0x66d8, 63, 1 },
	{ "image/bdwlan.206", 0x66d8, 64, 1 },
	{ "image/bdwlan.b04", 0x66d8, 65, 1 },
	{ "image/bdwlan.b07", 0x66d8, 66, 1 },
	{ "image/bdwlan.b09", 0x66d8, 67, 1 },
	{ "image/bdwlan.b0a", 0x66d8, 68, 1 },
	{ "image/bdwlan.b0b", 0x66d8, 69, 1 },
	{ "image/bdwlan.b0d", 0x66d8, 70, 1 },
	{ "image/bdwlan.b0e", 0x66d8, 71, 1 },
	{ "image/bdwlan.b0f", 0x66d8, 72, 1 },
	{ "image/bdwlan.b14", 0x66d8, 73, 1 },
	{ "image/bdwlan.b15", 0x66d8, 74, 1 },
	{ "image/bdwlan.b30", 0x66d8, 75, 1 },
	{ "image/bdwlan.b31", 0x66d8, 76, 1 },
	{ "image/bdwlan.b32", 0x66d8, 77, 1 },
	{ "image/bdwlan.b33", 0x66d8, 78, 1 },
	{ "image/bdwlan.b34", 0x66d8, 79, 1 },
	{ "image/bdwlan.b35", 0x66d8, 80, 1 },
	{ "image/bdwlan.b36", 0x66d8, 81, 1 },
	{ "image/bdwlan.b37", 0x66d8, 82, 1 },
	{ "image/bdwlan.b38", 0x66d8, 83, 1 },
	{ "image/bdwlan.bin", 0x66d8, 84, 1 },
	{ "image/bdwlans.202", 0x66d8, 85, 1 },
	{ "image/bdwlant.202", 0x66d8, 86, 1 },
	{ "image/bdwlanu.202", 0x66d8, 87, 1 },
	{ "image/bdwlanu.206", 0x66d8, 88, 1 },
	{ "image/bdwlanu.206", 0x66d8, 89, 1 },
	{ "image/cardapp.b00", 0x134, 90, 1 },
	{ "image/cardapp.b01", 0x1b28, 91, 1 },
	{ "image/cardapp.b02", 0x97d3, 92, 1 },
	{ "image/cardapp.b03", 0x91, 93, 1 },
	{ "image/cardapp.b04", 0x2b8, 94, 1 },
	{ "image/cardapp.b05", 0x14c, 95, 1 },
	{ "image/cardapp.b06", 0x98, 96, 1 },
	{ "image/cardapp.b07", 0x1cab, 97, 1 },
	{ "image/cardapp.mdt", 0x1c5c, 98, 1 },
	{ "image/cdsp.b00", 0x1d4, 99, 1 },
	{ "image/cdsp.b01", 0x1c18, 100, 1 },
	{ "image/cdsp.b02", 0x39898, 101, 1 },
	{ "image/cdsp.b03", 0x1a07c, 102, 1 },
	{ "image/cdsp.b04", 0x44c, 103, 1 },
	{ "image/cdsp.b05", 0x20db5c, 104, 1 },
	{ "image/cdsp.b06", 0x5b60, 105, 1 },
	{ "image/cdsp.b08", 0x95378, 106, 1 },
	{ "image/cdsp.b09", 0xa20, 107, 1 },
	{ "image/cdsp.b11", 0xe614, 108, 1 },
	{ "image/cdsp.mdt", 0x1dec, 109, 1 },
	{ "image/cdspr.jsn", 0x193, 110, 1 },
	{ "image/data.msc", 0x9c2df, 111, 1 },
	{ "image/featenab.b00", 0x200, 112, 1 },
	{ "image/featenab.b01", 0x4e70, 113, 1 },
	{ "image/featenab.b02", 0xa89d, 114, 1 },
	{ "image/featenab.b03", 0xc4, 115, 1 },
	{ "image/featenab.b04", 0x391, 116, 1 },
	{ "image/featenab.b05", 0x1c0, 117, 1 },
	{ "image/featenab.b06", 0x130, 118, 1 },
	{ "image/featenab.b07", 0x2231, 119, 1 },
	{ "image/featenab.mdt", 0x5070, 120, 1 },
	{ "image/fpctzapp.b00", 0x200, 121, 1 },
	{ "image/fpctzapp.b01", 0x1b28, 122, 1 },
	{ "image/fpctzapp.b02", 0x109ed8, 123, 1 },
	{ "image/fpctzapp.b03", 0x144, 124, 1 },
	{ "image/fpctzapp.b04", 0x103760, 125, 1 },
	{ "image/fpctzapp.b05", 0x468, 126, 1 },
	{ "image/fpctzapp.b06", 0x130, 127, 1 },
	{ "image/fpctzapp.b07", 0x24749, 128, 1 },
	{ "image/fpctzapp.mdt", 0x1d28, 129, 1 },
	{ "image/goodixfp.b00", 0x134, 130, 1 },
	{ "image/goodixfp.b01", 0x1b28, 131, 1 },
	{ "image/goodixfp.b02", 0x3b8c68, 132, 1 },
	{ "image/goodixfp.b03", 0x9b, 133, 1 },
	{ "image/goodixfp.b04", 0x2763a4, 134, 1 },
	{ "image/goodixfp.b05", 0x230, 135, 1 },
	{ "image/goodixfp.b06", 0xa8, 136, 1 },
	{ "image/goodixfp.b07", 0x268d6, 137, 1 },
	{ "image/goodixfp.mdt", 0x1c5c, 138, 1 },
	{ "image/gptest.b00", 0x134, 139, 1 },
	{ "image/gptest.b01", 0x1b28, 140, 1 },
	{ "image/gptest.b02", 0x1430b, 141, 1 },
	{ "image/gptest.b03", 0xd4, 142, 1 },
	{ "image/gptest.b04", 0x6bb4, 143, 1 },
	{ "image/gptest.b05", 0x430, 144, 1 },
	{ "image/gptest.b06", 0x98, 145, 1 },
	{ "image/gptest.b07", 0x6c49, 146, 1 },
	{ "image/gptest.mdt", 0x1c5c, 147, 1 },
	{ "image/hdcpsrm.b00", 0x200, 148, 1 },
	{ "image/hdcpsrm.b01", 0x1b28, 149, 1 },
	{ "image/hdcpsrm.b02", 0x541d, 150, 1 },
	{ "image/hdcpsrm.b03", 0xc2, 151, 1 },
	{ "image/hdcpsrm.b04", 0x388, 152, 1 },
	{ "image/hdcpsrm.b05", 0x1c0, 153, 1 },
	{ "image/hdcpsrm.b06", 0x130, 154, 1 },
	{ "image/hdcpsrm.b07", 0x137d, 155, 1 },
	{ "image/hdcpsrm.mdt", 0x1d28, 156, 1 },
	{ "image/loadalgo.b00", 0x200, 157, 1 },
	{ "image/loadalgo.b01", 0x1b28, 158, 1 },
	{ "image/loadalgo.b02", 0xc0f5, 159, 1 },
	{ "image/loadalgo.b03", 0xc5, 160, 1 },
	{ "image/loadalgo.b04", 0x4c60, 161, 1 },
	{ "image/loadalgo.b05", 0x180, 162, 1 },
	{ "image/loadalgo.b06", 0x130, 163, 1 },
	{ "image/loadalgo.b07", 0x10a9, 164, 1 },
	{ "image/loadalgo.mdt", 0x1d28, 165, 1 },
	{ "image/mfido.b00", 0x134, 166, 1 },
	{ "image/mfido.b01", 0x1b28, 167, 1 },
	{ "image/mfido.b02", 0x2bb74, 168, 1 },
	{ "image/mfido.b03", 0x89, 169, 1 },
	{ "image/mfido.b04", 0xe47c, 170, 1 },
	{ "image/mfido.b05", 0x188, 171, 1 },
	{ "image/mfido.b06", 0x98, 172, 1 },
	{ "image/mfido.b07", 0x68ee, 173, 1 },
	{ "image/mfido.mdt", 0x1c5c, 174, 1 },
	{ "image/miriskm.b00", 0x200, 175, 1 },
	{ "image/miriskm.b01", 0x1b28, 176, 1 },
	{ "image/miriskm.b02", 0x1b20a, 177, 1 },
	{ "image/miriskm.b03", 0xbe, 178, 1 },
	{ "image/miriskm.b04", 0xc7c, 179, 1 },
	{ "image/miriskm.b05", 0x298, 180, 1 },
	{ "image/miriskm.b06", 0x130, 181, 1 },
	{ "image/miriskm.b07", 0x3931, 182, 1 },
	{ "image/miriskm.mdt", 0x1d28, 183, 1 },
	{ "image/mlipay.b00", 0x134, 184, 1 },
	{ "image/mlipay.b01", 0x1b28, 185, 1 },
	{ "image/mlipay.b02", 0xaf99, 186, 1 },
	{ "image/mlipay.b03", 0x8a, 187, 1 },
	{ "image/mlipay.b04", 0xa88, 188, 1 },
	{ "image/mlipay.b05", 0x174, 189, 1 },
	{ "image/mlipay.b06", 0x98, 190, 1 },
	{ "image/mlipay.b07", 0x231b, 191, 1 },
	{ "image/mlipay.mdt", 0x1c5c, 192, 1 },
	{ "image/modem.b00", 0x414, 193, 1 },
	{ "image/modem.b01", 0x1f78, 194, 1 },
	{ "image/modem.b02", 0x2930, 195, 1 },
	{ "image/modem.b03", 0x2518, 196, 1 },
	{ "image/modem.b04", 0x4a3ac, 197, 1 },
	{ "image/modem.b05", 0x119d8, 198, 1 },
	{ "image/modem.b06", 0x100c, 199, 1 },
	{ "image/modem.b07", 0x90b4, 200, 1 },
	{ "image/modem.b08", 0x4f054, 201, 1 },
	{ "image/modem.b09", 0x143ee8, 202, 1 },
	{ "image/modem.b10", 0x40000, 203, 1 },
	{ "image/modem.b11", 0x540, 204, 1 },
	{ "image/modem.b12", 0xe2210, 205, 1 },
	{ "image/modem.b14", 0x2d, 206, 1 },
	{ "image/modem.b15", 0x7c, 207, 1 },
	{ "image/modem.b16", 0x9c1a0, 208, 1 },
	{ "image/modem.b17", 0x61110, 209, 1 },
	{ "image/modem.b18", 0x2d29a9, 210, 1 },
	{ "image/modem.b19", 0x2d94cc, 211, 1 },
	{ "image/modem.b22", 0x10ec0, 212, 1 },
	{ "image/modem.b23", 0x1722040, 213, 1 },
	{ "image/modem.b24", 0xa07900, 214, 1 },
	{ "image/modem.b25", 0x77a45, 215, 1 },
	{ "image/modem.b26", 0x15104, 216, 1 },
	{ "image/modem.b27", 0x1378, 217, 1 },
	{ "image/modem.b28", 0xb0, 218, 1 },
	{ "image/modem.b29", 0x7d52c, 219, 1 },
	{ "image/modem.mdt", 0x238c, 220, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/cmcc_sub/sr_dsds/mcfg_hw.mbn", 0x588c, 221, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/7+7_mode/sr_dsds/mcfg_hw.mbn", 0x4404, 222, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/generic/common/divar/la/ss/mcfg_hw.mbn", 0x402c, 223, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.dig", 0x20, 224, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/mbn_hw.txt", 0x232, 225, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_hw/oem_hw.txt", 0x1, 226, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/cellc/commerci/safrica/mcfg_sw.mbn", 0xb4ac, 227, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/kenya/airtel/commerci/mcfg_sw.mbn", 0xb838, 228, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/inwi/commerci/mcfg_sw.mbn", 0xb7d8, 229, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/morocco/orange/commerci/mcfg_sw.mbn", 0xbdc8, 230, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/af/moroccot/commerci/ma/mcfg_sw.mbn", 0xb50c, 231, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/airtel/commerci/mcfg_sw.mbn", 0xc048, 232, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/asiacell/commerci/mcfg_sw.mbn", 0xc16c, 233, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/bakcell/commerci/mcfg_sw.mbn", 0xc328, 234, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/commerci/mcfg_sw.mbn", 0xbf28, 235, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/beeline/uzbe/commerci/mcfg_sw.mbn", 0xc418, 236, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/cellcom/commerci/israel/mcfg_sw.mbn", 0xc2bc, 237, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dcm/commerci/mcfg_sw.mbn", 0xbb40, 238, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/dialog/commerci/sl/mcfg_sw.mbn", 0xbd1c, 239, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/grameenp/commerci/mcfg_sw.mbn", 0xbc08, 240, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/htmobile/commerci/mcfg_sw.mbn", 0xc464, 241, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/idea/commerci/mcfg_sw.mbn", 0xbc00, 242, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jazz/commerci/mcfg_sw.mbn", 0xb7cc, 243, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/jtl/commerci/mcfg_sw.mbn", 0xb898, 244, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kddi/commerci/mcfg_sw.mbn", 0xeb6c, 245, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/kenya/commerci/mcfg_sw.mbn", 0xc05c, 246, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/mobitel/commerci/mcfg_sw.mbn", 0xb964, 247, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ncell/commerci/nepal/mcfg_sw.mbn", 0xb948, 248, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ntc/commerci/nepal/mcfg_sw.mbn", 0xbd34, 249, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ooredoo/commerci/mcfg_sw.mbn", 0xaff0, 250, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/partner/commerci/mcfg_sw.mbn", 0xbedc, 251, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/pelephon/commerci/israel/mcfg_sw.mbn", 0xc240, 252, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/rakuten/commerci/mcfg_sw.mbn", 0xf490, 253, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/reliance/commerci/mcfg_sw.mbn", 0x1011c, 254, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/robi/commerci/mcfg_sw.mbn", 0xb388, 255, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/sbm/commerci/mcfg_sw.mbn", 0xb8b8, 256, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/tele2/commerci/mcfg_sw.mbn", 0xbc40, 257, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/telenor/commerci/mcfg_sw.mbn", 0xb8d8, 258, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ucell/commerci/mcfg_sw.mbn", 0xb80c, 259, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/ufone/commerci/mcfg_sw.mbn", 0xc2dc, 260, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/uzmobile/commerci/mcfg_sw.mbn", 0xb7dc, 261, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zain/commerci/mcfg_sw.mbn", 0xb940, 262, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/apac/zong/commerci/mcfg_sw.mbn", 0xbcf4, 263, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/optus/commerci/au/mcfg_sw.mbn", 0xc4a4, 264, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/telstra/commerci/mcfg_sw.mbn", 0xc794, 265, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/aunz/vodafone/commerci/mcfg_sw.mbn", 0xc148, 266, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_cm/mcfg_sw.mbn", 0xd030, 267, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/commerci/volte_op/mcfg_sw.mbn", 0xe098, 268, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/agnss_lo/mcfg_sw.mbn", 0x6c5c, 269, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/conf_vol/mcfg_sw.mbn", 0xcc88, 270, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/eps_only/mcfg_sw.mbn", 0x5eb0, 271, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/lpp_loct/mcfg_sw.mbn", 0x6c64, 272, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_4r/mcfg_sw.mbn", 0xc664, 273, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/nsiot_vo/mcfg_sw.mbn", 0xc7e4, 274, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/rrlp_loc/mcfg_sw.mbn", 0x6c64, 275, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/tgl_comb/mcfg_sw.mbn", 0x5b60, 276, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cmcc/lab/w_irat_c/mcfg_sw.mbn", 0x5e04, 277, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/hvolte_o/mcfg_sw.mbn", 0x119bc, 278, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/openmkt/mcfg_sw.mbn", 0xa698, 279, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/commerci/volte_op/mcfg_sw.mbn", 0x11c50, 280, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/cta/mcfg_sw.mbn", 0x7618, 281, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/eps_only/mcfg_sw.mbn", 0xe4c0, 282, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/noapn_vo/mcfg_sw.mbn", 0xe120, 283, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test/mcfg_sw.mbn", 0x9fa0, 284, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_eps/mcfg_sw.mbn", 0xa060, 285, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/test_no_/mcfg_sw.mbn", 0x9cc4, 286, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ct/lab/volte_co/mcfg_sw.mbn", 0xe350, 287, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/ctm/commerci/mcfg_sw.mbn", 0xbcc0, 288, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/openmkt/mcfg_sw.mbn", 0x690c, 289, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/commerci/volte/mcfg_sw.mbn", 0xd604, 290, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/china/cu/lab/test/mcfg_sw.mbn", 0x52f4, 291, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/row/commerci/mcfg_sw.mbn", 0xb7f4, 292, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/test/commerci/mcfg_sw.mbn", 0xc568, 293, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/common/w_one/commerci/mcfg_sw.mbn", 0xab94, 294, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/1and1/commerci/de/mcfg_sw.mbn", 0xc878, 295, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/2degress/commerci/mcfg_sw.mbn", 0xbbc0, 296, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/4ka/commerci/slovenia/mcfg_sw.mbn", 0xbbf0, 297, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/at_comme/mcfg_sw.mbn", 0xb824, 298, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/bulgaria/mcfg_sw.mbn", 0xb90c, 299, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/mcfg_sw.mbn", 0xbbfc, 300, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/commerci/slovenia/mcfg_sw.mbn", 0xc304, 301, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/croatia_/mcfg_sw.mbn", 0xbeec, 302, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/macedoni/mcfg_sw.mbn", 0xbe0c, 303, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/a1/serbia_c/mcfg_sw.mbn", 0xb88c, 304, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bh/commerci/ba/mcfg_sw.mbn", 0xb8f4, 305, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/latvia/mcfg_sw.mbn", 0xbbe0, 306, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bite/commerci/lithuani/mcfg_sw.mbn", 0xbbf8, 307, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/bouygues/commerci/france/mcfg_sw.mbn", 0xc734, 308, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/btbd/commerci/mcfg_sw.mbn", 0xb854, 309, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/coop/commerci/italy/mcfg_sw.mbn", 0xc7c4, 310, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/cosmote/commerci/mcfg_sw.mbn", 0xd034, 311, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/hungary/mcfg_sw.mbn", 0xc4c8, 312, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/commerci/spain/mcfg_sw.mbn", 0xbf00, 313, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/digi/romania_/mcfg_sw.mbn", 0xb448, 314, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/austria/mcfg_sw.mbn", 0xcd68, 315, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/croatia/mcfg_sw.mbn", 0xcac8, 316, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/cz/mcfg_sw.mbn", 0xccfc, 317, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/greece/mcfg_sw.mbn", 0xd02c, 318, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/hungary/mcfg_sw.mbn", 0xd688, 319, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/macedoni/mcfg_sw.mbn", 0xbff8, 320, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/nl/mcfg_sw.mbn", 0xca3c, 321, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/pl/mcfg_sw.mbn", 0xcdc4, 322, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/commerci/slovakia/mcfg_sw.mbn", 0xc82c, 323, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/non_volt/commerci/mcfg_sw.mbn", 0x5b58, 324, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/commerci/mcfg_sw.mbn", 0xca60, 325, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/dt/volte/romania_/mcfg_sw.mbn", 0xca98, 326, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/du/commerci/uae/mcfg_sw.mbn", 0xbb48, 327, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ee/commerci/mcfg_sw.mbn", 0xc688, 328, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/eir/commerci/ireland/mcfg_sw.mbn", 0xb7d8, 329, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/estonia/mcfg_sw.mbn", 0xbdf0, 330, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/elisa/commerci/fi/mcfg_sw.mbn", 0xbfe0, 331, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/enetwork/commerci/mcfg_sw.mbn", 0xbc40, 332, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/egypt/mcfg_sw.mbn", 0xb944, 333, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/etisalat/commerci/uae/mcfg_sw.mbn", 0xbecc, 334, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/fastweb/commerci/italy/mcfg_sw.mbn", 0xd440, 335, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/free/volte/france/mcfg_sw.mbn", 0xbac8, 336, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/gibtel/commerci/gibra/mcfg_sw.mbn", 0xba58, 337, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/austria/mcfg_sw.mbn", 0xb954, 338, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/denmark/mcfg_sw.mbn", 0xbf58, 339, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/ireland/mcfg_sw.mbn", 0xbec4, 340, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/italy/mcfg_sw.mbn", 0xd450, 341, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/se/mcfg_sw.mbn", 0xc0d0, 342, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/h3g/commerci/uk/mcfg_sw.mbn", 0xbba4, 343, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ice/commerci/norway/mcfg_sw.mbn", 0xc1cc, 344, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kpn/commerci/mcfg_sw.mbn", 0xc03c, 345, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/kyvistar/volte/ukraine/mcfg_sw.mbn", 0xc018, 346, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lebara/commerci/france/mcfg_sw.mbn", 0xbce8, 347, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lifecell/volte/ukraine/mcfg_sw.mbn", 0xc0cc, 348, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/lmt/commerci/latvia/mcfg_sw.mbn", 0xb5e4, 349, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/meo/commerci/mcfg_sw.mbn", 0xc828, 350, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/moldcell/commerci/moldova/mcfg_sw.mbn", 0xb3fc, 351, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtel/commerci/ba/mcfg_sw.mbn", 0xb8f4, 352, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/ng/mcfg_sw.mbn", 0xc190, 353, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mtn/commerci/safrica/mcfg_sw.mbn", 0xc94c, 354, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/armenia/mcfg_sw.mbn", 0xbf18, 355, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/commerci/belarus/mcfg_sw.mbn", 0xbb1c, 356, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/mts/volte/serbia/mcfg_sw.mbn", 0xb95c, 357, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nos/commerci/mcfg_sw.mbn", 0xbb00, 358, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/nova/commerci/mcfg_sw.mbn", 0xce10, 359, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/czech/mcfg_sw.mbn", 0xc678, 360, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/commerci/slovakia/mcfg_sw.mbn", 0xc7e4, 361, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/o2/uk_comme/mcfg_sw.mbn", 0xc5dc, 362, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/ora/commerci/france/mcfg_sw.mbn", 0xb8b8, 363, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/belgium/mcfg_sw.mbn", 0xbf04, 364, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/caraibes/mcfg_sw.mbn", 0xb8fc, 365, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/egypt/mcfg_sw.mbn", 0xc1d0, 366, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/france/mcfg_sw.mbn", 0xd5ac, 367, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/group_no/mcfg_sw.mbn", 0x6180, 368, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/luxembou/mcfg_sw.mbn", 0xb84c, 369, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/poland/mcfg_sw.mbn", 0xdc88, 370, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/reunion/mcfg_sw.mbn", 0xb8f8, 371, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/romania/mcfg_sw.mbn", 0xc9c0, 372, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/slovakia/mcfg_sw.mbn", 0xc5d0, 373, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/commerci/spain/mcfg_sw.mbn", 0xcbb4, 374, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/orange/moldova_/mcfg_sw.mbn", 0xaf7c, 375, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/play/poland_c/mcfg_sw.mbn", 0xcc50, 376, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plintron/commerci/italy/mcfg_sw.mbn", 0xb8cc, 377, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/plus/poland_c/mcfg_sw.mbn", 0xcf44, 378, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/post/commerci/luxembou/mcfg_sw.mbn", 0xb9d8, 379, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/commerci/mcfg_sw.mbn", 0xbc4c, 380, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/proximus/volte/luxembou/mcfg_sw.mbn", 0xb8e8, 381, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/salt/commerci/mcfg_sw.mbn", 0xc568, 382, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/altice/mcfg_sw.mbn", 0xbe88, 383, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sfr/commerci/fr/mcfg_sw.mbn", 0xbd30, 384, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spark/commerci/mcfg_sw.mbn", 0xc150, 385, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/at/mcfg_sw.mbn", 0xbaf8, 386, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/it/mcfg_sw.mbn", 0xbcf8, 387, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/spusu/commerci/uk/mcfg_sw.mbn", 0xbde0, 388, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/sunrise/commerci/mcfg_sw.mbn", 0xb8c8, 389, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/swisscom/commerci/swiss/mcfg_sw.mbn", 0xc2d0, 390, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tdc/commerci/denmark/mcfg_sw.mbn", 0xbd74, 391, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/estonia/mcfg_sw.mbn", 0xc7e0, 392, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/latvia/mcfg_sw.mbn", 0xc7dc, 393, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/lithuani/mcfg_sw.mbn", 0xc800, 394, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/nl/mcfg_sw.mbn", 0xbf6c, 395, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tele2/commerci/sweden/mcfg_sw.mbn", 0xb4b8, 396, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/de/mcfg_sw.mbn", 0xbff8, 397, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/spain/mcfg_sw.mbn", 0xc9a0, 398, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/commerci/uk/mcfg_sw.mbn", 0xbd20, 399, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telefoni/non_volt/spain/mcfg_sw.mbn", 0xc8a4, 400, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/argentin/mcfg_sw.mbn", 0xb288, 401, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telekom/commerci/slovenia/mcfg_sw.mbn", 0xbdb0, 402, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/commerci/mcfg_sw.mbn", 0xb9fc, 403, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telemach/hr_comme/mcfg_sw.mbn", 0xb4a0, 404, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenet/volte/belgium/mcfg_sw.mbn", 0xbb18, 405, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/bulgaria/mcfg_sw.mbn", 0xb6c8, 406, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/denmark/mcfg_sw.mbn", 0xc754, 407, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/finland/mcfg_sw.mbn", 0xb9a8, 408, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/hungary/mcfg_sw.mbn", 0xbda8, 409, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/monteneg/mcfg_sw.mbn", 0xb8b8, 410, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/myanmar/mcfg_sw.mbn", 0xb524, 411, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/norway/mcfg_sw.mbn", 0xbf44, 412, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/serbia/mcfg_sw.mbn", 0xc224, 413, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telenor/commerci/sweden/mcfg_sw.mbn", 0xbf18, 414, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/denmark/mcfg_sw.mbn", 0xc478, 415, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/estonia/mcfg_sw.mbn", 0xc2b0, 416, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/finland/mcfg_sw.mbn", 0xc7fc, 417, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/lithuani/mcfg_sw.mbn", 0xc314, 418, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/norway/mcfg_sw.mbn", 0xcb54, 419, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telia/commerci/sweden/mcfg_sw.mbn", 0xca0c, 420, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/telkom/commerci/mcfg_sw.mbn", 0xc144, 421, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/italy/mcfg_sw.mbn", 0xc8d4, 422, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/kena/mcfg_sw.mbn", 0xa828, 423, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tim/commerci/welcome/mcfg_sw.mbn", 0xa7a8, 424, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/tmo/poland_c/mcfg_sw.mbn", 0xb924, 425, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turkcell/commerci/mcfg_sw.mbn", 0xbb10, 426, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/turktele/commerci/mcfg_sw.mbn", 0xb9c4, 427, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vee/commerci/mcfg_sw.mbn", 0xc0dc, 428, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vimla/commerci/mcfg_sw.mbn", 0xbf50, 429, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/virgin/commerci/uk/mcfg_sw.mbn", 0xbb34, 430, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vivacom/commerci/mcfg_sw.mbn", 0xc2cc, 431, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/albania/mcfg_sw.mbn", 0xbf60, 432, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/hungary/mcfg_sw.mbn", 0xb818, 433, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/commerci/ireland/mcfg_sw.mbn", 0xc6b4, 434, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/india_vo/mcfg_sw.mbn", 0xb91c, 435, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/non_volt/mcfg_sw.mbn", 0x5f6c, 436, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ctc/mcfg_sw.mbn", 0xc1cc, 437, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/cz/mcfg_sw.mbn", 0xcee0, 438, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/egypt/mcfg_sw.mbn", 0xc6e4, 439, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/germany/mcfg_sw.mbn", 0xce40, 440, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/gr/mcfg_sw.mbn", 0xc720, 441, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/italy/mcfg_sw.mbn", 0xab68, 442, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/netherla/mcfg_sw.mbn", 0xcca4, 443, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/portugal/mcfg_sw.mbn", 0xab60, 444, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/ro/mcfg_sw.mbn", 0xc118, 445, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/safrica/mcfg_sw.mbn", 0xc360, 446, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/spain/mcfg_sw.mbn", 0xcc98, 447, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/turkey/mcfg_sw.mbn", 0xc384, 448, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/vodafone/volte/uk/mcfg_sw.mbn", 0xcec8, 449, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/we/commerci/egypt/mcfg_sw.mbn", 0xbdbc, 450, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/wind/commerci/mcfg_sw.mbn", 0xc5e0, 451, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/yoigo/commerci/mcfg_sw.mbn", 0xb708, 452, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/eu/zeop/commerci/mcfg_sw.mbn", 0xc064, 453, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/kt/commerci/mcfg_sw.mbn", 0xb368, 454, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/lgu/commerci/mcfg_sw.mbn", 0xaee4, 455, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/skt/commerci/mcfg_sw.mbn", 0xa210, 456, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/korea/tta/commerci/mcfg_sw.mbn", 0x9bf4, 457, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/commerci/mx/mcfg_sw.mbn", 0xc238, 458, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/non_volt/mcfg_sw.mbn", 0x393c, 459, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/amx/volte/mcfg_sw.mbn", 0xc638, 460, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/antel/commerci/mcfg_sw.mbn", 0xbd84, 461, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/brisanet/commerci/brazil/mcfg_sw.mbn", 0xbcb4, 462, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/arg/mcfg_sw.mbn", 0xd9f0, 463, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/brazil/mcfg_sw.mbn", 0xda10, 464, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/chile/mcfg_sw.mbn", 0xe430, 465, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/colombia/mcfg_sw.mbn", 0xebf0, 466, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/cri/mcfg_sw.mbn", 0xda28, 467, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/dominica/mcfg_sw.mbn", 0xda40, 468, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/ecuador/mcfg_sw.mbn", 0xdd08, 469, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/guatemal/mcfg_sw.mbn", 0xda34, 470, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/honduras/mcfg_sw.mbn", 0xda2c, 471, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/nicaragu/mcfg_sw.mbn", 0xda30, 472, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/panama/mcfg_sw.mbn", 0xd9d4, 473, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/paraguay/mcfg_sw.mbn", 0xd8b8, 474, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/peru/mcfg_sw.mbn", 0xeba8, 475, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/puerto/mcfg_sw.mbn", 0xd9cc, 476, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/salvador/mcfg_sw.mbn", 0xda2c, 477, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/claro/commerci/uruguay/mcfg_sw.mbn", 0xd980, 478, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/chile/mcfg_sw.mbn", 0xfb3c, 479, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/entel/commerci/peru/mcfg_sw.mbn", 0xefa0, 480, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/liberty/commerci/pa/mcfg_sw.mbn", 0xbc00, 481, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/argentin/mcfg_sw.mbn", 0xb4b8, 482, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/chile_co/mcfg_sw.mbn", 0xb7fc, 483, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/ecuador_/mcfg_sw.mbn", 0xb578, 484, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/mx_comme/mcfg_sw.mbn", 0xb5fc, 485, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/panama_c/mcfg_sw.mbn", 0xbd6c, 486, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/movistar/uruguay_/mcfg_sw.mbn", 0xb508, 487, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/mx-att/commerci/mcfg_sw.mbn", 0xe110, 488, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/colombia/mcfg_sw.mbn", 0xbd1c, 489, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/telefoni/commerci/peru/mcfg_sw.mbn", 0xe0c8, 490, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/guatemal/mcfg_sw.mbn", 0xbb24, 491, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tigo/commerci/mcfg_sw.mbn", 0xba68, 492, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/tim/commerci/brazil/mcfg_sw.mbn", 0xa2c0, 493, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/unifique/commerci/brazil/mcfg_sw.mbn", 0xbf00, 494, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/vivo/commerci/brazil/mcfg_sw.mbn", 0xc10c, 495, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/colombia/mcfg_sw.mbn", 0xc114, 496, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/latam/wom/commerci/mcfg_sw.mbn", 0xb014, 497, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/azerbaij/commerci/mcfg_sw.mbn", 0xc7d0, 498, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/bahrain_/mcfg_sw.mbn", 0xb7f0, 499, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/betelco_/mcfg_sw.mbn", 0xb7f0, 500, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/viva/commerci/mcfg_sw.mbn", 0xb79c, 501, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/bahrain/zain/commerci/mcfg_sw.mbn", 0xb79c, 502, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/cyta/commerci/mcfg_sw.mbn", 0xbab4, 503, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/epic/commerci/mcfg_sw.mbn", 0xb8a0, 504, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/orange/commerci/mcfg_sw.mbn", 0xb9f0, 505, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/jordan/umniah/commerci/mcfg_sw.mbn", 0xb958, 506, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/ooredoo/commerci/mcfg_sw.mbn", 0xbb10, 507, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/viva/commerci/mcfg_sw.mbn", 0xbd94, 508, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/kuwait/zain/commerci/mcfg_sw.mbn", 0xbd28, 509, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/omantel/commerci/mcfg_sw.mbn", 0xb798, 510, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/ooredoo/commerci/mcfg_sw.mbn", 0xb8dc, 511, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/oman/vodafone/commerci/mcfg_sw.mbn", 0xc2fc, 512, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/primetel/commerci/mcfg_sw.mbn", 0xb8b0, 513, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/qatar/commerci/mcfg_sw.mbn", 0xb420, 514, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/mobily/commerci/mcfg_sw.mbn", 0xbc18, 515, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/salam/commerci/mcfg_sw.mbn", 0xb61c, 516, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/saudi/zain/commerci/mcfg_sw.mbn", 0xb7e4, 517, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mea/stc/commerci/sa/mcfg_sw.mbn", 0xb8d4, 518, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/altan/commerci/mcfg_sw.mbn", 0xb948, 519, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/mexico/axtel/commerci/mcfg_sw.mbn", 0xb928, 520, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/american/mcfg_sw.mbn", 0xd5fc, 521, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/firstnet/mcfg_sw.mbn", 0xd4a0, 522, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/guam_cnm/mcfg_sw.mbn", 0xd5e4, 523, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/non_volt/mcfg_sw.mbn", 0x61b8, 524, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/att/volte/mcfg_sw.mbn", 0xd490, 525, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/bell/commerci/ca/mcfg_sw.mbn", 0xc9c8, 526, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/cbrs/commerci/us/mcfg_sw.mbn", 0x5454, 527, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/dish/commerci/us/mcfg_sw.mbn", 0xbb7c, 528, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/rogers/commerci/ca/mcfg_sw.mbn", 0xbff0, 529, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/commerci/mcfg_sw.mbn", 0xf0dc, 530, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/sprint/vowifi/mcfg_sw.mbn", 0xf0d8, 531, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/telus/commerci/ca/mcfg_sw.mbn", 0xc434, 532, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/commerci/mcfg_sw.mbn", 0xd210, 533, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/tmo/poland_c/mcfg_sw.mbn", 0xba04, 534, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/uscc/commerci/mcfg_sw.mbn", 0xc850, 535, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/cdmaless/mcfg_sw.mbn", 0x116e8, 536, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/hvolte/mcfg_sw.mbn", 0x132dc, 537, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/na/verizon/imsless/mcfg_sw.mbn", 0x93f0, 538, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/beeline/gen_3gpp/mcfg_sw.mbn", 0xcec8, 539, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/megafon/commerci/ru/mcfg_sw.mbn", 0xc97c, 540, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/motiv/commerci/mcfg_sw.mbn", 0xb79c, 541, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/mcfg_sw.mbn", 0xb8d8, 542, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/mts/commerci/ru/mcfg_sw.mbn", 0xb964, 543, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/sberbank/commerci/mcfg_sw.mbn", 0xbdc8, 544, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/russia/tele2/commerci/mcfg_sw.mbn", 0xc778, 545, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sa/brazil/commerci/mcfg_sw.mbn", 0x32cc, 546, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/3hk/commerci/hk/mcfg_sw.mbn", 0xbed0, 547, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/airtel/commerci/lanka/mcfg_sw.mbn", 0xb904, 548, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ais/commerci/thailand/mcfg_sw.mbn", 0xb850, 549, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/apt/commerci/mcfg_sw.mbn", 0xbaa4, 550, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/celcom/commerci/mcfg_sw.mbn", 0xbdec, 551, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/chunghwa/commerci/tw/mcfg_sw.mbn", 0xc99c, 552, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/cuhk/commerci/mcfg_sw.mbn", 0xb3dc, 553, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/digi/commerci/mcfg_sw.mbn", 0xca88, 554, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dito/commerci/ph/mcfg_sw.mbn", 0xc0b0, 555, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/th/mcfg_sw.mbn", 0xbdd0, 556, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/dtac/commerci/volte/mcfg_sw.mbn", 0xbdd0, 557, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/fareasto/commerci/mcfg_sw.mbn", 0xbee4, 558, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/globe/commerci/ph/mcfg_sw.mbn", 0xbdc0, 559, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/hkt/commerci/hk/mcfg_sw.mbn", 0xb8f0, 560, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/indosat/commerci/mcfg_sw.mbn", 0xba58, 561, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/m1/commerci/sg/mcfg_sw.mbn", 0xc208, 562, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/maxis/commerci/malaysia/mcfg_sw.mbn", 0xb250, 563, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/metfone/commerci/mcfg_sw.mbn", 0xbba4, 564, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mobifone/commerci/mcfg_sw.mbn", 0xc720, 565, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mpt/commerci/myanmar/mcfg_sw.mbn", 0xb428, 566, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/mytel/commerci/myanmar/mcfg_sw.mbn", 0xbbb8, 567, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/nt/commerci/thailand/mcfg_sw.mbn", 0xc364, 568, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/p1/commerci/malaysia/mcfg_sw.mbn", 0xbb8c, 569, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/seatel/commerci/mcfg_sw.mbn", 0xd70c, 570, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/singtel/commerci/singapor/mcfg_sw.mbn", 0xc6e4, 571, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart/commerci/mcfg_sw.mbn", 0xb950, 572, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smart_ph/commerci/mcfg_sw.mbn", 0xb938, 573, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/mcfg_sw.mbn", 0xc2a4, 574, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartfre/commerci/vowifi/mcfg_sw.mbn", 0xc254, 575, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/smartone/commerci/hk/mcfg_sw.mbn", 0xb9a4, 576, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/starhub/commerci/sg/mcfg_sw.mbn", 0xb9c8, 577, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/telkomse/commerci/mcfg_sw.mbn", 0xc388, 578, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tm/commerci/mcfg_sw.mbn", 0xbd54, 579, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tpg/commerci/tpg/mcfg_sw.mbn", 0xbcb8, 580, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tri/commerci/mcfg_sw.mbn", 0xbb04, 581, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/thailand/mcfg_sw.mbn", 0xc010, 582, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/truemove/commerci/volte/mcfg_sw.mbn", 0xc030, 583, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/tstar/commerci/tw/mcfg_sw.mbn", 0xa408, 584, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/umobile/commerci/malaysia/mcfg_sw.mbn", 0xc5f0, 585, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vietnamo/commerci/vietnamo/mcfg_sw.mbn", 0xb8f0, 586, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/viettel/commerci/vietnam/mcfg_sw.mbn", 0xbb30, 587, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/vinaphon/commerci/vietnam/mcfg_sw.mbn", 0xbdbc, 588, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/xl/commerci/mcfg_sw.mbn", 0xb990, 589, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/sea/ytl/commerci/mcfg_sw.mbn", 0xbb84, 590, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/er_iot/mcfg_sw.mbn", 0xb7b0, 591, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/generic/test/nk_iot/mcfg_sw.mbn", 0xb744, 592, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.dig", 0x20, 593, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/mbn_sw.txt", 0x4a2c, 594, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_eea.txt", 0x378d, 595, 1 },
	{ "image/modem_pr/mcfg/configs/mcfg_sw/oem_sw.txt", 0x3ece, 596, 1 },
	{ "image/modemr.jsn", 0x2cf, 597, 1 },
	{ "image/modemuw.jsn", 0x2c9, 598, 1 },
	{ "image/otrpapp.b00", 0x134, 599, 1 },
	{ "image/otrpapp.b01", 0x1b28, 600, 1 },
	{ "image/otrpapp.b02", 0x4c0c0, 601, 1 },
	{ "image/otrpapp.b03", 0x8f, 602, 1 },
	{ "image/otrpapp.b04", 0x3b80, 603, 1 },
	{ "image/otrpapp.b05", 0x238, 604, 1 },
	{ "image/otrpapp.b06", 0x98, 605, 1 },
	{ "image/otrpapp.b07", 0xb3c3, 606, 1 },
	{ "image/otrpapp.mdt", 0x1c5c, 607, 1 },
	{ "image/qdsp6m.qdb", 0x3df2b6, 608, 1 },
	{ "image/rtic.mbn", 0x493ba, 609, 1 },
	{ "image/secauth.b00", 0x134, 610, 1 },
	{ "image/secauth.b01", 0x1b28, 611, 1 },
	{ "image/secauth.b02", 0x55b1, 612, 1 },
	{ "image/secauth.b03", 0x98, 613, 1 },
	{ "image/secauth.b04", 0x460, 614, 1 },
	{ "image/secauth.b05", 0xdc, 615, 1 },
	{ "image/secauth.b06", 0x98, 616, 1 },
	{ "image/secauth.b07", 0x102c, 617, 1 },
	{ "image/secauth.mdt", 0x1c5c, 618, 1 },
	{ "image/smplap32.b00", 0x134, 619, 1 },
	{ "image/smplap32.b01", 0x1b28, 620, 1 },
	{ "image/smplap32.b02", 0x43c7c, 621, 1 },
	{ "image/smplap32.b03", 0xcc, 622, 1 },
	{ "image/smplap32.b04", 0x1688bc, 623, 1 },
	{ "image/smplap32.b05", 0x4c0, 624, 1 },
	{ "image/smplap32.b06", 0x98, 625, 1 },
	{ "image/smplap32.b07", 0xb1a5, 626, 1 },
	{ "image/smplap32.mdt", 0x1c5c, 627, 1 },
	{ "image/smplap64.b00", 0x200, 628, 1 },
	{ "image/smplap64.b01", 0x1b28, 629, 1 },
	{ "image/smplap64.b02", 0x4ff70, 630, 1 },
	{ "image/smplap64.b03", 0xf8, 631, 1 },
	{ "image/smplap64.b04", 0x16b250, 632, 1 },
	{ "image/smplap64.b05", 0x980, 633, 1 },
	{ "image/smplap64.b06", 0x130, 634, 1 },
	{ "image/smplap64.b07", 0x1452b, 635, 1 },
	{ "image/smplap64.mdt", 0x1d28, 636, 1 },
	{ "image/soter64.b00", 0x200, 637, 1 },
	{ "image/soter64.b01", 0x1b28, 638, 1 },
	{ "image/soter64.b02", 0xcecd, 639, 1 },
	{ "image/soter64.b03", 0xc2, 640, 1 },
	{ "image/soter64.b04", 0x840, 641, 1 },
	{ "image/soter64.b05", 0x288, 642, 1 },
	{ "image/soter64.b06", 0x130, 643, 1 },
	{ "image/soter64.b07", 0x1b48, 644, 1 },
	{ "image/soter64.mdt", 0x1d28, 645, 1 },
	{ "image/vsimapp.b00", 0x134, 646, 1 },
	{ "image/vsimapp.b01", 0x1b28, 647, 1 },
	{ "image/vsimapp.b02", 0x7d3b, 648, 1 },
	{ "image/vsimapp.b03", 0xbb, 649, 1 },
	{ "image/vsimapp.b04", 0x334, 650, 1 },
	{ "image/vsimapp.b05", 0x10c, 651, 1 },
	{ "image/vsimapp.b06", 0x98, 652, 1 },
	{ "image/vsimapp.b07", 0x1510, 653, 1 },
	{ "image/vsimapp.mdt", 0x1c5c, 654, 1 },
	{ "image/widevine.mbn", 0x2ecfa, 655, 1 },
	{ "image/wlanmdsp.mbn", 0x3a39c0, 656, 1 },
};

static int
fw_all_find(const char *path)
{
	for (unsigned i = 0; i < sizeof(fw_all) / sizeof(fw_all[0]); i++) {
		if (strcmp(fw_all[i].path, path) == 0) {
			return (int)i;
		}
	}
	return -1;
}

/* read n bytes at off of file i from modem_a (extent by extent) */
static bool
fw_all_read(int i, uint64_t off, void *dst, uint64_t n)
{
	uint64_t base = 0, done = 0;
	for (unsigned k = 0; k < fw_all[i].next && done < n; k++) {
		uint32_t eo = fw_ext[fw_all[i].ext + k].off, el = fw_ext[fw_all[i].ext + k].len;
		if (off + done < base + el) {
			uint64_t in = off + done - base, take = MIN((uint64_t)el - in, n - done);
			if (sweet_ufs_read_part(fw_part, eo + in, (uint8_t *)dst + done, take) != 0) {
				return false;
			}
			done += take;
		}
		base += el;
	}
	return done == n;
}

/* QMI */
struct qmsg { uint8_t b[512]; uint32_t n; };

static void
qm_init(struct qmsg *m, uint16_t txn, uint16_t msg)
{
	m->b[0] = QMI_RESP;
	memcpy(m->b + 1, &txn, 2);
	memcpy(m->b + 3, &msg, 2);
	m->n = 7;
}

static void
qm_tlv(struct qmsg *m, uint8_t type, const void *v, uint16_t len)
{
	if (m->n + 3 + len > sizeof(m->b)) {
		return;
	}
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &len, 2);
	memcpy(m->b + m->n + 3, v, len);
	m->n += 3 + len;
}

static void
qm_result(struct qmsg *m, bool ok)
{
	uint16_t r[2] = { (uint16_t)(ok ? 0 : 1), (uint16_t)(ok ? 0 : 1) };
	qm_tlv(m, 2, r, 4);
}

static void
qm_send(struct qmsg *m, uint32_t src_port, uint32_t node, uint32_t port)
{
	uint16_t l = (uint16_t)(m->n - 7);
	memcpy(m->b + 5, &l, 2);
	qrtr_send(QRTR_TYPE_DATA, src_port, node, port, m->b, m->n);
}

static const uint8_t *
qm_find(const uint8_t *d, uint32_t len, uint8_t type, uint16_t *tlen)
{
	uint32_t o = 7;
	while (o + 3 <= len) {
		uint16_t l;
		memcpy(&l, d + o + 1, 2);
		if (o + 3 + l > len) {
			break;
		}
		if (d[o] == type) {
			*tlen = l;
			return d + o + 3;
		}
		o += 3 + l;
	}
	return NULL;
}

static void
qm_string(const uint8_t *d, uint32_t len, uint8_t type, char *out, size_t max)
{
	uint16_t l = 0;
	const uint8_t *v = qm_find(d, len, type, &l);
	out[0] = 0;
	if (v != NULL) {
		l = (uint16_t)MIN((size_t)l, max - 1);
		memcpy(out, v, l);
		out[l] = 0;
	}
}

/* pd-mapper */
static const struct { const char *svc, *domain; } pd_map[] = {
	{ "tms/servreg",       "msm/modem/root_pd" },
	{ "tms/pdr_enabled",   "msm/modem/root_pd" },
	{ "gps/gps_service",   "msm/modem/root_pd" },
	{ "kernel/elf_loader", "msm/modem/wlan_pd" },
	{ "tms/servreg",       "msm/modem/wlan_pd" },
	{ "wlan/fw",           "msm/modem/wlan_pd" },
};
static unsigned pd_reqs;

static void
pdm_rx(uint32_t node, uint32_t port, uint16_t txn, uint16_t msg, const uint8_t *d, uint32_t len)
{
	struct qmsg m;
	char name[80];
	uint8_t list[256];
	uint32_t n = 1, cnt = 0;
	uint16_t v;

	qm_init(&m, txn, msg);
	if (msg != 0x21) {                      /* PFR etc.: just say OK */
		LOG("t=%u ms: pd-mapper: msg %x", g_ms, msg);
		qm_result(&m, true);
		qm_send(&m, PORT_PDM, node, port);
		return;
	}
	qm_string(d, len, 1, name, sizeof(name));
	for (unsigned i = 0; i < sizeof(pd_map) / sizeof(pd_map[0]); i++) {
		if (strcmp(pd_map[i].svc, name) == 0) {
			uint32_t dl = (uint32_t)strlen(pd_map[i].domain), inst = 180, zero = 0;
			list[n++] = (uint8_t)dl;
			memcpy(list + n, pd_map[i].domain, dl);
			n += dl;
			memcpy(list + n, &inst, 4);
			n += 4;
			list[n++] = 0;                  /* service_data_valid */
			memcpy(list + n, &zero, 4);
			n += 4;
			cnt++;
		}
	}
	list[0] = (uint8_t)cnt;
	qm_result(&m, true);
	v = (uint16_t)cnt;
	qm_tlv(&m, 0x10, &v, 2);
	v = 1;
	qm_tlv(&m, 0x11, &v, 2);
	if (cnt != 0) {
		qm_tlv(&m, 0x12, list, (uint16_t)n);
	}
	qm_send(&m, PORT_PDM, node, port);
	pd_reqs++;
	LOG("t=%u ms: pd-mapper: \"%s\" -> %u domain(s)", g_ms, name, cnt);
}

/* tftp */
#define OP_RRQ   1
#define OP_WRQ   2
#define OP_DATA  3
#define OP_ACK   4
#define OP_ERROR 5
#define OP_OACK  6

struct tftp_sess {
	bool used, write;
	uint32_t port, node, rport;
	int fi;                                 /* fw_all index, -1 = none */
	int rw;                                 /* /readwrite file in RAM, -1 = none */
	uint64_t size, sent;
	uint32_t blk, rsize, wsize, seek;
};
static struct tftp_sess sess[16];
static uint32_t next_sess_port = PORT_SESS;
static unsigned tftp_reqs, tftp_missing, tftp_writes;

struct rw_file { char path[96]; uint8_t *data; uint32_t size, cap; };
static struct rw_file rw_files[12];

static int
rw_find(const char *path, bool create)
{
	for (int i = 0; i < 12; i++) {
		if (rw_files[i].path[0] != 0 && strcmp(rw_files[i].path, path) == 0) {
			return i;
		}
	}
	for (int i = 0; create && i < 12; i++) {
		if (rw_files[i].path[0] == 0) {
			strlcpy(rw_files[i].path, path, sizeof(rw_files[i].path));
			return i;
		}
	}
	return -1;
}

static bool
rw_write(int r, uint64_t off, const uint8_t *src, uint32_t n)
{
	struct rw_file *f = &rw_files[r];
	uint64_t end = off + n;
	if (off > 0x100000 || n > 0x100000) {
		return false;
	}
	if (end > f->cap) {
		uint32_t cap = (uint32_t)MAX(end, (uint64_t)f->cap * 2);
		uint8_t *d = (uint8_t *)IOMalloc(cap);
		if (d == NULL) {
			return false;
		}
		bzero(d, cap);
		if (f->data != NULL) {
			memcpy(d, f->data, f->size);
			IOFree(f->data, f->cap);
		}
		f->data = d;
		f->cap = cap;
	}
	memcpy(f->data + off, src, n);
	f->size = (uint32_t)MAX((uint64_t)f->size, end);
	return true;
}

static void
tftp_send(struct tftp_sess *s, const void *p, uint32_t len)
{
	qrtr_send(QRTR_TYPE_DATA, s->port, s->node, s->rport, p, len);
}

static void
tftp_error(struct tftp_sess *s, uint16_t code, const char *msg)
{
	uint8_t b[64] = { 0, OP_ERROR, (uint8_t)(code >> 8), (uint8_t)code };
	uint32_t l = (uint32_t)strlcpy((char *)b + 4, msg, sizeof(b) - 4) + 1;
	tftp_send(s, b, 4 + l);
}

static void
tftp_close(struct tftp_sess *s)
{
	bzero(s, sizeof(*s));
}

/* tqftpserv tftp_send_data() */
static bool
tftp_data(struct tftp_sess *s, uint32_t block, uint64_t offset, uint32_t resp_size)
{
	uint8_t *b = (uint8_t *)IOMalloc(s->blk + 4);
	uint32_t n = s->blk, send;

	if (b == NULL) {
		return false;
	}
	b[0] = 0; b[1] = OP_DATA; b[2] = (uint8_t)(block >> 8); b[3] = (uint8_t)block;
	if (offset >= s->size) {
		n = 0;
	} else {
		n = (uint32_t)MIN((uint64_t)n, s->size - offset);
		if (s->rw >= 0) {
			memcpy(b + 4, rw_files[s->rw].data + offset, n);
		} else if (!fw_all_read(s->fi, offset, b + 4, n)) {
			LOG("tftp sess %x: read FAILED at %llu", s->port, offset);
			IOFree(b, s->blk + 4);
			tftp_error(s, 0, "Read error");
			return false;
		}
	}
	send = 4 + n;
	if (resp_size != 0) {
		if (4 + resp_size > send) {
			IOFree(b, s->blk + 4);
			return false;
		}
		send = 4 + resp_size;
	}
	tftp_send(s, b, send);
	s->sent += send - 4;
	IOFree(b, s->blk + 4);
	return true;
}

static char *
tftp_opt(char *p, char *end, const char *name, uint64_t val)
{
	char num[24];
	snprintf(num, sizeof(num), "%llu", val);
	size_t a = strlen(name) + 1, b = strlen(num) + 1;
	if (p + a + b > end) {
		return p;
	}
	memcpy(p, name, a);
	memcpy(p + a, num, b);
	return p + a + b;
}

/* the modem's path -> fw_all path: /readonly/firmware/image/x -> image/x */
static bool
tftp_map(const char *path, char *out, size_t max)
{
	static const char *pre[] = {
		"/readonly/firmware/image/", "/readonly/vendor/firmware_mnt/image/", "/readonly/vendor/firmware/",
	};
	const char *rest = NULL;
	size_t k;

	for (unsigned i = 0; i < 3; i++) {
		if (strncmp(path, pre[i], strlen(pre[i])) == 0) {
			rest = path + strlen(pre[i]);
			strlcpy(out, "image/", max);
			break;
		}
	}
	if (rest == NULL && strncmp(path, "/readonly/firmware/", 19) == 0) {
		rest = path + 19;
		out[0] = 0;
	}
	if (rest == NULL) {
		return false;
	}
	k = strlen(out);
	for (; *rest != 0 && k + 1 < max; rest++) {
		char c = *rest;
		out[k++] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
	}
	out[k] = 0;
	return true;
}

static uint32_t
dec(const char *p)
{
	uint32_t v = 0;
	while (*p >= '0' && *p <= '9') {
		v = v * 10 + (uint32_t)(*p++ - '0');
	}
	return v;
}

static void
tftp_request(uint32_t node, uint32_t port, const uint8_t *d, uint32_t len)
{
	char buf[512], *p, *end, *file, *mode, oack[256], *o, path[128];
	uint16_t op = (uint16_t)(d[0] << 8 | d[1]);
	struct tftp_sess *s = NULL;
	bool opts, want_tsize = false;
	uint32_t timeoutms = 1000;

	if (len < 4 || len >= sizeof(buf)) {
		return;
	}
	memcpy(buf, d, len);
	buf[len] = 0;
	end = buf + len;
	file = buf + 2;
	mode = file + strlen(file) + 1;
	if (mode >= end) {
		return;
	}
	p = mode + strlen(mode) + 1;
	opts = p < end;
	for (unsigned i = 0; i < 16; i++) {
		if (!sess[i].used) {
			s = &sess[i];
			break;
		}
	}
	if (s == NULL) {
		LOG("tftp: out of sessions");
		return;
	}
	bzero(s, sizeof(*s));
	s->used = true;
	s->write = op == OP_WRQ;
	s->port = next_sess_port++;
	s->node = node;
	s->rport = port;
	s->blk = 512;
	s->wsize = 1;
	s->rw = -1;
	s->fi = -1;
	while (p < end) {
		char *name = p, *val = p + strlen(p) + 1;
		if (val >= end) {
			break;
		}
		uint32_t v = dec(val);
		if (strcmp(name, "blksize") == 0) {
			s->blk = MAX(8u, MIN(v, 16384u));
		} else if (strcmp(name, "timeoutms") == 0) {
			timeoutms = v;
		} else if (strcmp(name, "tsize") == 0) {
			want_tsize = true;
		} else if (strcmp(name, "rsize") == 0) {
			s->rsize = v;
		} else if (strcmp(name, "wsize") == 0) {
			s->wsize = MAX(1u, v);
		} else if (strcmp(name, "seek") == 0) {
			s->seek = v;
		}
		p = val + strlen(val) + 1;
	}
	tftp_reqs++;
	if (strncmp(file, "/readwrite/", 11) == 0) {
		s->rw = rw_find(file, s->write);
	}
	if (s->write) {
		tftp_writes++;
		LOG("t=%u ms: tftp WRQ %s seek %u%s", g_ms, file, s->seek, s->rw < 0 ? " (no RAM slot, dropped)" : "");
		if (s->rw >= 0 && s->seek == 0) {
			rw_files[s->rw].size = 0;       /* a write from the start replaces the file */
		}
	} else if (s->rw >= 0) {
		s->size = rw_files[s->rw].size;
		LOG("t=%u ms: tftp RRQ %s (RAM, %llu B)", g_ms, file, s->size);
	} else {
		if (!tftp_map(file, path, sizeof(path)) || (s->fi = fw_all_find(path)) < 0) {
			tftp_missing++;
			LOG("t=%u ms: tftp RRQ %s: not found", g_ms, file);
			tftp_error(s, 1, "file not found");
			tftp_close(s);
			return;
		}
		s->size = fw_all[s->fi].size;
		LOG("t=%u ms: tftp RRQ %s seek %u rsize %u blk %u w %u (%llu B)", g_ms, path, s->seek, s->rsize, s->blk, s->wsize, s->size);
	}
	if (!opts) {
		if (s->write) {
			uint8_t ack[4] = { 0, OP_ACK, 0, 0 };
			tftp_send(s, ack, 4);
		} else {
			tftp_data(s, 1, 0, 0);
		}
		return;
	}
	o = oack;
	*o++ = 0;
	*o++ = OP_OACK;
	o = tftp_opt(o, oack + sizeof(oack), "blksize", s->blk);
	o = tftp_opt(o, oack + sizeof(oack), "timeoutms", timeoutms);
	if (want_tsize && !s->write) {
		o = tftp_opt(o, oack + sizeof(oack), "tsize", s->size);
	}
	o = tftp_opt(o, oack + sizeof(oack), "wsize", s->wsize);
	if (s->rsize != 0) {
		o = tftp_opt(o, oack + sizeof(oack), "rsize", s->rsize);
	}
	if (s->seek != 0) {
		o = tftp_opt(o, oack + sizeof(oack), "seek", s->seek);
	}
	tftp_send(s, oack, (uint32_t)(o - oack));
}

/* traffic on a session port: ACKs for reads, DATA for writes (tqftpserv handle_reader) */
static bool
tftp_session(uint32_t dst_port, const uint8_t *d, uint32_t len)
{
	struct tftp_sess *s = NULL;
	for (unsigned i = 0; i < 16; i++) {
		if (sess[i].used && sess[i].port == dst_port) {
			s = &sess[i];
			break;
		}
	}
	if (s == NULL || len < 4) {
		return s != NULL;
	}
	uint16_t op = (uint16_t)(d[0] << 8 | d[1]), blk = (uint16_t)(d[2] << 8 | d[3]);
	if (op == OP_ERROR) {
		/* rsize 0 + tsize is a stat: the modem ends it with ERROR 9 "End of Transfer" */
		if (blk != 9) {
			LOG("t=%u ms: tftp sess %x: modem ERROR %u after %llu B", g_ms, s->port, blk, s->sent);
		}
		tftp_close(s);
		return true;
	}
	if (s->write) {
		if (op == OP_DATA) {
			uint32_t payload = len - 4;
			if (s->rw >= 0 && blk >= 1) {
				(void)rw_write(s->rw, s->seek + (uint64_t)(blk - 1) * s->blk, d + 4, payload);
			}
			if ((blk % s->wsize) == 0 || payload < s->blk) {
				uint8_t ack[4] = { 0, OP_ACK, (uint8_t)(blk >> 8), (uint8_t)blk };
				tftp_send(s, ack, 4);
			}
			if (payload < s->blk) {
				tftp_close(s);
			}
		}
		return true;
	}
	if (op != OP_ACK) {
		tftp_error(s, 4, "Expected ACK opcode");
		tftp_close(s);
		return true;
	}
	if (s->rsize == 0) {
		/* the whole file: blocks until a short one */
		uint64_t total = s->size > s->seek ? s->size - s->seek : 0;
		uint32_t last = (uint32_t)(total / s->blk) + 1;
		if ((uint32_t)blk >= last) {
			tftp_close(s);
			return true;
		}
		for (uint32_t i = blk + 1u; i <= blk + s->wsize && i <= last; i++) {
			if (!tftp_data(s, i, s->seek + (uint64_t)(i - 1) * s->blk, 0)) {
				break;
			}
		}
		return true;
	}
	if ((uint64_t)blk * s->blk > s->rsize) {
		tftp_close(s);
		return true;
	}
	for (uint32_t i = blk; i < blk + s->wsize; i++) {
		uint64_t off = s->seek + (uint64_t)i * s->blk;
		uint32_t rs = (i + 1) * s->blk > s->rsize ? s->rsize % s->blk : 0;
		if (!tftp_data(s, i + 1, off, rs) || (i + 1) * s->blk > s->rsize) {
			break;
		}
	}
	return true;
}

/* rmtfs */
#define RMTFS_BUF_SIZE      0x300000            /* qcom,rmtfs_sharedmem size */
#define VMID_HLOS           3
#define VMID_MSS_MSA        15
#define VMID_NAV            0x2B
#define PERM_RW             6

static const struct { const char *path, *part; } rmtfs_parts[] = {
	{ "/boot/modem_fs1", "modemst1" },
	{ "/boot/modem_fs2", "modemst2" },
	{ "/boot/modem_fsc", "fsc" },
	{ "/boot/modem_fsg", "fsg" },
};
static uint64_t rmtfs_pa;
static uint8_t *rmtfs_va;
static unsigned rmtfs_reads, rmtfs_writes, rmtfs_err;

/* hyp_assign_phys (SCM 0x0C/0x16): from HLOS to the given VMIDs, read-write */
static uint64_t
scm_assign(uint64_t addr, uint64_t size, const uint32_t *vmids, unsigned count)
{
	uint64_t pa = 0, res = 0;
	uint64_t *p = (uint64_t *)pool_alloc(0x1000, &pa);

	if (p == NULL) {
		return ~0ull;
	}
	p[0] = addr;                                /* 0x000: mem map {addr, size} */
	p[1] = size;
	uint32_t *src = (uint32_t *)(p + 8);        /* 0x040: src vmids */
	src[0] = VMID_HLOS;
	uint32_t *dst = (uint32_t *)(p + 16);       /* 0x080: {vmid, perm, ctx(8), ctx_size, unused} x3 */
	for (unsigned i = 0; i < count && i < 8; i++) {
		dst[6 * i] = vmids[i];
		dst[6 * i + 1] = PERM_RW;
	}
	p[32] = 4;                                  /* 0x100: extended args: src size */
	p[33] = pa + 0x80;                          /* dest */
	p[34] = count * 24;                         /* dest size */
	p[35] = 0;
	mb();
	uint64_t args[8] = { SCM_FN(0x0C, 0x16), 7 | (1 << 4) | (1 << 8) | (1 << 12), pa, 16, pa + 0x40, pa + 0x100, 0, 0 };
	smc(args);
	for (unsigned n = 0; args[0] == 1 && n < 100000; n++) {
		uint64_t again[8] = { 1, 7 | (1 << 4) | (1 << 8) | (1 << 12), pa, 16, pa + 0x40, pa + 0x100, args[6], 0 };
		memcpy(args, again, sizeof(args));
		smc(args);
	}
	res = args[1];
	LOG("SCM assign %llx+%llx -> %u vmids: ret=%llx res=%llx", addr, size, count, args[0], res);
	return args[0] != 0 ? args[0] : res;
}

static void
rmtfs_rx(uint32_t node, uint32_t port, uint16_t txn, uint16_t msg, const uint8_t *d, uint32_t len)
{
	struct qmsg m;
	char path[64];
	uint16_t l = 0;
	const uint8_t *v;
	uint32_t caller = 0, i;
	bool ok = true;

	qm_init(&m, txn, msg);
	v = qm_find(d, len, 1, &l);
	if (msg != 1 && v != NULL && l >= 4) {
		memcpy(&caller, v, 4);
	}
	switch (msg) {
	case 1:                                     /* OPEN */
		qm_string(d, len, 1, path, sizeof(path));
		for (i = 0; i < 4; i++) {
			if (strcmp(path, rmtfs_parts[i].path) == 0) {
				break;
			}
		}
		if (i < 4) {
			uint8_t probe[16];
			ok = sweet_ufs_read_part(rmtfs_parts[i].part, 0, probe, sizeof(probe)) == 0;
		} else {
			ok = false;
		}
		LOG("t=%u ms: rmtfs: open %s -> %s", g_ms, path, ok ? rmtfs_parts[i].part : "FAILED");
		qm_result(&m, ok);
		if (ok) {
			caller = i + 1;
			qm_tlv(&m, 0x10, &caller, 4);
		}
		break;
	case 2:                                     /* CLOSE */
		qm_result(&m, true);
		break;
	case 3: {                                   /* RW_IOVEC */
		uint16_t l2 = 0;
		const uint8_t *dir = qm_find(d, len, 2, &l2);
		const uint8_t *iov = qm_find(d, len, 3, &l2);
		bool write = dir != NULL && *dir != 0;
		ok = caller >= 1 && caller <= 4 && iov != NULL && rmtfs_pa != 0;
		for (i = 0; ok && i < iov[0]; i++) {
			uint32_t e[3];                      /* sector, phys, count */
			memcpy(e, iov + 1 + 12 * i, 12);
			uint64_t bytes = (uint64_t)e[2] * 512;
			if (e[1] < rmtfs_pa || e[1] + bytes > rmtfs_pa + RMTFS_BUF_SIZE) {
				ok = false;
				break;
			}
			if (write) {
				rmtfs_writes++;                 /* never written to the phone's EFS */
			} else {
				ok = sweet_ufs_read_part(rmtfs_parts[caller - 1].part, (uint64_t)e[0] * 512,
				    rmtfs_va + (e[1] - rmtfs_pa), bytes) == 0;
				rmtfs_reads++;
			}
		}
		if (!ok && ++rmtfs_err <= 3) {
			LOG("t=%u ms: rmtfs: iovec caller %u %s FAILED", g_ms, caller, write ? "write" : "read");
		}
		qm_result(&m, ok);
		break;
	}
	case 4: {                                   /* ALLOC_BUFF */
		if (rmtfs_pa == 0) {
			rmtfs_va = (uint8_t *)pool_alloc(RMTFS_BUF_SIZE, &rmtfs_pa);   /* the modem owns it from now on */
			if (rmtfs_va == NULL) {
				rmtfs_pa = 0;
			} else {
				static const uint32_t v[] = { VMID_HLOS, VMID_MSS_MSA, VMID_NAV };
				(void)scm_assign(rmtfs_pa, RMTFS_BUF_SIZE, v, 3);
			}
		}
		LOG("t=%u ms: rmtfs: alloc for caller %u -> %llx", g_ms, caller, rmtfs_pa);
		ok = rmtfs_pa != 0;
		qm_result(&m, ok);
		if (ok) {
			uint64_t a = rmtfs_pa;
			qm_tlv(&m, 0x10, &a, 8);
		}
		break;
	}
	case 5: {                                   /* GET_DEV_ERROR */
		uint8_t st = 0;
		qm_result(&m, true);
		qm_tlv(&m, 0x10, &st, 1);
		break;
	}
	default:
		LOG("t=%u ms: rmtfs: msg %x", g_ms, msg);
		qm_result(&m, false);
		break;
	}
	qm_send(&m, PORT_RMTFS, node, port);
}

/* ---------------------------------------------------------------- WLFW (QMI 0x45) */

/*
 * The WLAN firmware service, the way ath10k qmi.c talks to it (port of topaz-woa Wlfw.c):
 * ind_register -> host_cap -> msa_info (+ assign the MSA regions) -> msa_ready -> cap ->
 * MSA_READY ind -> board data (bdwlan.*) -> cal_report -> FW_READY ind -> wlan_cfg (CE
 * layout) -> wlan_mode mission. Nothing touches the WLAN registers here (CE comes next and
 * only after mission mode: before it a register read hangs the bus).
 */
#define PORT_WLFWC          0x4004              /* our QMI client port towards WLFW */
#define QMI_IND             4
#define WLFW_IND_REGISTER   0x20
#define WLFW_FW_READY_IND   0x21
#define WLFW_WLAN_MODE      0x22
#define WLFW_WLAN_CFG       0x23
#define WLFW_CAP            0x24
#define WLFW_BDF_DOWNLOAD   0x25
#define WLFW_CAL_REPORT     0x26
#define WLFW_MSA_READY_IND  0x2B
#define WLFW_MSA_INFO       0x2D
#define WLFW_MSA_READY      0x2E
#define WLFW_HOST_CAP       0x34
#define WLFW_MSA_BASE       0x51900000u         /* DTB wlan_msa_region, inside "PIL Reserved" */
#define WLFW_MSA_SIZE       0x00100000u
#define WLFW_CLIENT_ID      0x4b4e454c          /* ath10k ATH10K_QMI_CLIENT_ID */
#define WLFW_SEG            6144                /* QMI_WLFW_MAX_DATA_SIZE_V01 */
#define VMID_WLAN           0x18
#define VMID_WLAN_CE        0x19

static uint32_t w_node, w_port, w_board = 0xFF, w_bdf_seg;
static uint16_t w_txn = 1;
static bool w_msa_ind, w_cap_done, w_bdf_started, w_fw_ready, w_mission;
static int w_bdf_fi = -1;
static uint32_t w_bdf_len, w_bdf_off;
static const char *w_step = "not started";

struct wmsg { uint8_t *b; uint32_t n; };

static bool
wm_new(struct wmsg *m, uint16_t msg)
{
	m->b = (uint8_t *)IOMalloc(WLFW_SEG + 128);
	if (m->b == NULL) {
		return false;
	}
	bzero(m->b, WLFW_SEG + 128);
	m->b[0] = QMI_REQ;
	uint16_t t = w_txn++;
	memcpy(m->b + 1, &t, 2);
	memcpy(m->b + 3, &msg, 2);
	m->n = 7;
	return true;
}

static void
wm_tlv(struct wmsg *m, uint8_t type, const void *v, uint16_t len)
{
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &len, 2);
	if (len != 0) {
		memcpy(m->b + m->n + 3, v, len);
	}
	m->n += 3 + len;
}

static void wm_u8(struct wmsg *m, uint8_t t, uint8_t v) { wm_tlv(m, t, &v, 1); }
static void wm_u32(struct wmsg *m, uint8_t t, uint32_t v) { wm_tlv(m, t, &v, 4); }

/* TLV with a u8 element count in front of an array */
static void
wm_array(struct wmsg *m, uint8_t type, uint8_t count, const void *v, uint16_t bytes)
{
	uint16_t l = (uint16_t)(1 + bytes);
	m->b[m->n] = type;
	memcpy(m->b + m->n + 1, &l, 2);
	m->b[m->n + 3] = count;
	memcpy(m->b + m->n + 4, v, bytes);
	m->n += 4 + bytes;
}

static void
wm_send(struct wmsg *m)
{
	uint16_t l = (uint16_t)(m->n - 7);
	memcpy(m->b + 5, &l, 2);
	qrtr_send(QRTR_TYPE_DATA, PORT_WLFWC, w_node, w_port, m->b, m->n);
	IOFree(m->b, WLFW_SEG + 128);
}

static bool
w_result(const uint8_t *d, uint32_t len, uint16_t *err)
{
	uint16_t l = 0;
	const uint8_t *r = qm_find(d, len, 2, &l);
	uint16_t res = 0xFFFF;
	*err = 0xFFFF;
	if (r != NULL && l >= 4) {
		memcpy(&res, r, 2);
		memcpy(err, r + 2, 2);
	}
	return res == 0;
}

static void
w_simple(uint16_t msg, const char *step)
{
	struct wmsg m;
	if (wm_new(&m, msg)) {
		wm_send(&m);
		w_step = step;
	}
}

static void
w_bdf_segment(void)
{
	struct wmsg m;
	uint32_t chunk = MIN(w_bdf_len - w_bdf_off, (uint32_t)WLFW_SEG);
	bool end = w_bdf_off + chunk >= w_bdf_len;

	if (!wm_new(&m, WLFW_BDF_DOWNLOAD)) {
		return;
	}
	wm_u8(&m, 0x01, 1);                         /* valid */
	wm_u32(&m, 0x10, 0);                        /* file_id */
	wm_u32(&m, 0x11, w_bdf_len);                /* total_size */
	wm_u32(&m, 0x12, w_bdf_seg);                /* seg_id */
	uint8_t *t = m.b + m.n;                     /* data: u16 len + bytes */
	uint16_t tl = (uint16_t)(2 + chunk), dl = (uint16_t)chunk;
	t[0] = 0x13;
	memcpy(t + 1, &tl, 2);
	memcpy(t + 3, &dl, 2);
	if (!fw_all_read(w_bdf_fi, w_bdf_off, t + 5, chunk)) {
		LOG("wlfw: board data read failed");
		IOFree(m.b, WLFW_SEG + 128);
		return;
	}
	m.n += 5 + chunk;
	wm_u8(&m, 0x14, end ? 1 : 0);               /* end */
	wm_send(&m);
	w_bdf_off += chunk;
	w_bdf_seg++;
	w_step = "bdf download";
}

/* bdwlan.bNN for board ids < 0x100, bdwlan.NNN otherwise, bdwlan.bin as fallback */
static void
w_start_bdf(void)
{
	char name[40];
	if (w_bdf_started || !w_msa_ind || !w_cap_done) {
		return;
	}
	w_bdf_started = true;
	if (w_board == 0xFF) {
		snprintf(name, sizeof(name), "image/bdwlan.bin");
	} else if (w_board < 0x100) {
		snprintf(name, sizeof(name), "image/bdwlan.b%02x", w_board);
	} else {
		snprintf(name, sizeof(name), "image/bdwlan.%03x", w_board);
	}
	w_bdf_fi = fw_all_find(name);
	if (w_bdf_fi < 0) {
		LOG("wlfw: %s missing, using bdwlan.bin", name);
		snprintf(name, sizeof(name), "image/bdwlan.bin");
		w_bdf_fi = fw_all_find(name);
	}
	if (w_bdf_fi < 0) {
		LOG("wlfw: no board data file");
		return;
	}
	w_bdf_len = fw_all[w_bdf_fi].size;
	w_bdf_off = 0;
	w_bdf_seg = 0;
	LOG("t=%u ms: wlfw: board data %s, %u bytes", g_ms, name, w_bdf_len);
	w_bdf_segment();
}

/*
 * ath10k_snoc_wlan_enable(): the copy-engine layout (Linux snoc.c target_ce_config_wlan,
 * target_service_to_ce_map_wlan, target_shadow_reg_cfg_map). PIPEDIR 1 in, 2 out, 3 inout;
 * flags bit 3 = DIS_INTR.
 */
static const uint32_t w_tgt_ce[12][5] = {
	{ 0, 2, 32, 2048, 0 }, { 1, 1, 32, 2048, 0 }, { 2, 1, 64, 2048, 0 }, { 3, 2, 32, 2048, 0 },
	{ 4, 2, 256, 256, 8 }, { 5, 2, 1024, 64, 8 }, { 6, 3, 32, 16384, 0 }, { 7, 4, 0, 0, 8 },
	{ 8, 1, 32, 2048, 0 }, { 9, 1, 32, 2048, 0 }, { 10, 1, 32, 2048, 0 }, { 11, 1, 32, 2048, 0 },
};
static const uint32_t w_svc_ce[][3] = {
	{ 0x104, 2, 3 }, { 0x104, 1, 2 }, { 0x102, 2, 3 }, { 0x102, 1, 2 }, { 0x101, 2, 3 }, { 0x101, 1, 2 },
	{ 0x103, 2, 3 }, { 0x103, 1, 2 }, { 0x100, 2, 3 }, { 0x100, 1, 2 }, { 0x001, 2, 0 }, { 0x001, 1, 2 },
	{ 0xFE00, 2, 0 }, { 0xFE00, 1, 2 }, { 0x300, 2, 4 }, { 0x300, 1, 1 }, { 0xFE00, 2, 5 }, { 0x301, 1, 9 },
	{ 0x302, 1, 10 }, { 0x600, 1, 11 }, { 0, 0, 0 },
};
static const uint16_t w_shadow[12][2] = {
	{ 0, 0x3C }, { 3, 0x3C }, { 4, 0x3C }, { 5, 0x3C }, { 7, 0x3C }, { 1, 0x40 },
	{ 2, 0x40 }, { 7, 0x40 }, { 8, 0x40 }, { 9, 0x40 }, { 10, 0x40 }, { 11, 0x40 },
};

static void
w_on_cap(const uint8_t *d, uint32_t len)
{
	uint16_t l = 0;
	const uint8_t *v;
	char s[80];
	uint32_t a = 0, b = 0;

	if ((v = qm_find(d, len, 0x10, &l)) != NULL && l >= 8) {
		memcpy(&a, v, 4);
		memcpy(&b, v + 4, 4);
		LOG("wlfw: chip_id %x family %x", a, b);
	}
	if ((v = qm_find(d, len, 0x11, &l)) != NULL && l >= 4) {
		memcpy(&w_board, v, 4);
	}
	LOG("wlfw: board_id %x", w_board);
	if ((v = qm_find(d, len, 0x13, &l)) != NULL && l >= 5) {
		size_t n = MIN((size_t)v[4], MIN((size_t)(l - 5), sizeof(s) - 1));
		memcpy(s, v + 5, n);
		s[n] = 0;
		memcpy(&a, v, 4);
		LOG("wlfw: fw_version %x built %s", a, s);
	}
	if ((v = qm_find(d, len, 0x14, &l)) != NULL) {
		size_t n = MIN((size_t)l, sizeof(s) - 1);
		memcpy(s, v, n);
		s[n] = 0;
		LOG("wlfw: fw_build_id %s", s);
	}
}

static void
w_on_msa_info(const uint8_t *d, uint32_t len)
{
	uint16_t l = 0;
	const uint8_t *v = qm_find(d, len, 0x03, &l);
	uint32_t n = (v != NULL && l >= 1) ? v[0] : 0;

	LOG("wlfw: %u MSA region(s)", n);
	IOSleep(20);                                /* ath10k: give the modem time before the XPU update */
	for (uint32_t i = 0; i < n && 1 + 13 * (i + 1) <= l; i++) {
		const uint8_t *e = v + 1 + 13 * i;      /* u64 addr, u32 size, u8 secure */
		uint64_t addr;
		uint32_t size;
		memcpy(&addr, e, 8);
		memcpy(&size, e + 8, 4);
		uint8_t sec = e[12];
		static const uint32_t vm[3] = { VMID_MSS_MSA, VMID_WLAN, VMID_WLAN_CE };
		LOG(" MSA region %u: %llx + %x secure %u", i, addr, size, sec);
		if (addr < WLFW_MSA_BASE || addr + size > (uint64_t)WLFW_MSA_BASE + WLFW_MSA_SIZE) {
			LOG(" out of the MSA window, skipped");
			continue;
		}
		(void)scm_assign(addr, size, vm, sec ? 2 : 3);
	}
}

static unsigned w_arrivals, w_cap_tries, w_cap_retry_ms;

static void
wlfw_arrive(uint32_t node, uint32_t port)
{
	if ((w_node != 0 || w_port != 0) && w_node == node && w_port == port) {
		return;
	}
	if (w_arrivals++ != 0) {
		/* the WLAN PD restarted (new server port): start the handshake again */
		LOG("t=%u ms: WLFW back on port %x (restart %u, last step: %s): handshake again", g_ms, port, w_arrivals - 1, w_step);
		w_msa_ind = w_cap_done = w_bdf_started = w_fw_ready = w_mission = false;
		w_board = 0xFF;
		w_cap_tries = 0;
		w_cap_retry_ms = 0;
	}
	w_node = node;
	w_port = port;
	struct wmsg m;
	if (wm_new(&m, WLFW_IND_REGISTER)) {
		wm_u8(&m, 0x10, 1);                     /* fw_ready_enable */
		wm_u8(&m, 0x13, 1);                     /* msa_ready_enable */
		wm_u32(&m, 0x15, WLFW_CLIENT_ID);       /* client_id */
		wm_send(&m);
		w_step = "ind_register sent";
	}
}

static void
wlfw_rx(const uint8_t *d, uint32_t len)
{
	uint16_t msg, err = 0, l = 0;
	struct wmsg m;

	if (len < 7) {
		return;
	}
	memcpy(&msg, d + 3, 2);
	if (d[0] == QMI_IND) {
		if (msg == WLFW_MSA_READY_IND) {
			LOG("t=%u ms: wlfw: MSA_READY indication", g_ms);
			w_msa_ind = true;
			w_start_bdf();
		} else if (msg == WLFW_FW_READY_IND) {
			w_fw_ready = true;
			LOG("t=%u ms: *** WLAN FIRMWARE READY ***", g_ms);
			if (wm_new(&m, WLFW_WLAN_CFG)) {
				wm_array(&m, 0x11, 12, w_tgt_ce, sizeof(w_tgt_ce));
				wm_array(&m, 0x12, (uint8_t)(sizeof(w_svc_ce) / sizeof(w_svc_ce[0])), w_svc_ce, sizeof(w_svc_ce));
				wm_array(&m, 0x13, 12, w_shadow, sizeof(w_shadow));
				wm_send(&m);
				w_step = "wlan_cfg sent";
			}
		} else {
			LOG("wlfw: indication %x", msg);
		}
		return;
	}
	if (d[0] != QMI_RESP) {
		return;
	}
	bool ok = w_result(d, len, &err);
	switch (msg) {
	case WLFW_IND_REGISTER: {
		const uint8_t *v = qm_find(d, len, 0x10, &l);
		uint64_t st = 0;
		if (v != NULL && l >= 8) {
			memcpy(&st, v, 8);
		}
		LOG("t=%u ms: wlfw: ind_register %s err %u, fw_status %llx", g_ms, ok ? "ok" : "FAILED", err, st);
		if (wm_new(&m, WLFW_HOST_CAP)) {
			wm_u32(&m, 0x10, 0);                /* daemon_support = 0 */
			wm_send(&m);
			w_step = "host_cap sent";
		}
		break;
	}
	case WLFW_HOST_CAP:
		LOG("wlfw: host_cap %s err %u", ok ? "ok" : "rejected (not fatal)", err);
		if (wm_new(&m, WLFW_MSA_INFO)) {
			uint64_t a = WLFW_MSA_BASE;
			wm_tlv(&m, 0x01, &a, 8);            /* msa_addr */
			wm_u32(&m, 0x02, WLFW_MSA_SIZE);
			wm_send(&m);
			w_step = "msa_info sent";
		}
		break;
	case WLFW_MSA_INFO:
		LOG("wlfw: msa_info %s err %u", ok ? "ok" : "FAILED", err);
		if (ok) {
			w_on_msa_info(d, len);
			w_simple(WLFW_MSA_READY, "msa_ready sent");
		}
		break;
	case WLFW_MSA_READY:
		LOG("wlfw: msa_ready %s err %u", ok ? "ok" : "FAILED", err);
		w_simple(WLFW_CAP, "cap sent");
		break;
	case WLFW_CAP:
		LOG("t=%u ms: wlfw: cap %s err %u (try %u)", g_ms, ok ? "ok" : "FAILED", err, w_cap_tries + 1);
		if (ok) {
			w_on_cap(d, len);
		} else {
			char hx[3 * 24 + 1];
			uint32_t n = MIN(len, 24u);
			for (uint32_t i = 0; i < n; i++) {
				snprintf(hx + 3 * i, 4, "%02x ", d[i]);
			}
			hx[3 * n] = 0;
			LOG("wlfw: cap response %u bytes: %s", len, hx);
			if (++w_cap_tries < 5) {
				w_cap_retry_ms = g_ms + 500;    /* ask again from the poll loop */
				break;
			}
		}
		w_cap_done = true;
		w_step = "waiting MSA_READY ind";
		w_start_bdf();
		break;
	case WLFW_BDF_DOWNLOAD:
		if (!ok) {
			LOG("wlfw: bdf segment %u FAILED err %u", w_bdf_seg - 1, err);
			break;
		}
		if (w_bdf_off < w_bdf_len) {
			w_bdf_segment();
		} else {
			LOG("wlfw: board data sent in %u segment(s)", w_bdf_seg);
			if (wm_new(&m, WLFW_CAL_REPORT)) {
				uint8_t none = 0;
				wm_tlv(&m, 0x01, &none, 1);     /* meta_data_len = 0 */
				wm_send(&m);
				w_step = "cal_report sent";
			}
		}
		break;
	case WLFW_CAL_REPORT:
		LOG("wlfw: cal_report %s err %u, waiting for FW_READY", ok ? "ok" : "FAILED", err);
		w_step = "waiting FW_READY";
		break;
	case WLFW_WLAN_CFG:
		LOG("wlfw: wlan_cfg %s err %u", ok ? "ok" : "FAILED", err);
		if (ok && wm_new(&m, WLFW_WLAN_MODE)) {
			wm_u32(&m, 0x01, 0);                /* mission */
			wm_u8(&m, 0x10, 0);                 /* hw_debug */
			wm_send(&m);
			w_step = "wlan_mode sent";
		}
		break;
	case WLFW_WLAN_MODE:
		LOG("t=%u ms: wlfw: wlan_mode mission %s err %u", g_ms, ok ? "ok" : "FAILED", err);
		if (ok) {
			w_mission = true;
			w_step = "WLAN ON (mission)";
			LOG("*** WLAN ON (mission mode) ***");
			if (wifi_enabled()) {
				IOSleep(100);
				(void)ce_start();               /* only now: before mission a CE access hangs the bus */
			}
		}
		break;
	default:
		LOG("wlfw: response %x %s err %u", msg, ok ? "ok" : "FAILED", err);
		break;
	}
}

/* ---------------------------------------------------------------- WLAN radio (wifi=1) */

/*
 * After WLAN_MODE mission (port of topaz-woa wlanprobe.c, ce.c, htc.c, wmi.c, scan.c):
 *  - apps SMMU: the WLAN stream 0x1A0 gets an identity context bank (SCTLR.M=0); a global
 *    bypass S2CR is turned into a fault by this SMMU. Done before the modem boots.
 *  - copy engines at 0xC800000+0x240000, polled; rings and buffers from our pool (Normal NC).
 *  - HTC: READY on ep0, connect HTT_DATA and WMI_CONTROL, SETUP_COMPLETE_EX.
 *  - WMI (TLV): SERVICE_READY -> INIT -> READY, then a passive scan (nothing is transmitted);
 *    each BSS seen is logged once, its BSSID masked.
 */
#define SMMU_PA             0x0C600000ull
#define SMMU_SIZE           0x00080000u
#define WLAN_SID            0x1A0
#define WLAN_SID_MASK       0x1
#define CB_CLONE_CBAR       0x0001f000u         /* S1_TRANS_S2_FAULT, S1_MEMATTR WB */
#define CB_CLONE_CBA2R      0x00000001u         /* VA64 */
#define CB_CLONE_SCTLR      0x000000e0u         /* CFCFG|CFIE|CFRE, M=0: no translation */

static int wifi_on = -1;

static bool
wifi_enabled(void)
{
	if (wifi_on < 0) {
		uint32_t v = 0;
		wifi_on = PE_parse_boot_argn("wifi", &v, sizeof(v)) && v != 0;
	}
	return wifi_on > 0;
}

static inline uint32_t r32(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void w32(volatile uint8_t *b, uint32_t o, uint32_t v) { mb(); *(volatile uint32_t *)(b + o) = v; }

static bool
smmu_sid_match(uint32_t smr, uint32_t sid, uint32_t mask)
{
	uint32_t id = smr & 0xFFFF, m = (smr >> 16) & 0x7FFF;
	return ((id ^ sid) & ~(m | mask) & 0x7FFF) == 0;
}

static bool
smmu_identity(uint32_t sid, uint32_t mask, const char *name)
{
	volatile uint8_t *s = (volatile uint8_t *)ml_io_map(SMMU_PA, SMMU_SIZE);
	if (s == NULL) {
		LOG("smmu: map failed");
		return false;
	}
	uint32_t cr0 = r32(s, 0x0), id0 = r32(s, 0x20), id1 = r32(s, 0x24);
	uint32_t nsmr = id0 & 0xFF, ncb = id1 & 0xFF;
	uint32_t psize = (id1 & (1u << 31)) ? 0x10000 : 0x1000, npage = 1u << (((id1 >> 28) & 7) + 1);
	uint32_t slot = ~0u, cb = ~0u, smr, s2cr;
	uint64_t used = 0;

	LOG("smmu: sCR0 %08x ID0 %08x ID1 %08x: %u SMRs, %u CBs, page %x x%u", cr0, id0, id1, nsmr, ncb, psize, npage);
	for (uint32_t i = 0; i < nsmr && i < 128; i++) {
		smr = r32(s, 0x800 + 4 * i);
		s2cr = r32(s, 0xC00 + 4 * i);
		if ((smr >> 31) != 0 && smmu_sid_match(smr, sid, mask)) {
			slot = i;                           /* already there: reuse */
		} else if ((smr >> 31) == 0 && slot == ~0u) {
			slot = i;
		}
		if ((smr >> 31) != 0 && ((s2cr >> 16) & 3) == 0) {
			used |= 1ull << (s2cr & 0x3F);
		}
	}
	for (uint32_t i = 4; i < ncb && i < 64; i++) {  /* keep clear of the loader's cb0..3 */
		if ((used & (1ull << i)) == 0) {
			cb = i;
			break;
		}
	}
	if (slot == ~0u || cb == ~0u || (uint64_t)(npage + cb + 1) * psize > SMMU_SIZE) {
		LOG("smmu: no free SMR/CB (slot %u cb %u)", slot, cb);
		return false;
	}
	volatile uint8_t *gr1 = s + psize, *cbp = s + (uint64_t)npage * psize + (uint64_t)cb * psize;
	w32(cbp, 0x58, 0xFFFFFFFF);                 /* FSR: clear */
	w32(cbp, 0x20, 0);                          /* TTBR0 */
	w32(cbp, 0x24, 0);
	w32(cbp, 0x30, 0);                          /* TCR */
	w32(cbp, 0x10, 0);                          /* TCR2 */
	w32(cbp, 0x38, 0);                          /* MAIR */
	w32(cbp, 0x3C, 0);
	w32(gr1, 0x800 + 4 * cb, CB_CLONE_CBA2R);
	w32(gr1, 0x000 + 4 * cb, CB_CLONE_CBAR);
	w32(cbp, 0x0, CB_CLONE_SCTLR);              /* SCTLR last */
	w32(s, 0xC00 + 4 * slot, cb);               /* S2CR: type 0 (translate), this bank */
	w32(s, 0x800 + 4 * slot, (1u << 31) | (mask << 16) | sid);   /* SMR valid last */
	w32(s, 0x70, 0);                            /* sTLBGSYNC */
	uint32_t n;
	for (n = 0; n < 100000 && (r32(s, 0x74) & 1) != 0; n++) {
		IODelay(1);
	}
	smr = r32(s, 0x800 + 4 * slot);
	s2cr = r32(s, 0xC00 + 4 * slot);
	bool ok = (smr >> 31) != 0 && smmu_sid_match(smr, sid, mask) && ((s2cr >> 16) & 3) == 0 && (s2cr & 0x3F) == cb;
	LOG("smmu: %s identity (sid %x mask %x) CB%u SMR%u: SMR %08x S2CR %08x SCTLR %08x -> %s", name, sid, mask, cb,
	    slot, smr, s2cr, r32(cbp, 0x0), ok ? "OK" : "NOT as written");
	return ok;
}

/* ---------------- copy engines (ath10k ce.c, WCN3990: 64-bit descriptors, shadow regs, RRI in DDR) */

#define CE_PA               (0x0C800000ull + 0x240000)
#define CE_COUNT            12
#define SHADOW_PA           (0x0C800000ull + 0x32000)
#define SR_BASE_LO          0x00
#define SR_BASE_HI          0x04
#define SR_SIZE             0x08
#define DR_BASE_LO          0x0C
#define DR_BASE_HI          0x10
#define DR_SIZE             0x14
#define CTRL1               0x18
#define SR_WR_IDX           0x3C
#define DR_WR_IDX           0x40
#define CUR_SRRI            0x44
#define CUR_DRRI            0x48
#define SRC_WM              0x4C
#define DST_WM              0x50
#define WRAP_RRI_LO         0xC004
#define WRAP_RRI_HI         0xC008
#define CTRL1_DMAX          0x0000FFFFu
#define CTRL1_SRC_SWAP      (1u << 17)
#define CTRL1_DST_SWAP      (1u << 18)
#define CTRL1_RRI_UPD       (1u << 19)
#define ADDR_HI_MASK        0x1Fu
#define ATTR_DIS_INTR       0x08
#define RX_BUF_SIZE         2048
#define RX_POST             32
#define TX_SLOTS_MAX        64

struct ce_desc { uint64_t addr; uint16_t nbytes, flags; uint32_t toeplitz; } __attribute__((packed));
struct ce_attr { uint16_t src_n, src_max, dst_n, flags; };

/* host_ce_config_wlan (ath10k snoc.c) */
static const struct ce_attr ce_attr[CE_COUNT] = {
	{ 16, 2048, 0, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 64, 0 }, { 32, 2048, 0, 0 },
	{ 2048, 256, 0, ATTR_DIS_INTR }, { 0, 512, 512, 0 }, { 0, 0, 0, 0 }, { 2, 2048, 2, 0 },
	{ 0, 2048, 128, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 512, 0 }, { 0, 2048, 512, 0 },
};

struct ce_state {
	volatile struct ce_desc *src, *dst;
	uint64_t src_pa, dst_pa, rx_pa, tx_pa;
	uint32_t src_n, dst_n, src_wr, dst_wr, dst_sw;
	uint8_t *rx_buf, *tx_buf;
	uint32_t rx, tx, tx_full;
};
static struct ce_state ce[CE_COUNT];
static volatile uint8_t *ce_regs, *ce_shadow;
static volatile uint32_t *ce_rri;
static uint64_t ce_rri_pa;
static bool ce_up;

static bool ce_host_rx(uint32_t c) { return ce_attr[c].dst_n != 0 && c != 7; }
static bool ce_host_tx(uint32_t c) { return c == 0 || c == 3 || c == 4; }
static uint32_t ce_tx_slots(uint32_t c) { return MIN(ce[c].src_n, (uint32_t)TX_SLOTS_MAX); }
static uint32_t ce_tx_sz(uint32_t c) { return ce_attr[c].src_max; }
static uint32_t pow2(uint32_t n) { uint32_t p = 1; while (p < n) { p <<= 1; } return p; }
static uint32_t ce_rd(uint32_t c, uint32_t o) { return r32(ce_regs, c * 0x1000 + o); }
static void ce_wr(uint32_t c, uint32_t o, uint32_t v) { w32(ce_regs, c * 0x1000 + o, v); }

static void
ce_post(uint32_t c)
{
	struct ce_state *s = &ce[c];
	uint32_t slot = s->dst_wr;
	s->dst[slot].addr = s->rx_pa + (uint64_t)(slot % RX_POST) * RX_BUF_SIZE;
	s->dst[slot].nbytes = 0;
	s->dst_wr = (slot + 1) & (s->dst_n - 1);
}

static bool
ce_start(void)
{
	uint64_t off = 0x1000, pa = 0;              /* RRI in the first page */

	if (ce_up) {
		return true;
	}
	ce_regs = (volatile uint8_t *)ml_io_map(CE_PA, (CE_COUNT + 1) * 0x1000);
	ce_shadow = (volatile uint8_t *)ml_io_map(SHADOW_PA, 0x1000);
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		ce[c].src_n = ce_attr[c].src_n ? pow2(ce_attr[c].src_n) : 0;
		ce[c].dst_n = ce_attr[c].dst_n ? pow2(ce_attr[c].dst_n) : 0;
		off += ((uint64_t)ce[c].src_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		off += ((uint64_t)ce[c].dst_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		off += ce_host_rx(c) ? RX_POST * RX_BUF_SIZE : 0;
		off += ce_host_tx(c) ? ce_tx_slots(c) * ce_tx_sz(c) : 0;
	}
	uint8_t *va = (uint8_t *)pool_alloc(off, &pa);
	if (ce_regs == NULL || ce_shadow == NULL || va == NULL) {
		LOG("ce: map/alloc failed (%llu bytes)", off);
		return false;
	}
	LOG("ce: host memory %llx + %llx", pa, off);
	ce_rri = (volatile uint32_t *)va;
	ce_rri_pa = pa;
	off = 0x1000;
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (s->src_n != 0) {
			s->src = (volatile struct ce_desc *)(va + off);
			s->src_pa = pa + off;
			off += ((uint64_t)s->src_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		}
		if (s->dst_n != 0) {
			s->dst = (volatile struct ce_desc *)(va + off);
			s->dst_pa = pa + off;
			off += ((uint64_t)s->dst_n * sizeof(struct ce_desc) + 0xfff) & ~0xfffull;
		}
		if (ce_host_rx(c)) {
			s->rx_buf = va + off;
			s->rx_pa = pa + off;
			off += RX_POST * RX_BUF_SIZE;
		}
		if (ce_host_tx(c)) {
			s->tx_buf = va + off;
			s->tx_pa = pa + off;
			off += ce_tx_slots(c) * ce_tx_sz(c);
		}
	}
	/* the CE writes its read indices to ce_rri[ce] (src [15:0], dst [31:16]) */
	w32(ce_regs, WRAP_RRI_LO, (uint32_t)ce_rri_pa);
	w32(ce_regs, WRAP_RRI_HI, (uint32_t)(ce_rri_pa >> 32) & ADDR_HI_MASK);
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		ce_wr(c, CTRL1, ce_rd(c, CTRL1) | CTRL1_RRI_UPD);
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (s->src_n != 0) {                    /* ath10k_ce_init_src_ring */
			s->src_wr = ce_rd(c, SR_WR_IDX) & (s->src_n - 1);
			ce_wr(c, SR_BASE_LO, (uint32_t)s->src_pa);
			ce_wr(c, SR_BASE_HI, (uint32_t)(s->src_pa >> 32) & ADDR_HI_MASK);
			ce_wr(c, SR_SIZE, s->src_n);
			ce_wr(c, CTRL1, (ce_rd(c, CTRL1) & ~(CTRL1_DMAX | CTRL1_SRC_SWAP)) | ce_attr[c].src_max);
			ce_wr(c, SRC_WM, s->src_n);
		}
		if (s->dst_n != 0) {                    /* ath10k_ce_init_dest_ring */
			s->dst_sw = ce_rd(c, CUR_DRRI) & (s->dst_n - 1);
			s->dst_wr = ce_rd(c, DR_WR_IDX) & (s->dst_n - 1);
			ce_wr(c, DR_BASE_LO, (uint32_t)s->dst_pa);
			ce_wr(c, DR_BASE_HI, (ce_rd(c, DR_BASE_HI) & ~ADDR_HI_MASK) | ((uint32_t)(s->dst_pa >> 32) & ADDR_HI_MASK));
			ce_wr(c, DR_SIZE, s->dst_n);
			ce_wr(c, CTRL1, ce_rd(c, CTRL1) & ~CTRL1_DST_SWAP);
			ce_wr(c, DST_WM, s->dst_n);
		}
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {   /* ath10k_snoc_rx_post */
		if (!ce_host_rx(c)) {
			continue;
		}
		for (uint32_t i = 0; i < RX_POST; i++) {
			ce_post(c);
		}
		mb();
		ce_wr(c, DR_WR_IDX, ce[c].dst_wr);
	}
	ce_up = true;
	LOG("t=%u ms: ce: %u rings programmed, rx posted", g_ms, CE_COUNT);
	return true;
}

static void htc_rx(uint32_t c, const uint8_t *d, uint32_t len);

static bool
ce_poll(void)
{
	bool any = false;
	if (!ce_up) {
		return false;
	}
	for (uint32_t c = 0; c < CE_COUNT; c++) {
		struct ce_state *s = &ce[c];
		if (!ce_host_rx(c)) {
			continue;
		}
		for (int k = 0; k < 16; k++) {
			uint32_t slot = s->dst_sw, n = s->dst[slot].nbytes;
			if (n == 0) {
				break;
			}
			mb();                               /* payload after the descriptor */
			s->dst[slot].nbytes = 0;
			s->dst_sw = (slot + 1) & (s->dst_n - 1);
			s->rx++;
			htc_rx(c, s->rx_buf + (slot % RX_POST) * RX_BUF_SIZE, n);
			ce_post(c);
			mb();
			ce_wr(c, DR_WR_IDX, s->dst_wr);
			any = true;
		}
	}
	return any;
}

static bool
ce_send(uint32_t c, const void *data, uint32_t len, uint32_t transfer_id)
{
	if (!ce_up || c >= CE_COUNT || !ce_host_tx(c) || len > ce_tx_sz(c)) {
		return false;
	}
	struct ce_state *s = &ce[c];
	uint32_t mask = s->src_n - 1, sw = ce_rri[c] & 0xFFFF;
	if (((sw - 1 - s->src_wr) & mask) == 0 || ((s->src_wr - sw) & mask) >= ce_tx_slots(c) - 1) {
		s->tx_full++;
		return false;
	}
	uint32_t slot = s->src_wr;
	memcpy(s->tx_buf + (slot % ce_tx_slots(c)) * ce_tx_sz(c), data, len);
	volatile struct ce_desc *d = &s->src[slot];
	d->addr = s->tx_pa + (uint64_t)(slot % ce_tx_slots(c)) * ce_tx_sz(c);
	d->nbytes = (uint16_t)len;
	d->flags = (uint16_t)((transfer_id << 4) & 0xFFF0);
	d->toeplitz = 0;
	s->src_wr = (slot + 1) & mask;
	s->tx++;
	mb();
	*(volatile uint32_t *)(ce_shadow + 4 * c) = s->src_wr;
	mb();
	return true;
}

/* ---------------- HTC (ath10k htc.c) */

#define HTC_MSG_READY       1
#define HTC_MSG_CONNECT     2
#define HTC_MSG_CONNECT_RESP 3
#define HTC_MSG_SETUP_COMPLETE_EX 5
#define HTC_FLAG_NEED_CREDIT_UPDATE 0x01
#define HTC_FLAG_TRAILER    0x02
#define HTC_CONN_NO_CREDIT_FLOW 0x0008
#define SVC_RSVD_CTRL       0x0001
#define SVC_WMI_CONTROL     0x0100
#define SVC_HTT_DATA        0x0300
#define EP_MAX              8

struct htc_hdr { uint8_t eid, flags; uint16_t len; uint8_t trailer, seq; uint16_t pad; } __attribute__((packed));
struct htc_ep { uint16_t svc, max_msg; uint8_t ul, dl, seq, credits; bool up, credit_flow; uint32_t rx; };
static struct htc_ep ep[EP_MAX];
static uint8_t wmi_eid = 0xFF, htt_eid = 0xFF;
static bool htc_ready, htc_setup;

static bool
htc_send(uint8_t eid, const void *payload, uint32_t len)
{
	static uint8_t buf[2048];
	struct htc_hdr *h = (struct htc_hdr *)buf;
	if (eid >= EP_MAX || len + sizeof(*h) > sizeof(buf)) {
		return false;
	}
	struct htc_ep *e = &ep[eid];
	if (e->credit_flow) {
		if (e->credits == 0) {
			return false;
		}
		e->credits--;
	}
	bzero(h, sizeof(*h));
	h->eid = eid;
	h->len = (uint16_t)len;
	h->flags = e->credit_flow ? HTC_FLAG_NEED_CREDIT_UPDATE : 0;
	h->seq = e->seq++;
	memcpy(buf + sizeof(*h), payload, len);
	if (!ce_send(e->ul, buf, (uint32_t)sizeof(*h) + len, eid)) {
		LOG("htc: send on ep %u (ce%u) FAILED", eid, e->ul);
		return false;
	}
	return true;
}

static void
htc_connect(uint16_t svc)
{
	uint8_t m[8] = {};
	uint16_t id = HTC_MSG_CONNECT, flags = svc == SVC_WMI_CONTROL ? (uint16_t)(1u << 8) : HTC_CONN_NO_CREDIT_FLOW;
	memcpy(m, &id, 2);
	memcpy(m + 2, &svc, 2);
	memcpy(m + 4, &flags, 2);
	htc_send(0, m, sizeof(m));
}

/* WMI waits here while the endpoint has no credit */
#define WMIQ_LEN 6
#define WMIQ_MAX 2040
static uint8_t wmiq[WMIQ_LEN][WMIQ_MAX];
static uint16_t wmiq_len[WMIQ_LEN];
static uint32_t wmiq_head, wmiq_tail;

static void
wmiq_pump(void)
{
	while (wmiq_head != wmiq_tail && wmi_eid != 0xFF && ep[wmi_eid].credits != 0) {
		uint32_t i = wmiq_head % WMIQ_LEN;
		if (!htc_send(wmi_eid, wmiq[i], wmiq_len[i])) {
			break;
		}
		wmiq_head++;
	}
}

static bool
htc_wmi_send(const void *data, uint32_t len)
{
	if (wmi_eid == 0xFF || len > WMIQ_MAX) {
		return false;
	}
	if (wmiq_head == wmiq_tail && ep[wmi_eid].credits != 0) {
		return htc_send(wmi_eid, data, len);
	}
	if (wmiq_tail - wmiq_head >= WMIQ_LEN) {
		LOG("htc: wmi queue full, message dropped");
		return false;
	}
	uint32_t i = wmiq_tail++ % WMIQ_LEN;
	memcpy(wmiq[i], data, len);
	wmiq_len[i] = (uint16_t)len;
	return true;
}

static void wmi_rx(const uint8_t *d, uint32_t len);
static void htt_rx(const uint8_t *p, uint32_t len);

static void
htc_rx(uint32_t c, const uint8_t *d, uint32_t len)
{
	const struct htc_hdr *h = (const struct htc_hdr *)d;
	const uint8_t *p = d + sizeof(*h);
	if (len < sizeof(*h)) {
		return;
	}
	uint32_t plen = MIN((uint32_t)h->len, len - (uint32_t)sizeof(*h));
	if ((h->flags & HTC_FLAG_TRAILER) != 0 && h->trailer <= plen) {
		const uint8_t *t = p + plen - h->trailer;   /* records {id, len, data}; id 1 = credits */
		for (uint32_t o = 0; o + 2 <= h->trailer && o + 2 + t[o + 1] <= h->trailer; o += 2 + t[o + 1]) {
			if (t[o] == 1) {
				for (uint32_t i = 0; i + 1 < t[o + 1]; i += 2) {
					if (t[o + 2 + i] < EP_MAX) {
						ep[t[o + 2 + i]].credits = (uint8_t)(ep[t[o + 2 + i]].credits + t[o + 3 + i]);
					}
				}
			}
		}
		plen -= h->trailer;
		wmiq_pump();
	}
	if (h->eid < EP_MAX) {
		ep[h->eid].rx++;
	}
	if (h->eid == 0) {
		uint16_t msg = 0;
		if (plen >= 2) {
			memcpy(&msg, p, 2);
		}
		if (msg == HTC_MSG_READY && plen >= 8) {
			uint16_t cc, cs;
			memcpy(&cc, p + 2, 2);
			memcpy(&cs, p + 4, 2);
			htc_ready = true;
			ep[0].svc = SVC_RSVD_CTRL;
			ep[0].ul = 0;
			ep[0].dl = 2;
			ep[0].up = true;
			LOG("t=%u ms: *** HTC READY: credits %u x %u, %u endpoints ***", g_ms, cc, cs, p[6]);
			htc_connect(SVC_HTT_DATA);          /* ath10k_core_start: htt, then wmi */
		} else if (msg == HTC_MSG_CONNECT_RESP && plen >= 8) {
			uint16_t svc, max;
			memcpy(&svc, p + 2, 2);
			memcpy(&max, p + 6, 2);
			uint8_t status = p[4], eid = p[5];
			LOG("t=%u ms: htc: connect resp svc %x status %u eid %u max_msg %u", g_ms, svc, status, eid, max);
			if (status != 0 || eid >= EP_MAX) {
				return;
			}
			ep[eid].svc = svc;
			ep[eid].max_msg = max;
			ep[eid].ul = svc == SVC_WMI_CONTROL ? 3 : 4;
			ep[eid].dl = svc == SVC_WMI_CONTROL ? 2 : 1;
			ep[eid].up = true;
			ep[eid].credit_flow = svc == SVC_WMI_CONTROL;
			ep[eid].credits = ep[eid].credit_flow ? 1 : 0;
			if (svc == SVC_HTT_DATA) {
				htt_eid = eid;
				htc_connect(SVC_WMI_CONTROL);
			} else if (svc == SVC_WMI_CONTROL) {
				wmi_eid = eid;
				uint8_t m[12] = {};
				uint16_t id = HTC_MSG_SETUP_COMPLETE_EX;
				memcpy(m, &id, 2);
				htc_setup = htc_send(0, m, sizeof(m));
				LOG("t=%u ms: htc: SETUP_COMPLETE_EX %s", g_ms, htc_setup ? "sent" : "FAILED");
			}
		}
	} else if (h->eid == wmi_eid) {
		wmi_rx(p, plen);
	} else if (h->eid == htt_eid) {
		htt_rx(p, plen);
	}
}

/* ---------------- WMI (TLV, ath10k wmi-tlv.c) and the passive scan */

#define WMI_EV_SERVICE_READY 0x1
#define WMI_EV_READY        0x2
#define WMI_EV_SERVICE_AVAILABLE 0x3
#define WMI_EV_SCAN         0x3001
#define WMI_EV_MGMT_RX      0x7001
#define WMI_CMD_INIT        0x1
#define CMD_START_SCAN      0x3001
#define CMD_SCAN_CHAN_LIST  0x3003
#define CMD_VDEV_CREATE     0x5001
#define TAG_ARRAY_UINT32    0x10
#define TAG_ARRAY_BYTE      0x11
#define TAG_ARRAY_STRUCT    0x12
#define TAG_ARRAY_FIXED_STRUCT 0x13
#define TAG_SERVICE_READY   0x20
#define TAG_HOST_MEM_REQ    0x22
#define TAG_READY_EVENT     0x23
#define TAG_SCAN_EVENT      0x24
#define TAG_MGMT_RX_HDR     0x2C
#define TAG_INIT_CMD        0x4A
#define TAG_RESOURCE_CONFIG 0x4B
#define TAG_HOST_MEM_CHUNK  0x4C
#define TAG_START_SCAN_CMD  0x4D
#define TAG_SCAN_CHAN_LIST_CMD 0x4F
#define TAG_CHANNEL         0x50
#define TAG_VDEV_CREATE_CMD 0x56
#define TAG_SERVICE_AVAILABLE 0x22F
#define ABI_VER0            0x01000000u
#define ABI_VER1            53
#define ABI_NS0             0x5F414351u         /* "QCA_" */
#define ABI_NS1             0x00004C4Du         /* "ML" */
#define SVC_RX_FULL_REORDER 65
#define SVC_EXTEND_ADDRESS  179
#define NUM_PEERS           33
#define NUM_VDEVS           4
#define MAX_MEM_REQS        8
#define MAX_MEM_TOTAL       (4u << 20)

struct wmi_mem_req { uint32_t req_id, unit_size, num_unit_info, num_units; };
static uint32_t wmi_svc_map[32], wmi_svc_ext[4], wmi_nreq;
static struct wmi_mem_req wmi_req[MAX_MEM_REQS];
static uint64_t wmi_chunk_pa[MAX_MEM_REQS];
static uint32_t wmi_chunk_len[MAX_MEM_REQS];
static bool wmi_init_sent, wmi_ready, wmi_seen_bmap, wmi_seen_reqs;
static uint8_t wmi_mac[6];

static bool
wmi_svc(uint32_t bit)
{
	if (bit < 128) {
		return (wmi_svc_map[bit / 4] & (1u << (bit % 4))) != 0;
	}
	bit -= 128;
	return bit < 128 && (wmi_svc_ext[bit / 32] & (1u << (bit % 32))) != 0;
}

static uint8_t *
wmi_put_tlv(uint8_t *p, uint16_t tag, uint32_t len)
{
	uint16_t l = (uint16_t)len;
	memcpy(p, &l, 2);
	memcpy(p + 2, &tag, 2);
	return p + 4;
}

static uint8_t *
put32(uint8_t *p, uint32_t v)
{
	memcpy(p, &v, 4);
	return p + 4;
}

static bool
wmi_send(uint32_t cmd, const void *tlvs, uint32_t len)
{
	static uint8_t m[2048];
	if (len + 4 > sizeof(m)) {
		return false;
	}
	put32(m, cmd & 0xFFFFFF);
	memcpy(m + 4, tlvs, len);
	return htc_wmi_send(m, len + 4);
}

/* walk the TLVs of SERVICE_READY (ath10k_wmi_tlv_svc_rdy_parse) */
static void
wmi_svc_ready(const uint8_t *p, uint32_t len)
{
	for (uint32_t o = 0; o + 4 <= len;) {
		uint16_t l, tag;
		memcpy(&l, p + o, 2);
		memcpy(&tag, p + o + 2, 2);
		if (o + 4 + l > len) {
			break;
		}
		const uint8_t *v = p + o + 4;
		if (tag == TAG_SERVICE_READY && l >= 16) {
			uint32_t fw, a0, rf;
			memcpy(&fw, v, 4);
			memcpy(&a0, v + 4, 4);
			memcpy(&rf, v + 4 + 24 + 8, 4);
			LOG("wmi: fw %08x abi %08x rf_chains %u", fw, a0, rf);
		} else if (tag == TAG_ARRAY_UINT32 && !wmi_seen_bmap) {
			wmi_seen_bmap = true;
			memcpy(wmi_svc_map, v, MIN((uint32_t)l, (uint32_t)sizeof(wmi_svc_map)));
		} else if (tag == TAG_ARRAY_STRUCT && !wmi_seen_reqs) {
			wmi_seen_reqs = true;
			for (uint32_t q = 0; q + 4 <= l;) {
				uint16_t ql, qt;
				memcpy(&ql, v + q, 2);
				memcpy(&qt, v + q + 2, 2);
				if (q + 4 + ql > l) {
					break;
				}
				if (qt == TAG_HOST_MEM_REQ && ql >= sizeof(struct wmi_mem_req) && wmi_nreq < MAX_MEM_REQS) {
					memcpy(&wmi_req[wmi_nreq], v + q + 4, sizeof(struct wmi_mem_req));
					LOG("wmi: mem_req %u: id %u unit %u info %x units %u", wmi_nreq, wmi_req[wmi_nreq].req_id,
					    wmi_req[wmi_nreq].unit_size, wmi_req[wmi_nreq].num_unit_info, wmi_req[wmi_nreq].num_units);
					wmi_nreq++;
				}
				q += 4 + ql;
			}
		}
		o += 4 + l;
	}
}

static void
wmi_send_init(void)
{
	static uint8_t m[1024];
	uint8_t *p = m;
	uint32_t total = 0, n = wmi_nreq;
	uint64_t pa = 0;

	for (uint32_t i = 0; i < n; i++) {
		uint32_t units = wmi_req[i].num_units;
		if (wmi_req[i].num_unit_info & 6) {
			units = NUM_PEERS + 1;
		} else if (wmi_req[i].num_unit_info & 1) {
			units = NUM_VDEVS + 1;
		}
		wmi_chunk_len[i] = units * ((wmi_req[i].unit_size + 3) & ~3u);
		total += (wmi_chunk_len[i] + 0xfff) & ~0xfffu;
	}
	if (total != 0 && (total > MAX_MEM_TOTAL || pool_alloc(total, &pa) == NULL)) {
		LOG("wmi: host memory %u bytes not given", total);
		n = 0;
	}
	for (uint32_t i = 0, off = 0; i < n; i++) {
		wmi_chunk_pa[i] = pa + off;
		off += (wmi_chunk_len[i] + 0xfff) & ~0xfffu;
	}
	bzero(m, sizeof(m));
	p = wmi_put_tlv(p, TAG_INIT_CMD, 24 + 4);
	p = put32(p, ABI_VER0);
	p = put32(p, ABI_VER1);
	p = put32(p, ABI_NS0);
	p = put32(p, ABI_NS1);
	p = put32(p, 0);
	p = put32(p, 0);
	p = put32(p, n);
	/* wmi_tlv_resource_config, 44 x u32, ath10k values for wcn3990 */
	uint32_t rc[44] = {};
	rc[0] = NUM_VDEVS;
	rc[1] = NUM_PEERS;
	if (wmi_svc(SVC_RX_FULL_REORDER)) {
		rc[2] = NUM_VDEVS;
		rc[3] = NUM_VDEVS;
	}
	rc[4] = 2;                                  /* peer keys */
	rc[5] = NUM_PEERS * 2;                      /* tids */
	rc[6] = 16;                                 /* ast skid limit */
	rc[7] = 0x7;                                /* tx chain mask */
	rc[8] = 0x7;                                /* rx chain mask */
	rc[9] = 0x64; rc[10] = 0x64; rc[11] = 0x64; rc[12] = 0x28;
	rc[13] = 1;                                 /* rx decap: native wifi */
	rc[14] = 4;                                 /* scan max pending */
	rc[15] = NUM_VDEVS;                         /* bmiss offload */
	rc[16] = NUM_VDEVS;                         /* roam offload */
	rc[17] = 8;                                 /* roam ap profiles */
	rc[21] = 0x400;                             /* tx dbglog size */
	rc[22] = 2;                                 /* wds entries */
	rc[27] = 2;                                 /* gtk offload */
	rc[28] = 1024 + 32;                         /* msdu descs */
	rc[29] = 2;                                 /* frag entries */
	rc[30] = 1;                                 /* tdls vdevs */
	rc[31] = 0x20;                              /* tdls conn table */
	rc[32] = 2;                                 /* beacon tx offload */
	rc[33] = 5;                                 /* mcast filter entries */
	rc[34] = 22;                                /* wow filters */
	rc[35] = 6;                                 /* keepalive patterns */
	rc[37] = 1;
	rc[38] = 1;
	rc[43] = 1u << 9;                           /* MGMT_BUNDLE_TX_COMPL */
	p = wmi_put_tlv(p, TAG_RESOURCE_CONFIG, sizeof(rc));
	memcpy(p, rc, sizeof(rc));
	p += sizeof(rc);
	p = wmi_put_tlv(p, TAG_ARRAY_STRUCT, n * 20);
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *c = wmi_put_tlv(p, TAG_HOST_MEM_CHUNK, 16);
		c = put32(c, wmi_req[i].req_id);
		c = put32(c, (uint32_t)wmi_chunk_pa[i]);
		c = put32(c, wmi_chunk_len[i]);
		put32(c, wmi_svc(SVC_EXTEND_ADDRESS) ? (uint32_t)(wmi_chunk_pa[i] >> 32) : 0);
		p += 20;
	}
	wmi_init_sent = wmi_send(WMI_CMD_INIT, m, (uint32_t)(p - m));
	LOG("t=%u ms: wmi: INIT (%u chunks, %u bytes host memory) %s", g_ms, n, total, wmi_init_sent ? "sent" : "FAILED");
}

/* scan: 2.4 GHz 1-13, 5 GHz UNII-1/2/2e/3 */
static const uint16_t scan_freqs[] = {
	2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472,
	5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320,
	5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700,
	5745, 5765, 5785, 5805, 5825,
};
#define NFREQ ((uint32_t)(sizeof(scan_freqs) / sizeof(scan_freqs[0])))
static const uint8_t our_mac[6] = { 0x02, 0x54, 0x4F, 0x50, 0x41, 0x5A };   /* locally administered */
struct bss {
	uint8_t bssid[6]; char ssid[33]; uint8_t chan; int8_t rssi; uint16_t seen;
	uint8_t raw_ssid[32], raw_ssid_len; uint16_t freq, bintval, capab, ie_len; uint32_t last_ms; uint8_t ie[768];
};
static struct bss bss[64];
static uint32_t nbss, scans, scan_frames;
static volatile uint32_t api_scans_done;
static bool scan_started, scanning;
static unsigned next_scan_ms;

static bool
scan_send_start(void)
{
	static uint8_t m[512];
	uint8_t *p = m, *c;
	bzero(m, sizeof(m));
	c = wmi_put_tlv(p, TAG_START_SCAN_CMD, 100);
	c = put32(c, 0xA000 | (scans + 1));         /* scan_id */
	c = put32(c, 0xA000 | 1);                   /* scan_req_id */
	c = put32(c, 0);                            /* vdev */
	c = put32(c, 1);                            /* priority low */
	c = put32(c, 0x001 | 0x002 | 0x004 | 0x008 | 0x100 | 0x010);   /* events */
	c = put32(c, 50);                           /* dwell active */
	c = put32(c, 150);                          /* dwell passive */
	c = put32(c, 50);                           /* min rest */
	c = put32(c, 500);                          /* max rest */
	c = put32(c, 0);
	c = put32(c, 0);
	c = put32(c, 0);
	c = put32(c, 20000);                        /* max scan time */
	c = put32(c, 5);                            /* probe delay */
	c = put32(c, (0x01 | 0x10) ^ 0x20);         /* passive, chan stat; probe-req filter inverted on TLV */
	c = put32(c, 0);
	c = put32(c, NFREQ);
	c = put32(c, 1);                            /* bssids */
	c = put32(c, 0);                            /* ssids */
	c = put32(c, 0);                            /* ie_len */
	c = put32(c, 3);                            /* num_probes */
	p = c + 16;                                 /* mac addr + mask: zero */
	c = wmi_put_tlv(p, TAG_ARRAY_UINT32, NFREQ * 4);
	for (uint32_t i = 0; i < NFREQ; i++) {
		c = put32(c, scan_freqs[i]);
	}
	p = wmi_put_tlv(c, TAG_ARRAY_FIXED_STRUCT, 0);
	c = wmi_put_tlv(p, TAG_ARRAY_FIXED_STRUCT, 8);  /* bssid: broadcast */
	memset(c, 0xFF, 6);
	p = wmi_put_tlv(c + 8, TAG_ARRAY_BYTE, 0);
	return wmi_send(CMD_START_SCAN, m, (uint32_t)(p - m));
}

static void
scan_start(void)
{
	static uint8_t m[1536];
	uint8_t *p = m, *c;

	if (scan_started) {
		return;
	}
	scan_started = true;
	/* channel list, every channel passive: nothing is transmitted */
	p = put32(wmi_put_tlv(p, TAG_SCAN_CHAN_LIST_CMD, 4), NFREQ);
	p = wmi_put_tlv(p, TAG_ARRAY_STRUCT, NFREQ * (4 + 24));
	for (uint32_t i = 0; i < NFREQ; i++) {
		uint32_t mode = scan_freqs[i] < 4000 ? 1 : 0;
		uint32_t flags = mode | (1u << 7) | (1u << 11) | (scan_freqs[i] < 4000 ? 0 : (1u << 12));
		c = wmi_put_tlv(p, TAG_CHANNEL, 24);
		c = put32(c, scan_freqs[i]);
		c = put32(c, scan_freqs[i]);
		c = put32(c, 0);
		c = put32(c, flags);
		c = put32(c, (40u << 8) | (40u << 16));
		p = put32(c, 40u << 8);
	}
	bool a = wmi_send(CMD_SCAN_CHAN_LIST, m, (uint32_t)(p - m));
	uint8_t v[4 + 20] = {};
	c = wmi_put_tlv(v, TAG_VDEV_CREATE_CMD, 20);
	c = put32(c, 0);                            /* vdev 0 */
	c = put32(c, 2);                            /* station */
	c = put32(c, 0);
	memcpy(c, our_mac, 6);
	bool b = wmi_send(CMD_VDEV_CREATE, v, sizeof(v));
	scanning = scan_send_start();
	LOG("t=%u ms: scan: chan list (%u ch) %s, vdev %s, passive scan %s", g_ms, NFREQ, a ? "ok" : "FAILED",
	    b ? "ok" : "FAILED", scanning ? "requested" : "FAILED");
}

/* ---------------- HTT setup (ath10k htt.c ath10k_htt_setup, 64-bit, WCN3990), after WMI READY */

/*
 * Without the host rx ring the radio receives (pdev rx_frame grows) but no frame reaches the
 * firmware's rx path: no beacon, no scan result (topaz-woa v0.12). So, before the scan:
 * VERSION_REQ -> VERSION_CONF (or 2 s), FRAG_DESC_BANK_CFG, RX_RING_CFG (host ring of buffer
 * addresses + host-written alloc index), AGGR_CFG v2. All on the HTT endpoint (CE4).
 */
#define H2T_VERSION_REQ     0
#define H2T_RX_RING_CFG     2
#define H2T_AGGR_CFG        5
#define H2T_FRAG_DESC_BANK_CFG 6
#define T2H_VERSION_CONF    0x00
#define T2H_RX_IN_ORD_PADDR_IND 0x12
#define HTT_RING_SIZE       256
#define HTT_BUF_SIZE        2048
#define HTT_MAX_PENDING_TX  (1024 + 32)
#define HTT_EXT_DESC_SIZE   72
/* wcn3990 htt_rx_desc_v2 offsets in 4-byte words */
#define OFF_ATTENTION       1
#define OFF_FRAG_INFO       2
#define OFF_MPDU_START      4
#define OFF_MSDU_START      7
#define OFF_MSDU_END        12
#define OFF_MPDU_END        26
#define OFF_PPDU_START      27
#define OFF_PPDU_END        37
#define OFF_HDR_STATUS      74
#define OFF_MSDU_PAYLOAD    90

static uint8_t *htt_blk, *htt_bank, *htt_buf;
static uint64_t htt_ring_pa, htt_idx_pa, htt_bank_pa, htt_buf_pa;
static volatile uint64_t *htt_ring;
static volatile uint32_t *htt_idx;
static uint32_t htt_alloc, htt_state, htt_msgs, htt_msdus, htt_req_ms;

static bool htt_send(const void *d, uint32_t len) { return htt_eid != 0xFF && htc_send(htt_eid, d, len); }

static void
htt_post(uint64_t pa)
{
	*(volatile uint32_t *)(htt_buf + (pa - htt_buf_pa) + 4 * OFF_ATTENTION) = 0;
	htt_ring[htt_alloc] = pa;
	htt_alloc = (htt_alloc + 1) & (HTT_RING_SIZE - 1);
}

static void
htt_publish(void)
{
	mb();                                       /* ring entries before the index */
	*htt_idx = htt_alloc;
	mb();
}

static uint8_t *put16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); return p + 2; }
static uint8_t *put64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); return p + 8; }

static void
htt_configure(const char *why)
{
	uint8_t m[64] = {}, *p = m;

	htt_state = 2;
	/* ath10k_htt_send_frag_desc_bank_cfg_64 */
	*p++ = H2T_FRAG_DESC_BANK_CFG;
	*p++ = 0;
	*p++ = 1;                                   /* num_banks */
	*p++ = HTT_EXT_DESC_SIZE;
	p = put64(p, htt_bank_pa);
	p += 3 * 8;
	p = put16(p, 0);
	p = put16(p, HTT_MAX_PENDING_TX - 1);
	p += 3 * 4;
	p += 4 + 2 + 2;
	*p++ = 1;
	*p++ = 0;
	bool a = htt_send(m, sizeof(m));
	/* ath10k_htt_send_rx_ring_cfg_64 */
	for (uint32_t i = 0; i < HTT_RING_SIZE - 1; i++) {
		htt_post(htt_buf_pa + (uint64_t)i * HTT_BUF_SIZE);
	}
	uint8_t r[48] = {};
	p = r;
	*p++ = H2T_RX_RING_CFG;
	*p++ = 1;
	p = put16(p, 0);
	p = put64(p, htt_idx_pa);
	p = put64(p, htt_ring_pa);
	p = put16(p, HTT_RING_SIZE);
	p = put16(p, HTT_BUF_SIZE);
	p = put16(p, 0xFFFF);                       /* every frame type, management included */
	p = put16(p, 0);                            /* fw_idx_init_val */
	p = put16(p, OFF_HDR_STATUS);
	p = put16(p, OFF_MSDU_PAYLOAD);
	p = put16(p, OFF_PPDU_START);
	p = put16(p, OFF_PPDU_END);
	p = put16(p, OFF_MPDU_START);
	p = put16(p, OFF_MPDU_END);
	p = put16(p, OFF_MSDU_START);
	p = put16(p, OFF_MSDU_END);
	p = put16(p, OFF_ATTENTION);
	p = put16(p, OFF_FRAG_INFO);
	htt_publish();
	bool b = htt_send(r, sizeof(r));
	uint8_t g[4] = { H2T_AGGR_CFG, 64, 3, 0 };
	bool c = htt_send(g, sizeof(g));
	LOG("t=%u ms: htt (%s): frag bank %s, rx ring cfg %s (%u bufs), aggr cfg %s", g_ms, why, a ? "ok" : "FAILED",
	    b ? "ok" : "FAILED", HTT_RING_SIZE - 1, c ? "ok" : "FAILED");
	scan_start();
}

static void
htt_start(void)
{
	uint64_t pa = 0;
	uint32_t ring = (HTT_RING_SIZE * 8 + 64 + 0xfff) & ~0xfffu;
	uint32_t bank = (HTT_MAX_PENDING_TX * HTT_EXT_DESC_SIZE + 0xfff) & ~0xfffu;

	if (htt_state != 0) {
		return;
	}
	htt_blk = (uint8_t *)pool_alloc(ring + bank + HTT_RING_SIZE * HTT_BUF_SIZE, &pa);
	if (htt_blk == NULL) {
		LOG("htt: no memory, scanning without it");
		scan_start();
		return;
	}
	htt_ring = (volatile uint64_t *)htt_blk;
	htt_ring_pa = pa;
	htt_idx = (volatile uint32_t *)(htt_blk + HTT_RING_SIZE * 8);
	htt_idx_pa = pa + HTT_RING_SIZE * 8;
	htt_bank = htt_blk + ring;
	htt_bank_pa = pa + ring;
	htt_buf = htt_blk + ring + bank;
	htt_buf_pa = pa + ring + bank;
	htt_state = 1;
	htt_req_ms = g_ms;
	uint8_t m[4] = { H2T_VERSION_REQ, 0, 0, 0 };
	LOG("t=%u ms: htt: VERSION_REQ %s", g_ms, htt_send(m, sizeof(m)) ? "sent" : "FAILED");
}

static void
htt_rx(const uint8_t *p, uint32_t len)
{
	uint8_t type = len != 0 ? p[0] : 0xFF;
	htt_msgs++;
	if (type == T2H_VERSION_CONF && len >= 3) {
		LOG("t=%u ms: *** HTT VERSION_CONF %u.%u ***", g_ms, p[2], p[1]);
		if (htt_state == 1) {
			htt_configure("version conf");
		}
		return;
	}
	if (type == 0x07 && len >= 4) {            /* TX_COMPL_IND */
		htt_tx_compl(p, len);
		return;
	}
	if (type == T2H_RX_IN_ORD_PADDR_IND && len >= 8) {
		uint16_t n;
		memcpy(&n, p + 6, 2);
		const uint8_t *d = p + 8;
		for (uint32_t i = 0; i < n && 8 + (i + 1) * 12 <= len; i++, d += 12) {
			uint64_t pa;
			memcpy(&pa, d, 8);
			if (pa < htt_buf_pa || pa >= htt_buf_pa + (uint64_t)HTT_RING_SIZE * HTT_BUF_SIZE ||
			    ((pa - htt_buf_pa) % HTT_BUF_SIZE) != 0) {
				continue;
			}
			htt_msdus++;
			uint16_t mlen;
			memcpy(&mlen, d + 8, 2);
			if (mlen <= HTT_BUF_SIZE - 4 * OFF_MSDU_PAYLOAD) {
				w_rx_native(htt_buf + (pa - htt_buf_pa) + 4 * OFF_MSDU_PAYLOAD, mlen);
			}
			htt_post(pa);                       /* recycle the same buffer */
		}
		htt_publish();
	}
}

static void
htt_poll(void)
{
	if (htt_state == 1 && g_ms - htt_req_ms > 2000) {
		LOG("t=%u ms: htt: no VERSION_CONF in 2 s, configuring anyway", g_ms);
		htt_configure("timeout");
	}
}

static void
scan_event(const uint8_t *t, uint32_t len)
{
	uint16_t tag;
	uint32_t e[6];
	if (len < 4 + 24) {
		return;
	}
	memcpy(&tag, t + 2, 2);
	if (tag != TAG_SCAN_EVENT) {
		return;
	}
	memcpy(e, t + 4, sizeof(e));
	if (e[0] == 0x008 || e[0] == 0x100 || e[0] == 0x004) {
		return;                                 /* foreign channel / exit / bss channel */
	}
	if (e[0] == 0x001) {
		LOG("t=%u ms: scan: STARTED", g_ms);
	}
	if (e[0] == 0x002 || e[0] == 0x040 || e[0] == 0x010) {
		scanning = false;
		scans++;
		LOG("t=%u ms: *** scan %u done (event %x): %u networks, %u beacons/probe responses, %u htt msdus ***", g_ms, scans, e[0], nbss, scan_frames, htt_msdus);
		next_scan_ms = scans < 3 ? g_ms + 10000 : 0;
		api_scans_done++;
	}
}

static void
scan_mgmt_rx(const uint8_t *t, uint32_t len)
{
	uint32_t hdr[6];
	const uint8_t *f = NULL;
	uint32_t flen = 0;
	bool have_hdr = false;

	for (uint32_t o = 0; o + 4 <= len;) {
		uint16_t l, tag;
		memcpy(&l, t + o, 2);
		memcpy(&tag, t + o + 2, 2);
		if (o + 4 + l > len) {
			break;
		}
		if (tag == TAG_MGMT_RX_HDR && l >= 24 && !have_hdr) {
			memcpy(hdr, t + o + 4, sizeof(hdr));
			have_hdr = true;
		} else if (tag == TAG_ARRAY_BYTE && f == NULL) {
			f = t + o + 4;
			flen = l;
		}
		o += 4 + l;
	}
	if (!have_hdr || f == NULL) {
		return;
	}
	flen = MIN(hdr[4], flen);
	if (flen >= 24 && (f[0] & 0x0C) == 0 && (f[0] >> 4) != 8 && (f[0] >> 4) != 5) {
		w_assoc_mgmt_rx(f, flen);               /* auth, assoc response, deauth, disassoc */
		return;
	}
	if (flen < 36 || (f[0] & 0x0C) != 0 || ((f[0] >> 4) != 8 && (f[0] >> 4) != 5)) {
		return;                                 /* beacons and probe responses only */
	}
	scan_frames++;
	int32_t rssi = (int32_t)hdr[1] - 95;
	uint32_t chan = hdr[0];
	char ssid[33] = "";
	for (uint32_t o = 36; o + 2 <= flen && o + 2 + f[o + 1] <= flen; o += 2 + f[o + 1]) {
		if (f[o] == 0) {
			uint32_t n = MIN((uint32_t)f[o + 1], 32u);
			for (uint32_t i = 0; i < n; i++) {
				uint8_t ch = f[o + 2 + i];
				ssid[i] = (ch >= 0x20 && ch != 0x7F) ? (char)ch : '?';
			}
			ssid[n] = 0;
			if (n == 0 || f[o + 2] == 0) {
				strlcpy(ssid, "<hidden>", sizeof(ssid));
			}
		} else if (f[o] == 3 && f[o + 1] == 1) {
			chan = f[o + 2];
		}
	}
	struct bss *b = NULL;
	bool fresh = false;
	for (uint32_t i = 0; i < nbss; i++) {
		if (memcmp(bss[i].bssid, f + 16, 6) == 0) {
			b = &bss[i];
			break;
		}
	}
	if (b == NULL) {
		if (nbss >= 64) {
			return;
		}
		b = &bss[nbss++];
		bzero(b, sizeof(*b));
		memcpy(b->bssid, f + 16, 6);
		fresh = true;
	}
	b->seen++;
	b->rssi = (int8_t)rssi;
	b->chan = (uint8_t)chan;
	b->freq = (uint16_t)(chan == 14 ? 2484 : chan < 14 ? 2407 + 5 * chan : 5000 + 5 * chan);
	b->last_ms = g_ms;
	memcpy(&b->bintval, f + 32, 2);
	memcpy(&b->capab, f + 34, 2);
	b->ie_len = (uint16_t)MIN(flen - 36, (uint32_t)sizeof(b->ie));
	memcpy(b->ie, f + 36, b->ie_len);
	strlcpy(b->ssid, ssid, sizeof(b->ssid));
	b->raw_ssid_len = 0;
	for (uint32_t o = 36; o + 2 <= flen && o + 2 + f[o + 1] <= flen; o += 2 + f[o + 1]) {
		if (f[o] == 0) {
			b->raw_ssid_len = (uint8_t)MIN((uint32_t)f[o + 1], 32u);
			memcpy(b->raw_ssid, f + o + 2, b->raw_ssid_len);
			break;
		}
	}
	if (!fresh) {
		return;
	}
	LOG("t=%u ms: net %2u: %02x:%02x:%02x:**:**:** ch %3u %4d dBm '%s'", g_ms, nbss, b->bssid[0], b->bssid[1],
	    b->bssid[2], b->chan, b->rssi, b->ssid);
}

/* ---------------- the radio for the rest of the kernel (sys-independent C API) */

/*
 * What another part of the kernel may ask of the radio once it is up: a scan, and what the
 * scans heard (each access point with its beacon's information elements, from which a reader
 * tells the network's protection). The modem thread does the work; a caller's scan waits.
 */
extern "C" {
struct topaz_wlan_bss {
	uint8_t bssid[6];
	uint8_t ssid[32], ssid_length;
	uint16_t frequency;                         /* MHz */
	int16_t rssi;                               /* dBm */
	uint16_t beacon_interval, capability;
	uint32_t age_ms;
	uint16_t elements_length;
	uint8_t elements[768];
};
void (*topaz_wlan_ready_hook)(void);
bool topaz_wlan_ready(void);
int topaz_wlan_scan(void);
bool topaz_wlan_heard(unsigned index, struct topaz_wlan_bss *bss);
void topaz_wlan_address(uint8_t address[6]);
}

static volatile bool api_scan_asked;
static bool api_ready;

bool
topaz_wlan_ready(void)
{
	return api_ready;
}

void
topaz_wlan_address(uint8_t address[6])
{
	memcpy(address, our_mac, 6);
}

int
topaz_wlan_scan(void)
{
	if (!api_ready) {
		return ENXIO;
	}
	uint32_t before = api_scans_done;
	api_scan_asked = true;
	for (int i = 0; i < 300 && api_scans_done == before; i++) {   /* 30 s */
		IOSleep(100);
	}
	return api_scans_done != before ? 0 : ETIMEDOUT;
}

bool
topaz_wlan_heard(unsigned index, struct topaz_wlan_bss *out)
{
	if (index >= nbss) {
		return false;
	}
	const struct bss *b = &bss[index];
	bzero(out, sizeof(*out));
	memcpy(out->bssid, b->bssid, 6);
	memcpy(out->ssid, b->raw_ssid, b->raw_ssid_len);
	out->ssid_length = b->raw_ssid_len;
	out->frequency = b->freq;
	out->rssi = b->rssi;
	out->beacon_interval = b->bintval;
	out->capability = b->capab;
	out->age_ms = g_ms - b->last_ms;
	out->elements_length = MIN(b->ie_len, (uint16_t)sizeof(out->elements));
	memcpy(out->elements, b->ie, out->elements_length);
	return true;
}

/* ---------------- WPA2-PSK crypto, self-contained (SHA-1, HMAC, PBKDF2, PRF, AES-128 unwrap) */

/*
 * The handshake is done here, in the kernel: wifid hands over the passphrase as typed. Small and
 * self-contained on purpose: libkern's SHA-1/AES go through corecrypto, which this kernel may not
 * have registered.
 */
struct w_sha1 { uint32_t h[5]; uint64_t n; uint8_t b[64]; uint32_t bl; };

static inline uint32_t rol32(uint32_t v, int s) { return (v << s) | (v >> (32 - s)); }

static void
w_sha1_block(struct w_sha1 *c, const uint8_t *p)
{
	uint32_t w[80], a = c->h[0], b = c->h[1], d = c->h[2], e = c->h[3], f = c->h[4];
	for (int i = 0; i < 16; i++) {
		w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
	}
	for (int i = 16; i < 80; i++) {
		w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
	}
	for (int i = 0; i < 80; i++) {
		uint32_t k, x;
		if (i < 20) {
			x = (b & d) | (~b & e); k = 0x5A827999;
		} else if (i < 40) {
			x = b ^ d ^ e; k = 0x6ED9EBA1;
		} else if (i < 60) {
			x = (b & d) | (b & e) | (d & e); k = 0x8F1BBCDC;
		} else {
			x = b ^ d ^ e; k = 0xCA62C1D6;
		}
		uint32_t t = rol32(a, 5) + x + f + k + w[i];
		f = e; e = d; d = rol32(b, 30); b = a; a = t;
	}
	c->h[0] += a; c->h[1] += b; c->h[2] += d; c->h[3] += e; c->h[4] += f;
}

static void
w_sha1_init(struct w_sha1 *c)
{
	c->h[0] = 0x67452301; c->h[1] = 0xEFCDAB89; c->h[2] = 0x98BADCFE; c->h[3] = 0x10325476; c->h[4] = 0xC3D2E1F0;
	c->n = 0;
	c->bl = 0;
}

static void
w_sha1_update(struct w_sha1 *c, const uint8_t *p, size_t len)
{
	c->n += len;
	while (len > 0) {
		uint32_t take = (uint32_t)MIN((size_t)(64 - c->bl), len);
		memcpy(c->b + c->bl, p, take);
		c->bl += take;
		p += take;
		len -= take;
		if (c->bl == 64) {
			w_sha1_block(c, c->b);
			c->bl = 0;
		}
	}
}

static void
w_sha1_final(struct w_sha1 *c, uint8_t out[20])
{
	uint64_t bits = c->n * 8;
	uint8_t pad = 0x80, z = 0;
	w_sha1_update(c, &pad, 1);
	while (c->bl != 56) {
		w_sha1_update(c, &z, 1);
	}
	uint8_t l[8];
	for (int i = 0; i < 8; i++) {
		l[i] = (uint8_t)(bits >> (56 - 8 * i));
	}
	w_sha1_update(c, l, 8);
	for (int i = 0; i < 5; i++) {
		out[4 * i] = (uint8_t)(c->h[i] >> 24);
		out[4 * i + 1] = (uint8_t)(c->h[i] >> 16);
		out[4 * i + 2] = (uint8_t)(c->h[i] >> 8);
		out[4 * i + 3] = (uint8_t)c->h[i];
	}
}

/* HMAC-SHA1 over up to three pieces */
static void
w_hmac_sha1(const uint8_t *key, size_t klen, const uint8_t *a, size_t al, const uint8_t *b, size_t bl,
    const uint8_t *c3, size_t cl, uint8_t out[20])
{
	uint8_t k[64] = {}, ipad[64], opad[64], inner[20];
	struct w_sha1 s;
	if (klen > 64) {
		w_sha1_init(&s);
		w_sha1_update(&s, key, klen);
		w_sha1_final(&s, k);
	} else {
		memcpy(k, key, klen);
	}
	for (int i = 0; i < 64; i++) {
		ipad[i] = k[i] ^ 0x36;
		opad[i] = k[i] ^ 0x5c;
	}
	w_sha1_init(&s);
	w_sha1_update(&s, ipad, 64);
	if (al) w_sha1_update(&s, a, al);
	if (bl) w_sha1_update(&s, b, bl);
	if (cl) w_sha1_update(&s, c3, cl);
	w_sha1_final(&s, inner);
	w_sha1_init(&s);
	w_sha1_update(&s, opad, 64);
	w_sha1_update(&s, inner, 20);
	w_sha1_final(&s, out);
}

/* PMK = PBKDF2-HMAC-SHA1(passphrase, ssid, 4096, 32) */
static void
w_pbkdf2_pmk(const uint8_t *pass, size_t plen, const uint8_t *ssid, size_t slen, uint8_t pmk[32])
{
	for (uint32_t blk = 1; blk <= 2; blk++) {
		uint8_t cnt[4] = { (uint8_t)(blk >> 24), (uint8_t)(blk >> 16), (uint8_t)(blk >> 8), (uint8_t)blk };
		uint8_t u[20], t[20];
		w_hmac_sha1(pass, plen, ssid, slen, cnt, 4, NULL, 0, u);
		memcpy(t, u, 20);
		for (int i = 1; i < 4096; i++) {
			w_hmac_sha1(pass, plen, u, 20, NULL, 0, NULL, 0, u);
			for (int j = 0; j < 20; j++) {
				t[j] ^= u[j];
			}
		}
		memcpy(pmk + 20 * (blk - 1), t, blk == 1 ? 20 : 12);
	}
}

/* IEEE 802.11 PRF-SHA1 */
static void
w_prf(const uint8_t *key, size_t klen, const char *label, const uint8_t *data, size_t dlen, uint8_t *out, size_t olen)
{
	uint8_t buf[128], h[20];
	size_t ll = strlen(label);
	for (uint8_t i = 0; olen > 0; i++) {
		memcpy(buf, label, ll);
		buf[ll] = 0;
		memcpy(buf + ll + 1, data, dlen);
		buf[ll + 1 + dlen] = i;
		w_hmac_sha1(key, klen, buf, ll + 2 + dlen, NULL, 0, NULL, 0, h);
		size_t take = MIN(olen, (size_t)20);
		memcpy(out, h, take);
		out += take;
		olen -= take;
	}
}

/* AES-128 decryption of one block (FIPS-197), for the key unwrap of the GTK */
static const uint8_t aes_sbox[256] = {
	0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
	0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
	0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
	0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
	0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
	0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
	0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
	0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static uint8_t aes_xt(uint8_t x) { return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0)); }
static uint8_t aes_mul(uint8_t a, uint8_t b) { uint8_t r = 0; while (b) { if (b & 1) r ^= a; a = aes_xt(a); b >>= 1; } return r; }

static void
aes128_expand(const uint8_t key[16], uint8_t rk[176])
{
	uint8_t rcon = 1;
	memcpy(rk, key, 16);
	for (int i = 16; i < 176; i += 4) {
		uint8_t t[4];
		memcpy(t, rk + i - 4, 4);
		if (i % 16 == 0) {
			uint8_t x = t[0];
			t[0] = aes_sbox[t[1]] ^ rcon; t[1] = aes_sbox[t[2]]; t[2] = aes_sbox[t[3]]; t[3] = aes_sbox[x];
			rcon = aes_xt(rcon);
		}
		for (int j = 0; j < 4; j++) {
			rk[i + j] = rk[i - 16 + j] ^ t[j];
		}
	}
}

static void
aes128_decrypt(const uint8_t rk[176], uint8_t s[16])
{
	uint8_t inv[256];
	for (int i = 0; i < 256; i++) {
		inv[aes_sbox[i]] = (uint8_t)i;
	}
	for (int j = 0; j < 16; j++) {
		s[j] ^= rk[160 + j];
	}
	for (int r = 9; r >= 0; r--) {
		uint8_t t[16];
		for (int c = 0; c < 4; c++) {           /* inverse shift rows + inverse sub bytes */
			for (int row = 0; row < 4; row++) {
				t[4 * ((c + row) % 4) + row] = inv[s[4 * c + row]];
			}
		}
		for (int j = 0; j < 16; j++) {
			t[j] ^= rk[16 * r + j];
		}
		if (r == 0) {
			memcpy(s, t, 16);
			break;
		}
		for (int c = 0; c < 4; c++) {           /* inverse mix columns */
			uint8_t *q = t + 4 * c, a0 = q[0], a1 = q[1], a2 = q[2], a3 = q[3];
			s[4 * c] = aes_mul(a0, 14) ^ aes_mul(a1, 11) ^ aes_mul(a2, 13) ^ aes_mul(a3, 9);
			s[4 * c + 1] = aes_mul(a0, 9) ^ aes_mul(a1, 14) ^ aes_mul(a2, 11) ^ aes_mul(a3, 13);
			s[4 * c + 2] = aes_mul(a0, 13) ^ aes_mul(a1, 9) ^ aes_mul(a2, 14) ^ aes_mul(a3, 11);
			s[4 * c + 3] = aes_mul(a0, 11) ^ aes_mul(a1, 13) ^ aes_mul(a2, 9) ^ aes_mul(a3, 14);
		}
	}
}

/* RFC 3394 key unwrap with a 128-bit KEK; out gets len-8 bytes. False if the check value is wrong. */
static bool
aes_unwrap(const uint8_t kek[16], const uint8_t *in, size_t len, uint8_t *out)
{
	uint8_t rk[176], a[8], b[16];
	size_t n = len / 8 - 1;
	if (len < 24 || len % 8 != 0) {
		return false;
	}
	aes128_expand(kek, rk);
	memcpy(a, in, 8);
	memcpy(out, in + 8, len - 8);
	for (int j = 5; j >= 0; j--) {
		for (size_t i = n; i >= 1; i--) {
			uint64_t t = (uint64_t)n * (uint64_t)j + i;
			memcpy(b, a, 8);
			for (int k = 0; k < 8; k++) {
				b[7 - k] ^= (uint8_t)(t >> (8 * k));
			}
			memcpy(b + 8, out + 8 * (i - 1), 8);
			aes128_decrypt(rk, b);
			memcpy(a, b, 8);
			memcpy(out + 8 * (i - 1), b + 8, 8);
		}
	}
	for (int k = 0; k < 8; k++) {
		if (a[k] != 0xA6) {
			return false;
		}
	}
	return true;
}

static bool
w_crypto_selftest(void)
{
	static const uint8_t kek[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	static const uint8_t ct[24] = { 0x1F,0xA6,0x8B,0x0A,0x81,0x12,0xB4,0x47,0xAE,0xF3,0x4B,0xD8,0xFB,0x5A,0x7B,0x82,
		0x9D,0x3E,0x86,0x23,0x71,0xD2,0xCF,0xE5 };
	static const uint8_t pt[16] = { 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF };
	static const uint8_t pmk_ref[4] = { 0xf4, 0x2c, 0x6f, 0xc5 };     /* "password"/"IEEE" (802.11 annex) */
	uint8_t out[16], pmk[32];
	bool a = aes_unwrap(kek, ct, 24, out) && memcmp(out, pt, 16) == 0;
	w_pbkdf2_pmk((const uint8_t *)"password", 8, (const uint8_t *)"IEEE", 4, pmk);
	bool b = memcmp(pmk, pmk_ref, 4) == 0;
	LOG("wpa: crypto self-test: key unwrap %s, pbkdf2 %s", a ? "ok" : "FAILED", b ? "ok" : "FAILED");
	return a && b;
}

/* ---------------- station: HTT data TX, association, WPA2-PSK, Ethernet bridge */

/*
 * Port of topaz-woa assoc.c and the TX half of htt.c, plus the 4-way handshake that Windows did
 * there: VDEV_START on the AP's channel, PEER_CREATE, open-system AUTH and ASSOC REQ as WMI
 * management frames, PEER_ASSOC + VDEV_UP, then EAPOL over the data path, keys by
 * VDEV_INSTALL_KEY. Frames to and from the rest of the kernel are Ethernet; the radio's are
 * native wifi (802.11 header without QoS + LLC/SNAP).
 */
#define CMD_VDEV_START          0x5003
#define CMD_VDEV_UP             0x5005
#define CMD_VDEV_STOP           0x5006
#define CMD_VDEV_DOWN           0x5007
#define CMD_VDEV_INSTALL_KEY    0x5009
#define CMD_PEER_CREATE         0x6001
#define CMD_PEER_DELETE         0x6002
#define CMD_PEER_SET_PARAM      0x6004
#define CMD_PEER_ASSOC          0x6005
#define CMD_MGMT_TX_SEND        0x7008
#define CMD_STA_PS_MODE         0x9001
#define TAG_VDEV_START          0x58
#define TAG_VDEV_UP             0x5c
#define TAG_VDEV_STOP           0x5d
#define TAG_VDEV_DOWN           0x5e
#define TAG_VDEV_INSTALL_KEY    0x60
#define TAG_PEER_CREATE         0x61
#define TAG_PEER_DELETE         0x62
#define TAG_PEER_SET_PARAM      0x64
#define TAG_PEER_ASSOC          0x65
#define TAG_VHT_RATE_SET        0x66
#define TAG_STA_PS_MODE         0x6c
#define TAG_MGMT_TX             0x1a6
#define PEER_FLAG_AUTH          0x1
#define PEER_FLAG_NEED_PTK_4WAY 0x4
#define PEER_PARAM_AUTHORIZE    3
#define WMI_CIPHER_TKIP         2
#define WMI_CIPHER_CCMP         4
#define FC_ASSOC_REQ            0x00
#define FC_ASSOC_RESP           0x10
#define FC_DISASSOC             0xA0
#define FC_AUTH                 0xB0
#define FC_DEAUTH               0xC0

enum sta_state { S_IDLE, S_START, S_AUTH, S_ASSOC, S_4WAY, S_UP };
static volatile enum sta_state s_state;
static volatile int s_result;                   /* of the last join: 0 joined, else an errno (EAUTH: passphrase) */
static volatile bool s_join_asked, s_leave_asked;
static volatile uint32_t s_join_done;           /* bumped when a join ends either way */
static uint8_t s_ssid[32], s_ssid_len, s_bssid[6], s_pmk[32];
static uint8_t s_body[768];                     /* the AP's beacon IEs */
static uint32_t s_body_len, s_freq, s_aid, s_tries, s_deadline, s_desc, s_msg1s;
static bool s_rsn, s_start_resp, s_ptk_set;
static int32_t s_start_status;
static uint8_t s_assoc_req[512];
static uint32_t s_assoc_req_len;
static uint8_t s_rsn_ie[22];
static uint8_t s_ptk[48], s_anonce[32], s_snonce[32], s_group_cipher;
static uint64_t s_replay;
static uint8_t *s_mgmt_va;
static uint64_t s_mgmt_pa;

void (*topaz_wlan_rx_hook)(const uint8_t *frame, size_t length);
void (*topaz_wlan_link_hook)(bool up);

static uint8_t *w_p32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); return p + 4; }
static uint8_t *w_pmac(uint8_t *p, const uint8_t *mac) { bzero(p, 8); memcpy(p, mac, 6); return p + 8; }

static uint8_t *
w_channel(uint8_t *p, uint32_t freq)
{
	uint8_t *c = wmi_put_tlv(p, TAG_CHANNEL, 24);
	c = w_p32(c, freq);
	c = w_p32(c, freq);
	c = w_p32(c, 0);
	c = w_p32(c, freq < 4000 ? 1 : 0);
	c = w_p32(c, (40u << 8) | (40u << 16));
	return w_p32(c, 40u << 8);
}

static const uint8_t *
w_find_ie(const uint8_t *ies, uint32_t len, uint8_t id)
{
	for (uint32_t o = 0; o + 2 <= len && o + 2 + ies[o + 1] <= len; o += 2 + ies[o + 1]) {
		if (ies[o] == id) {
			return ies + o;
		}
	}
	return NULL;
}

/* ---- HTT data TX (ath10k_htt_tx_64, native wifi) */
#define TX_SLOTS        64
#define TX_SLOT_SIZE    2048
#define H2T_TX_FRM      1
#define TXF0_MAC_HDR    0x01
#define TXF0_NO_ENCRYPT 0x04
#define TXF0_NATIVE     (1u << 5)
#define TXF1_TID_NONQOS (16u << 6)
#define TXF1_POSTPONED  (1u << 11)
#define PREFETCH_LEN    50
#define T2H_TX_COMPL_IND 0x07

static uint8_t *tx_pool;
static uint64_t tx_pool_pa, tx_busy;
static uint32_t tx_sent, tx_done, tx_fail, tx_drop, rx_up;

static bool
htt_tx_frame(const uint8_t *f, uint32_t len, bool clear)
{
	uint8_t m[20 + 52], *p = m;
	uint32_t id, pf = (MIN(len, (uint32_t)PREFETCH_LEN) + 3) & ~3u;
	if (tx_pool == NULL) {
		tx_pool = (uint8_t *)pool_alloc(TX_SLOTS * TX_SLOT_SIZE, &tx_pool_pa);
	}
	if (tx_pool == NULL || htt_bank == NULL || htt_state != 2 || len > TX_SLOT_SIZE) {
		return false;
	}
	for (id = 0; id < TX_SLOTS && (tx_busy & (1ull << id)) != 0; id++) {
	}
	if (id == TX_SLOTS) {
		return false;
	}
	uint64_t pa = tx_pool_pa + (uint64_t)id * TX_SLOT_SIZE;
	memcpy(tx_pool + id * TX_SLOT_SIZE, f, len);
	uint8_t *ext = htt_bank + id * HTT_EXT_DESC_SIZE;  /* frags[0] at +24: paddr_lo, paddr_hi16, len16 */
	bzero(ext, HTT_EXT_DESC_SIZE);
	uint32_t lo = (uint32_t)pa;
	uint16_t hi = (uint16_t)(pa >> 32), l16 = (uint16_t)len;
	memcpy(ext + 24, &lo, 4);
	memcpy(ext + 28, &hi, 2);
	memcpy(ext + 30, &l16, 2);
	mb();
	bzero(m, sizeof(m));
	*p++ = H2T_TX_FRM;
	*p++ = (uint8_t)(TXF0_MAC_HDR | TXF0_NATIVE | (clear ? TXF0_NO_ENCRYPT : 0));
	p = put16(p, (uint16_t)(TXF1_TID_NONQOS | TXF1_POSTPONED));   /* vdev 0 */
	p = put16(p, (uint16_t)len);
	p = put16(p, (uint16_t)id);
	p = put64(p, htt_bank_pa + (uint64_t)id * HTT_EXT_DESC_SIZE);
	p = w_p32(p, 0xFFFF);                       /* peer id: invalid */
	memcpy(p, f, MIN(len, pf));
	if (!htt_send(m, 20 + pf)) {
		return false;
	}
	tx_busy |= 1ull << id;
	tx_sent++;
	return true;
}

static void
htt_tx_compl(const uint8_t *p, uint32_t len)
{
	uint32_t n = p[2], st = p[1] & 7;
	for (uint32_t i = 0; i < n && 4 + 2 * (i + 1) <= len; i++) {
		uint16_t id;
		memcpy(&id, p + 4 + 2 * i, 2);
		if (id < TX_SLOTS) {
			tx_busy &= ~(1ull << id);
		}
	}
	tx_done += n;
	if (st != 0) {
		tx_fail += n;
		if (tx_fail <= 4) {
			LOG("t=%u ms: htt tx: %u frame(s) status %u", g_ms, n, st);
		}
	}
}

/* ---- WMI commands for the join (layouts: ath10k wmi-tlv) */
static bool
w_vdev_start(uint32_t freq, uint32_t bcn, uint32_t dtim)
{
	uint8_t m[4 + 72 + 4 + 24 + 4] = {}, *p;
	p = wmi_put_tlv(m, TAG_VDEV_START, 72);
	p = w_p32(p, 0);
	p = w_p32(p, 0);
	p = w_p32(p, bcn);
	p = w_p32(p, dtim);
	p = w_p32(p, 0);
	p += 36 + 16;                               /* ssid (empty for a station), tx rate/power, noa, hw ack */
	p = w_channel(p, freq);
	wmi_put_tlv(p, TAG_ARRAY_STRUCT, 0);
	return wmi_send(CMD_VDEV_START, m, sizeof(m));
}

static bool
w_vdev_simple(uint32_t cmd, uint16_t tag)
{
	uint8_t m[8], *p = wmi_put_tlv(m, tag, 4);
	w_p32(p, 0);
	return wmi_send(cmd, m, sizeof(m));
}

static bool
w_peer(uint32_t cmd, uint16_t tag, bool create)
{
	uint8_t m[4 + 16] = {}, *p = wmi_put_tlv(m, tag, create ? 16 : 12);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	if (create) {
		w_p32(p, 0);
	}
	return wmi_send(cmd, m, create ? 20 : 16);
}

static bool
w_peer_param(uint32_t param, uint32_t value)
{
	uint8_t m[4 + 20], *p = wmi_put_tlv(m, TAG_PEER_SET_PARAM, 20);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, param);
	w_p32(p, value);
	return wmi_send(CMD_PEER_SET_PARAM, m, sizeof(m));
}

static bool
w_ps_off(void)
{
	uint8_t m[12], *p = wmi_put_tlv(m, TAG_STA_PS_MODE, 8);
	p = w_p32(p, 0);
	w_p32(p, 0);
	return wmi_send(CMD_STA_PS_MODE, m, sizeof(m));
}

static bool
w_mgmt_tx(const uint8_t *f, uint32_t len)
{
	uint8_t m[4 + 28 + 4 + 64] = {}, *p;
	uint32_t buf = (MIN(len, 64u) + 3) & ~3u;
	if (s_mgmt_va == NULL) {
		s_mgmt_va = (uint8_t *)pool_alloc(0x1000, &s_mgmt_pa);
	}
	if (s_mgmt_va == NULL || len > 0x1000 - 8) {
		return false;
	}
	bzero(s_mgmt_va, ((len + 3) & ~3u) + 4);
	memcpy(s_mgmt_va, f, len);
	mb();
	p = wmi_put_tlv(m, TAG_MGMT_TX, 28);
	p = w_p32(p, 0);
	p = w_p32(p, ++s_desc);
	p = w_p32(p, 0);
	p = put64(p, s_mgmt_pa);
	p = w_p32(p, len);
	p = w_p32(p, buf);
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, buf);
	memcpy(p, s_mgmt_va, buf);
	return wmi_send(CMD_MGMT_TX_SEND, m, 4 + 28 + 4 + buf);
}

static uint32_t
w_ap_rates(uint8_t *out)
{
	static const uint8_t ids[2] = { 1, 50 };
	uint32_t n = 0;
	for (int k = 0; k < 2; k++) {
		const uint8_t *ie = w_find_ie(s_body, s_body_len, ids[k]);
		for (uint32_t i = 0; ie != NULL && i < ie[1] && n < 16; i++) {
			out[n++] = ie[2 + i];
		}
	}
	return n;
}

static bool
w_peer_assoc(uint32_t caps)
{
	uint8_t m[4 + 76 + 4 + 16 + 4 + 4 + 16] = {}, *p, *r, rates[16];
	uint32_t n = w_ap_rates(rates), rl = (MIN(n, 16u) + 3) & ~3u;
	p = wmi_put_tlv(m, TAG_PEER_ASSOC, 76);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, 0);
	p = w_p32(p, 1);                            /* new assoc */
	p = w_p32(p, s_aid);
	p = w_p32(p, PEER_FLAG_AUTH | (s_rsn ? PEER_FLAG_NEED_PTK_4WAY : 0));
	p = w_p32(p, caps);
	p = w_p32(p, 1);                            /* listen interval */
	p += 4 * 6;                                 /* ht caps, max mpdu, density, rate caps; nss below */
	p -= 4 * 2;
	p = w_p32(p, 1);                            /* nss */
	p = w_p32(p, 0);                            /* vht caps */
	p = w_p32(p, s_freq < 4000 ? 1 : 0);        /* phy mode: 11g / 11a */
	p += 8;
	p = w_p32(p, MIN(n, 16u));
	p = w_p32(p, 0);
	r = wmi_put_tlv(p, TAG_ARRAY_BYTE, rl);
	for (uint32_t i = 0; i < n && i < 16; i++) {
		uint8_t v = rates[i] & 0x7F;
		r[i] = (v == 2 || v == 4 || v == 11 || v == 22) ? (uint8_t)(v | 0x80) : v;
	}
	p = r + rl;
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, 0);
	wmi_put_tlv(p, TAG_VHT_RATE_SET, 16);
	return wmi_send(CMD_PEER_ASSOC, m, 4 + 76 + 4 + rl + 4 + 4 + 16);
}

static bool
w_install_key(bool group, uint32_t idx, uint32_t cipher, const uint8_t *key, uint32_t klen)
{
	uint8_t m[4 + 92 + 4 + 32] = {}, *p;
	uint32_t kl = (klen + 3) & ~3u;
	p = wmi_put_tlv(m, TAG_VDEV_INSTALL_KEY, 92);
	p = w_p32(p, 0);
	p = w_pmac(p, s_bssid);
	p = w_p32(p, idx);
	p = w_p32(p, group ? 1 : 0);
	p = w_p32(p, cipher);
	p += 3 * 8 + 16 + 16;
	p = w_p32(p, klen);
	p = w_p32(p, cipher == WMI_CIPHER_TKIP ? 8 : 0);
	p = w_p32(p, cipher == WMI_CIPHER_TKIP ? 8 : 0);
	p = wmi_put_tlv(p, TAG_ARRAY_BYTE, kl);
	memcpy(p, key, klen);
	return wmi_send(CMD_VDEV_INSTALL_KEY, m, 4 + 92 + 4 + kl);
}

/* ---- 802.11 management frames */
static uint8_t *
w_mgmt_hdr(uint8_t *f, uint8_t fc)
{
	bzero(f, 24);
	f[0] = fc;
	memcpy(f + 4, s_bssid, 6);
	memcpy(f + 10, our_mac, 6);
	memcpy(f + 16, s_bssid, 6);
	return f + 24;
}

static bool
w_send_auth(void)
{
	uint8_t f[30], *b = w_mgmt_hdr(f, FC_AUTH);
	bzero(b, 6);
	b[2] = 1;                                   /* open system, transaction 1 */
	return w_mgmt_tx(f, sizeof(f));
}

static void
w_build_assoc_req(void)
{
	uint8_t *f = s_assoc_req, *p = w_mgmt_hdr(f, FC_ASSOC_REQ), rates[16];
	uint32_t n = w_ap_rates(rates), cap = 0x0001 | 0x0020 | 0x0400;   /* ESS, short preamble, short slot */
	if (s_rsn) {
		cap |= 0x0010;
	}
	*p++ = (uint8_t)cap;
	*p++ = (uint8_t)(cap >> 8);
	*p++ = 10;
	*p++ = 0;
	*p++ = 0;
	*p++ = s_ssid_len;
	memcpy(p, s_ssid, s_ssid_len);
	p += s_ssid_len;
	*p++ = 1;
	*p++ = (uint8_t)MIN(n, 8u);
	for (uint32_t k = 0; k < n && k < 8; k++) {
		*p++ = rates[k];
	}
	if (n > 8) {
		*p++ = 50;
		*p++ = (uint8_t)(n - 8);
		for (uint32_t k = 8; k < n; k++) {
			*p++ = rates[k];
		}
	}
	if (s_rsn) {
		memcpy(p, s_rsn_ie, sizeof(s_rsn_ie));
		p += sizeof(s_rsn_ie);
	}
	s_assoc_req_len = (uint32_t)(p - f);
}

/* ---- Ethernet <-> native wifi */
static void
w_link(bool up)
{
	if (topaz_wlan_link_hook != NULL) {
		topaz_wlan_link_hook(up);
	}
}

/* an Ethernet frame (dst, src, type, payload) to the AP as a to-DS data frame */
static bool
w_tx_ether(const uint8_t *e, size_t len, bool clear)
{
	uint8_t f[TX_SLOT_SIZE];
	if (len < 14 || len - 14 + 24 + 8 > sizeof(f)) {
		return false;
	}
	bzero(f, 24);
	f[0] = 0x08;                                /* data */
	f[1] = (uint8_t)(0x01 | (s_ptk_set && !clear ? 0x40 : 0));   /* to DS (+ protected once keyed) */
	memcpy(f + 4, s_bssid, 6);                  /* addr1: the AP */
	memcpy(f + 10, our_mac, 6);                 /* addr2: us */
	memcpy(f + 16, e, 6);                       /* addr3: the destination */
	static const uint8_t snap[6] = { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00 };
	memcpy(f + 24, snap, 6);
	memcpy(f + 30, e + 12, 2);                  /* ethertype */
	memcpy(f + 32, e + 14, len - 14);
	return htt_tx_frame(f, (uint32_t)(len - 14 + 32), clear);
}

/* ---- WPA2 4-way handshake (EAPOL-Key, descriptor version 2: HMAC-SHA1-128 MIC, AES key wrap) */
static void w_join_end(int result);

static void
w_eapol_send(uint16_t info, const uint8_t *nonce, const uint8_t *data, uint16_t dlen)
{
	uint8_t e[14 + 4 + 95 + 64] = {}, *k = e + 14 + 4, mic[20];
	memcpy(e, s_bssid, 6);
	memcpy(e + 6, our_mac, 6);
	e[12] = 0x88;
	e[13] = 0x8E;
	uint16_t body = (uint16_t)(95 + dlen);
	e[14] = 1;                                  /* EAPOL version */
	e[15] = 3;                                  /* key */
	e[16] = (uint8_t)(body >> 8);
	e[17] = (uint8_t)body;
	k[0] = 2;                                   /* RSN key descriptor */
	k[1] = (uint8_t)(info >> 8);
	k[2] = (uint8_t)info;
	k[3] = 0;                                   /* key length 0 (from the STA) */
	k[4] = 0;
	for (int i = 0; i < 8; i++) {
		k[5 + i] = (uint8_t)(s_replay >> (56 - 8 * i));
	}
	if (nonce != NULL) {
		memcpy(k + 13, nonce, 32);
	}
	k[93] = (uint8_t)(dlen >> 8);
	k[94] = (uint8_t)dlen;
	if (dlen) {
		memcpy(k + 95, data, dlen);
	}
	w_hmac_sha1(s_ptk, 16, e + 14, 4 + body, NULL, 0, NULL, 0, mic);
	memcpy(k + 77, mic, 16);
	(void)w_tx_ether(e, 14 + 4 + body, true);
}

static bool
w_eapol_mic_ok(uint8_t *eapol, size_t len)
{
	uint8_t want[16], got[20];
	if (len < 4 + 95) {
		return false;
	}
	memcpy(want, eapol + 4 + 77, 16);
	bzero(eapol + 4 + 77, 16);
	w_hmac_sha1(s_ptk, 16, eapol, len, NULL, 0, NULL, 0, got);
	memcpy(eapol + 4 + 77, want, 16);
	return memcmp(want, got, 16) == 0;
}

static void
w_eapol_rx(const uint8_t *e, size_t elen)
{
	uint8_t frame[512];
	const uint8_t *src = e + 6;
	if (s_state != S_4WAY || elen < 14 + 4 + 95 || elen - 14 > sizeof(frame) || memcmp(src, s_bssid, 6) != 0) {
		return;
	}
	size_t len = elen - 14;
	memcpy(frame, e + 14, len);
	uint8_t *k = frame + 4;
	if (frame[1] != 3 || k[0] != 2) {
		return;
	}
	uint16_t info = (uint16_t)(k[1] << 8 | k[2]), dlen = (uint16_t)(k[93] << 8 | k[94]);
	uint64_t replay = 0;
	for (int i = 0; i < 8; i++) {
		replay = replay << 8 | k[5 + i];
	}
	if (95 + (size_t)dlen > len - 4) {
		return;
	}
	bool pairwise = info & 0x0008, ack = info & 0x0080, has_mic = info & 0x0100, install = info & 0x0040;
	if (pairwise && ack && !has_mic) {
		/* message 1: ANonce -> PTK, answer with message 2 */
		if (++s_msg1s > 3) {
			LOG("t=%u ms: wpa: the AP keeps sending message 1: wrong passphrase", g_ms);
			w_join_end(EAUTH);
			return;
		}
		s_replay = replay;
		memcpy(s_anonce, k + 13, 32);
		read_random(s_snonce, 32);
		uint8_t data[76];
		const uint8_t *aa = s_bssid, *spa = our_mac;
		bool amin = memcmp(aa, spa, 6) < 0, nmin = memcmp(s_anonce, s_snonce, 32) < 0;
		memcpy(data, amin ? aa : spa, 6);
		memcpy(data + 6, amin ? spa : aa, 6);
		memcpy(data + 12, nmin ? s_anonce : s_snonce, 32);
		memcpy(data + 44, nmin ? s_snonce : s_anonce, 32);
		w_prf(s_pmk, 32, "Pairwise key expansion", data, sizeof(data), s_ptk, 48);
		w_eapol_send(0x010A, s_snonce, s_rsn_ie, sizeof(s_rsn_ie));
		LOG("t=%u ms: wpa: message 1 -> message 2 sent", g_ms);
		return;
	}
	if (pairwise && ack && has_mic && install) {
		/* message 3: check, unwrap the GTK, answer with message 4, install the keys */
		if (!w_eapol_mic_ok(frame, len)) {
			LOG("t=%u ms: wpa: message 3 MIC wrong: wrong passphrase", g_ms);
			w_join_end(EAUTH);
			return;
		}
		s_replay = replay;
		uint8_t plain[256];
		uint8_t gtk[32], gtk_len = 0, gtk_idx = 1;
		if ((info & 0x1000) && dlen >= 24 && dlen <= sizeof(plain) + 8 && aes_unwrap(s_ptk + 16, k + 95, dlen, plain)) {
			for (uint32_t o = 0; o + 2 <= (uint32_t)dlen - 8 && o + 2 + plain[o + 1] <= (uint32_t)dlen - 8; o += 2 + plain[o + 1]) {
				if (plain[o] == 0xDD && plain[o + 1] >= 6 && plain[o + 2] == 0x00 && plain[o + 3] == 0x0F &&
				    plain[o + 4] == 0xAC && plain[o + 5] == 0x01) {
					gtk_idx = plain[o + 6] & 3;
					gtk_len = (uint8_t)MIN((uint32_t)plain[o + 1] - 6, 32u);
					memcpy(gtk, plain + o + 8, gtk_len);
				}
			}
		}
		w_eapol_send(0x030A, NULL, NULL, 0);
		IOSleep(5);                             /* message 4 out in the clear before the keys go in */
		w_install_key(false, 0, WMI_CIPHER_CCMP, s_ptk + 32, 16);
		if (gtk_len != 0) {
			w_install_key(true, gtk_idx, s_group_cipher == 2 ? WMI_CIPHER_TKIP : WMI_CIPHER_CCMP, gtk, gtk_len);
		}
		w_peer_param(PEER_PARAM_AUTHORIZE, 1);
		s_ptk_set = true;
		bzero(gtk, sizeof(gtk));
		LOG("t=%u ms: *** WPA2 keys installed (group key %s, idx %u) ***", g_ms, gtk_len ? "yes" : "NO", gtk_idx);
		w_join_end(0);
		return;
	}
}

/* ---- the join, step by step from the modem thread */
static void
w_teardown(void)
{
	if (s_state >= S_START) {
		if (s_state >= S_4WAY) {
			w_vdev_simple(CMD_VDEV_DOWN, TAG_VDEV_DOWN);
		}
		if (s_state >= S_AUTH) {
			w_peer(CMD_PEER_DELETE, TAG_PEER_DELETE, false);
		}
		w_vdev_simple(CMD_VDEV_STOP, TAG_VDEV_STOP);
	}
	if (s_state == S_UP) {
		w_link(false);
	}
	s_state = S_IDLE;
	s_ptk_set = false;
}

static void
w_join_end(int result)
{
	if (result != 0) {
		LOG("t=%u ms: join: failed (%d) in state %u", g_ms, result, s_state);
		w_teardown();
	} else {
		s_state = S_UP;
		LOG("t=%u ms: *** joined %.*s ***", g_ms, (int)s_ssid_len, s_ssid);
		w_link(true);
	}
	bzero(s_pmk, sizeof(s_pmk));
	s_result = result;
	s_join_done++;
}

static void
w_join_begin(void)
{
	/* the AP: the strongest one heard with that name (or the BSSID asked for) */
	int best = -1;
	for (uint32_t i = 0; i < nbss; i++) {
		if (bss[i].raw_ssid_len == s_ssid_len && memcmp(bss[i].raw_ssid, s_ssid, s_ssid_len) == 0 &&
		    (best < 0 || bss[i].rssi > bss[best].rssi)) {
			best = (int)i;
		}
	}
	bool want_bssid = false;
	for (int i = 0; i < 6; i++) {
		want_bssid |= s_bssid[i] != 0;
	}
	if (want_bssid) {
		for (uint32_t i = 0; i < nbss; i++) {
			if (memcmp(bss[i].bssid, s_bssid, 6) == 0) {
				best = (int)i;
			}
		}
	}
	if (best < 0) {
		LOG("join: no such network heard");
		w_join_end(ENOENT);
		return;
	}
	memcpy(s_bssid, bss[best].bssid, 6);
	s_freq = bss[best].freq;
	s_body_len = bss[best].ie_len;
	memcpy(s_body, bss[best].ie, s_body_len);
	const uint8_t *rsn = w_find_ie(s_body, s_body_len, 48);
	s_rsn = rsn != NULL;
	s_group_cipher = (rsn != NULL && rsn[1] >= 8) ? rsn[2 + 2 + 3] : 4;
	static const uint8_t hdr[4] = { 48, 20, 1, 0 };
	memcpy(s_rsn_ie, hdr, 4);
	uint8_t *q = s_rsn_ie + 4;
	*q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = s_group_cipher;
	*q++ = 1; *q++ = 0; *q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = 4;   /* pairwise CCMP */
	*q++ = 1; *q++ = 0; *q++ = 0x00; *q++ = 0x0F; *q++ = 0xAC; *q++ = 2;   /* AKM PSK */
	*q++ = 0; *q++ = 0;
	uint32_t bcn = bss[best].bintval ? bss[best].bintval : 100, dtim = 1;
	const uint8_t *tim = w_find_ie(s_body, s_body_len, 5);
	if (tim != NULL && tim[1] >= 2 && tim[3] != 0) {
		dtim = tim[3];
	}
	s_msg1s = 0;
	s_start_resp = false;
	LOG("t=%u ms: join: %.*s %02x:%02x:%02x:**:**:** %u MHz %s", g_ms, (int)s_ssid_len, s_ssid, s_bssid[0], s_bssid[1],
	    s_bssid[2], s_freq, s_rsn ? "WPA2" : "open");
	w_ps_off();
	if (!w_vdev_start(s_freq, bcn, dtim)) {
		w_join_end(EIO);
		return;
	}
	s_state = S_START;
	s_deadline = g_ms + 1500;
}

static void
w_assoc_mgmt_rx(const uint8_t *f, uint32_t len)
{
	if (len < 26 || s_state == S_IDLE || memcmp(f + 10, s_bssid, 6) != 0) {
		return;
	}
	uint8_t fc = f[0];
	uint16_t v = (uint16_t)(f[24] | f[25] << 8);
	if (fc == FC_AUTH && s_state == S_AUTH && len >= 30 && f[26] == 2) {
		v = (uint16_t)(f[28] | f[29] << 8);
		LOG("t=%u ms: join: auth reply status %u", g_ms, v);
		if (v != 0) {
			w_join_end(ECONNREFUSED);
			return;
		}
		s_state = S_ASSOC;
		s_tries = 1;
		w_build_assoc_req();
		w_mgmt_tx(s_assoc_req, s_assoc_req_len);
		s_deadline = g_ms + 400;
	} else if (fc == FC_ASSOC_RESP && s_state == S_ASSOC && len >= 30) {
		v = (uint16_t)(f[26] | f[27] << 8);
		s_aid = (uint32_t)(f[28] | f[29] << 8) & 0x3FFF;
		LOG("t=%u ms: join: assoc reply status %u aid %u", g_ms, v, s_aid);
		if (v != 0) {
			w_join_end(ECONNREFUSED);
			return;
		}
		w_peer_assoc((uint32_t)(f[24] | f[25] << 8));
		uint8_t m[20], *p = wmi_put_tlv(m, TAG_VDEV_UP, 16);
		p = w_p32(p, 0);
		p = w_p32(p, s_aid);
		w_pmac(p, s_bssid);
		wmi_send(CMD_VDEV_UP, m, sizeof(m));
		if (!s_rsn) {
			w_peer_param(PEER_PARAM_AUTHORIZE, 1);
			s_state = S_4WAY;                   /* (teardown order) */
			w_join_end(0);
		} else {
			s_state = S_4WAY;
			s_deadline = g_ms + 8000;
			LOG("t=%u ms: join: associated, waiting for the 4-way handshake", g_ms);
		}
	} else if (fc == FC_DEAUTH || fc == FC_DISASSOC) {
		LOG("t=%u ms: join: the AP ended the link, reason %u (state %u)", g_ms, v, s_state);
		if (s_state == S_UP) {
			w_teardown();
		} else {
			w_join_end(s_state == S_4WAY ? EAUTH : ECONNREFUSED);
		}
	}
}

static void
w_join_event(uint32_t id, const uint8_t *t, uint32_t len)
{
	uint16_t l = 0;
	uint32_t v[4] = {};
	if (len >= 4) {
		memcpy(&l, t, 2);
	}
	if (len < 4u + l) {
		return;
	}
	memcpy(v, t + 4, MIN((uint32_t)l, (uint32_t)sizeof(v)));
	if (id == 0x5001 && l >= 16) {             /* VDEV_START_RESP {vdev, req, type, status} */
		s_start_status = (int32_t)v[3];
		s_start_resp = true;
	} else if (id == 0x6001 && s_state == S_UP) {   /* PEER_STA_KICKOUT */
		LOG("t=%u ms: join: peer kickout", g_ms);
		w_teardown();
	}
}

/* every pass of the modem loop */
static void
w_join_poll(void)
{
	if (s_leave_asked) {
		s_leave_asked = false;
		w_teardown();
	}
	if (s_join_asked && !scanning) {
		s_join_asked = false;
		w_teardown();
		w_join_begin();
		return;
	}
	if (s_state == S_START) {
		if (s_start_resp) {
			if (s_start_status != 0) {
				w_join_end(EIO);
				return;
			}
			s_state = S_AUTH;
			s_tries = 1;
			w_peer(CMD_PEER_CREATE, TAG_PEER_CREATE, true);
			w_send_auth();
			s_deadline = g_ms + 400;
		} else if (g_ms > s_deadline) {
			w_join_end(ETIMEDOUT);
		}
	} else if ((s_state == S_AUTH || s_state == S_ASSOC) && g_ms > s_deadline) {
		if (s_tries >= 4) {
			w_join_end(ETIMEDOUT);
		} else {
			s_tries++;
			if (s_state == S_AUTH) {
				w_send_auth();
			} else {
				w_mgmt_tx(s_assoc_req, s_assoc_req_len);
			}
			s_deadline = g_ms + 400;
		}
	} else if (s_state == S_4WAY && g_ms > s_deadline) {
		LOG("t=%u ms: join: no 4-way handshake (%u message 1s)", g_ms, s_msg1s);
		w_join_end(s_msg1s != 0 ? EAUTH : ETIMEDOUT);
	}
}

/* a received data frame (native wifi, from the HTT rx ring) */
static void
w_rx_native(const uint8_t *f, uint32_t len)
{
	uint8_t e[14 + 1600];
	if (len < 24 + 8 || (f[0] & 0x0C) != 0x08) {
		return;                                 /* data frames only */
	}
	uint32_t hl = ((f[1] & 3) == 3) ? 30 : 24;
	const uint8_t *da = f + 4, *sa = (f[1] & 2) ? f + 16 : f + 10;   /* from DS: addr3 is the source */
	const uint8_t *llc = f + hl;
	if (len < hl + 8 || llc[0] != 0xAA || llc[1] != 0xAA || llc[2] != 0x03) {
		return;
	}
	uint32_t plen = len - hl - 8;
	if (plen > 1600 - 0) {
		return;
	}
	memcpy(e, da, 6);
	memcpy(e + 6, sa, 6);
	memcpy(e + 12, llc + 6, 2);
	memcpy(e + 14, llc + 8, plen);
	if (e[12] == 0x88 && e[13] == 0x8E) {
		w_eapol_rx(e, 14 + plen);
		return;
	}
	if (s_state == S_UP && topaz_wlan_rx_hook != NULL) {
		rx_up++;
		topaz_wlan_rx_hook(e, 14 + plen);
	}
}

/* ---- the C API */
extern "C" {
int topaz_wlan_join(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pass, unsigned pass_len);
int topaz_wlan_join_pmk(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t pmk[32]);
int topaz_wlan_leave(void);
bool topaz_wlan_joined(void);
bool topaz_wlan_transmit(const uint8_t *frame, size_t length);
}

static int w_join_with(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pmk);

/* Blocks until joined or failed (EAUTH: wrong passphrase; ENOENT: not heard; ETIMEDOUT). */
int
topaz_wlan_join(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pass, unsigned pass_len)
{
	static bool tested, crypto_ok;
	if (!api_ready || ssid_len == 0 || ssid_len > 32) {
		return api_ready ? EINVAL : ENXIO;
	}
	if (!tested) {
		tested = true;
		crypto_ok = w_crypto_selftest();
	}
	if (pass_len != 0 && (!crypto_ok || pass_len < 8 || pass_len > 63)) {
		return crypto_ok ? EINVAL : EIO;
	}
	uint8_t pmk[32] = {};
	if (pass_len != 0) {
		w_pbkdf2_pmk(pass, pass_len, ssid, ssid_len, pmk);
	}
	int r = w_join_with(ssid, ssid_len, bssid, pass_len != 0 ? pmk : NULL);
	bzero(pmk, sizeof(pmk));
	return r;
}

/* the same with the PMK already made of the passphrase (what wifid hands over) */
int
topaz_wlan_join_pmk(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t pmk[32])
{
	if (!api_ready || ssid_len == 0 || ssid_len > 32) {
		return api_ready ? EINVAL : ENXIO;
	}
	return w_join_with(ssid, ssid_len, bssid, pmk);
}

static int
w_join_with(const uint8_t *ssid, unsigned ssid_len, const uint8_t *bssid, const uint8_t *pmk)
{
	while (s_state != S_IDLE && s_state != S_UP && s_join_asked) {
		IOSleep(50);
	}
	memcpy(s_ssid, ssid, ssid_len);
	s_ssid_len = (uint8_t)ssid_len;
	if (bssid != NULL) {
		memcpy(s_bssid, bssid, 6);
	} else {
		bzero(s_bssid, 6);
	}
	if (pmk != NULL) {
		memcpy(s_pmk, pmk, 32);
	} else {
		bzero(s_pmk, 32);
	}
	uint32_t before = s_join_done;
	s_join_asked = true;
	for (int i = 0; i < 300 && s_join_done == before; i++) {   /* 30 s */
		IOSleep(100);
	}
	return s_join_done != before ? s_result : ETIMEDOUT;
}

int
topaz_wlan_leave(void)
{
	s_leave_asked = true;
	return 0;
}

bool
topaz_wlan_joined(void)
{
	return s_state == S_UP;
}

/* frames from the network stack wait here for the modem thread */
#define ETXQ 32
static struct { uint16_t len; uint8_t d[1514]; } etxq[ETXQ];
static volatile uint32_t etxq_head, etxq_tail;
static IOSimpleLock *etxq_lock;

bool
topaz_wlan_transmit(const uint8_t *frame, size_t length)
{
	if (s_state != S_UP || length < 14 || length > 1514 || etxq_lock == NULL) {
		return false;
	}
	bool ok = false;
	IOSimpleLockLock(etxq_lock);
	if (etxq_tail - etxq_head < ETXQ) {
		uint32_t i = etxq_tail % ETXQ;
		etxq[i].len = (uint16_t)length;
		memcpy(etxq[i].d, frame, length);
		etxq_tail++;
		ok = true;
	}
	IOSimpleLockUnlock(etxq_lock);
	if (!ok) {
		tx_drop++;
	}
	return ok;
}

static void
w_tx_pump(void)
{
	static uint8_t one[1514];
	for (int n = 0; n < 16 && etxq_head != etxq_tail; n++) {
		uint16_t len;
		IOSimpleLockLock(etxq_lock);
		uint32_t i = etxq_head % ETXQ;
		len = etxq[i].len;
		memcpy(one, etxq[i].d, len);
		IOSimpleLockUnlock(etxq_lock);
		if (s_state != S_UP || !w_tx_ether(one, len, false)) {
			if (s_state != S_UP) {
				IOSimpleLockLock(etxq_lock);
				etxq_head = etxq_tail;          /* not joined: drop them */
				IOSimpleLockUnlock(etxq_lock);
			}
			break;
		}
		IOSimpleLockLock(etxq_lock);
		etxq_head++;
		IOSimpleLockUnlock(etxq_lock);
	}
}

static void
wmi_rx(const uint8_t *d, uint32_t len)
{
	uint32_t id;
	if (len < 4) {
		return;
	}
	memcpy(&id, d, 4);
	id &= 0xFFFFFF;
	switch (id) {
	case WMI_EV_SERVICE_AVAILABLE: {
		uint16_t tag;
		memcpy(&tag, d + 6, 2);
		if (len >= 12 + sizeof(wmi_svc_ext) && tag == TAG_SERVICE_AVAILABLE) {
			memcpy(wmi_svc_ext, d + 12, sizeof(wmi_svc_ext));
		}
		break;
	}
	case WMI_EV_SERVICE_READY:
		LOG("t=%u ms: wmi: SERVICE_READY, %u bytes", g_ms, len);
		wmi_svc_ready(d + 4, len - 4);
		if (!wmi_init_sent) {
			wmi_send_init();
		}
		break;
	case WMI_EV_READY:
		for (uint32_t o = 4; o + 4 <= len;) {
			uint16_t l, tag;
			memcpy(&l, d + o, 2);
			memcpy(&tag, d + o + 2, 2);
			if (o + 4 + l > len) {
				break;
			}
			if (tag == TAG_READY_EVENT && l >= 24 + 8 + 4) {
				memcpy(wmi_mac, d + o + 4 + 24, 6);
				wmi_ready = true;
			}
			o += 4 + l;
		}
		LOG("t=%u ms: *** WMI READY (fw MAC %02x:%02x:%02x:%02x:%02x:%02x) ***", g_ms, wmi_mac[0], wmi_mac[1],
		    wmi_mac[2], wmi_mac[3], wmi_mac[4], wmi_mac[5]);
		if (wmi_ready) {
			htt_start();                        /* the host rx ring first, then the scan */
		}
		break;
	case WMI_EV_SCAN:
		scan_event(d + 4, len - 4);
		break;
	case WMI_EV_MGMT_RX:
		scan_mgmt_rx(d + 4, len - 4);
		break;
	case 0x5001:                                /* VDEV_START_RESP */
	case 0x6001:                                /* PEER_STA_KICKOUT */
		w_join_event(id, d + 4, len - 4);
		break;
	default:
		break;
	}
}

/* from the watch loop */
static void
radio_poll(unsigned ms)
{
	(void)ms;
	if (w_cap_retry_ms != 0 && g_ms >= w_cap_retry_ms) {
		w_cap_retry_ms = 0;
		w_simple(WLFW_CAP, "cap sent again");
	}
	if (!ce_up) {
		return;
	}
	(void)ce_poll();
	htt_poll();
	if (scan_started && !api_ready) {
		if (etxq_lock == NULL) {
			etxq_lock = IOSimpleLockAlloc();
		}
		api_ready = true;
		LOG("t=%u ms: wlan: radio ready for the rest of the kernel", g_ms);
		if (topaz_wlan_ready_hook != NULL) {
			topaz_wlan_ready_hook();
		}
	}
	if (api_ready) {
		w_join_poll();
		w_tx_pump();
	}
	if (scan_started && !scanning && api_scan_asked && (s_state == S_IDLE || s_state == S_UP)) {
		api_scan_asked = false;
		next_scan_ms = 0;
		scanning = scan_send_start();
		if (!scanning) {
			api_scans_done++;
		}
	}
	if (scan_started && !scanning && next_scan_ms != 0 && g_ms >= next_scan_ms) {
		next_scan_ms = 0;
		scanning = scan_send_start();
		LOG("t=%u ms: scan: passive scan %u %s", g_ms, scans + 1, scanning ? "requested" : "FAILED");
	}
}

static void
svc_announce(void)
{
	static const uint32_t svc[][3] = {
		{ 0x40, 0x101, PORT_PDM },              /* servreg-locator, version 0x101 instance 0 */
		{ 4096, 1,     PORT_TFTP },             /* tftp */
		{ 14,   1,     PORT_RMTFS },            /* rmtfs */
	};
	for (unsigned i = 0; i < 3; i++) {
		struct qrtr_ctrl c = { QRTR_TYPE_NEW_SERVER, svc[i][0], svc[i][1], QRTR_NODE_APPS, svc[i][2] };
		qrtr_send(QRTR_TYPE_NEW_SERVER, QRTR_PORT_CTRL, q_modem_node, QRTR_PORT_CTRL, &c, sizeof(c));
	}
	LOG("t=%u ms: announced pd-mapper 0x40, tftp 0x1000, rmtfs 0x0e", g_ms);
}

static bool
svc_rx(uint32_t src_node, uint32_t src_port, uint32_t dst_port, const uint8_t *data, uint32_t len)
{
	if (dst_port == PORT_TFTP) {
		if (len >= 2 && (data[1] == OP_RRQ || data[1] == OP_WRQ)) {
			tftp_request(src_node, src_port, data, len);
		}
		return true;
	}
	if (dst_port == PORT_WLFWC) {
		wlfw_rx(data, len);
		return true;
	}
	if (dst_port >= PORT_SESS && dst_port < PORT_SESS + 0x10000) {
		return tftp_session(dst_port, data, len);
	}
	if (len < 7 || data[0] != QMI_REQ) {
		return false;
	}
	uint16_t txn, msg;
	memcpy(&txn, data + 1, 2);
	memcpy(&msg, data + 3, 2);
	if (dst_port == PORT_PDM) {
		pdm_rx(src_node, src_port, txn, msg, data, len);
		return true;
	}
	if (dst_port == PORT_RMTFS) {
		rmtfs_rx(src_node, src_port, txn, msg, data, len);
		return true;
	}
	return false;
}

static void
svc_summary(void)
{
	LOG("WLFW: %s%s", w_step, w_fw_ready ? " (firmware ready)" : "");
	LOG("services: pd-mapper %u reqs; tftp %u reqs (%u missing, %u writes); rmtfs reads %u writes %u (not kept) errors %u",
	    pd_reqs, tftp_reqs, tftp_missing, tftp_writes, rmtfs_reads, rmtfs_writes, rmtfs_err);
}

/*
 * After auth_and_reset: watch the modem's "slave-kernel" entry (bit0 fatal, bit1 ready,
 * bit2 handover) and answer its IPA power query ("valid, AP IPA power off"): unanswered,
 * modem_cfg starves and the modem watchdog fires once data services come up.
 */
static void
modem_watch(struct smem_part_hdr *part, unsigned seconds)
{
	struct smp2p_hdr *in = NULL, *out = smp2p_create_out(part);
	volatile uint32_t *sk = NULL, *ipa_in = NULL, *ipa_out = NULL;
	uint32_t sk_last = ~0u, ipa_last = ~0u;
	bool ready = false;

	if (out == NULL) {
		LOG("cannot create SMP2P item %u", ITEM_SMP2P_OUT);
		return;
	}
	ipa_out = smp2p_add_out(out, "ipa", 0);
	bool glink = false;
	for (unsigned ms = 0; seconds == 0 || ms < seconds * 1000; ms++) {
		if (ready && !glink) {
			glink = glink_start(part);
			if (!glink) {
				ready = false;  /* no retry */
			}
		}
		if (glink) {
			glink_poll(part, ms);
			radio_poll(ms);
		}
		if (glink && (ms == 3000 || ms == 20000)) {
			glink_summary();                /* early, before the iOS UI covers the console */
		}
		if (in == NULL) {
			uint32_t sz = 0;
			in = (struct smp2p_hdr *)smem_get(part, ITEM_SMP2P_IN, &sz);
			if (in != NULL && in->magic != SMP2P_MAGIC) {
				in = NULL;
			}
			if (in != NULL) {
				LOG("t=%u ms: modem SMP2P item %u up (v%u, %u entries)", ms, ITEM_SMP2P_IN, in->version, in->valid);
			}
		}
		if (in != NULL && sk == NULL) {
			sk = smp2p_find(in, "slave-kernel");
		}
		if (in != NULL && ipa_in == NULL) {
			ipa_in = smp2p_find(in, "ipa");
		}
		if (sk != NULL && *sk != sk_last) {
			sk_last = *sk;
			LOG("t=%u ms: slave-kernel %08x%s%s%s", ms, sk_last, (sk_last & 1) ? " FATAL" : "",
			    (sk_last & 2) ? " READY" : "", (sk_last & 4) ? " HANDOVER" : "");
			if ((sk_last & 2) && !ready) {
				ready = true;
				LOG("modem is up (MODEM READY)");
			}
			if (sk_last & 1) {
				uint32_t sz = 0;
				const char *why = (const char *)smem_get(smem_partition(SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST), ITEM_MSS_CRASH, &sz);
				char buf[128] = {};
				if (why != NULL) {
					memcpy(buf, why, MIN(sz, (uint32_t)sizeof(buf) - 1));
				}
				LOG("modem crash reason: %s", why != NULL ? buf : "(none)");
				if (glink) {
					glink_summary();
				}
				return;
			}
		}
		if (ipa_in != NULL && ipa_out != NULL && *ipa_in != ipa_last) {
			uint32_t v = *ipa_in;
			if ((v & 1) && !(ipa_last != ~0u && (ipa_last & 1))) {
				*ipa_out = 1;                   /* answer valid, AP IPA power off */
				smp2p_kick();
				LOG("t=%u ms: answered IPA power query (%08x)", ms, v);
			}
			ipa_last = v;
		}
		IOSleep(1);
	}
	LOG("watch done: slave-kernel %08x, %s", sk_last, ready ? "READY" : "NOT ready");
	if (glink) {
		glink_summary();
	}
}

/* ---------------------------------------------------------------- PAS boot */

static bool
loadable(const struct elf32_phdr *p)
{
	return p->p_type == PT_LOAD && p->p_memsz != 0 && (p->p_flags & MDT_TYPE_MASK) != MDT_TYPE_HASH;
}

static bool
modem_boot(void)
{
	uint8_t *mdt = NULL, *meta;
	int64_t mdt_size;
	uint64_t meta_pa = 0, res = 0, st;
	uint32_t min_a = ~0u, max_a = 0, hash = ~0u;
	bool relocate = false, ok = false;

	struct smem_part_hdr *part = smem_partition(0, 1);
	LOG("SMEM apps<->modem partition %p", part);
	if (part == NULL) {
		return false;
	}
	if (smem_get(part, ITEM_SMP2P_IN, NULL) != NULL) {
		LOG("modem SMP2P item %u already exists: modem is not cold, not touching it", ITEM_SMP2P_IN);
		return false;
	}
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_IS_SUPPORTED), 1, PAS_ID_MODEM, 0, 0, &res);
	LOG("PAS id %u supported: ret=%llx res=%llx", PAS_ID_MODEM, st, res);
	/* informational only: on topaz TZ answers res=0 here, init_image decides */

	mdt_size = fw_size("modem.mdt");
	if (mdt_size <= (int64_t)sizeof(struct elf32_ehdr) || mdt_size > 0x100000) {
		LOG("modem.mdt: size %lld", mdt_size);
		return false;
	}
	mdt = (uint8_t *)IOMalloc(mdt_size);
	if (mdt == NULL || fw_read("modem.mdt", mdt, mdt_size) != mdt_size) {
		goto out;
	}
	{
		struct elf32_ehdr *eh = (struct elf32_ehdr *)mdt;
		if (memcmp(eh->e_ident, "\x7f" "ELF", 4) != 0 ||
		    eh->e_phoff + (uint64_t)eh->e_phnum * sizeof(struct elf32_phdr) > (uint64_t)mdt_size) {
			LOG("modem.mdt: bad ELF header");
			goto out;
		}
		struct elf32_phdr *ph = (struct elf32_phdr *)(mdt + eh->e_phoff);
		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			if ((ph[i].p_flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
				hash = i;
			}
			if (!loadable(&ph[i])) {
				continue;
			}
			if (ph[i].p_flags & MDT_RELOCATABLE) {
				relocate = true;
			}
			min_a = MIN(min_a, ph[i].p_paddr);
			max_a = MAX(max_a, ph[i].p_paddr + ph[i].p_memsz);
		}
		LOG("phdrs=%u hash=%u region %08x..%08x relocate=%u", eh->e_phnum, hash, min_a, max_a, relocate);
		if (hash == ~0u || min_a < 0x4AB00000u || max_a > 0x4AB00000u + 0x6900000u) {
			LOG("unexpected layout (carveout is 0x4ab00000+0x6900000)");
			goto out;
		}

		/* metadata = ELF header + phdrs + hash segment, contiguous, below 0xF0000000 */
		uint64_t hdr = eh->e_phoff + eh->e_phnum * sizeof(struct elf32_phdr);
		uint64_t meta_size = hdr + ph[hash].p_filesz;
		meta = (uint8_t *)pool_alloc(meta_size, &meta_pa);
		if (meta == NULL) {
			LOG("metadata: pool allocation of %llu bytes failed", meta_size);
			goto out;
		}
		memcpy(meta, mdt, hdr);
		if (ph[hash].p_offset + ph[hash].p_filesz <= (uint64_t)mdt_size) {
			memcpy(meta + hdr, mdt + ph[hash].p_offset, ph[hash].p_filesz);
		} else {
			/* split firmware: the hash segment is its own .bNN file (qcom_mdt_read_metadata) */
			char hname[16];
			snprintf(hname, sizeof(hname), "modem.b%02u", hash);
			int64_t got = fw_read(hname, meta + hdr, ph[hash].p_filesz);
			LOG("hash from %s: %lld of %u bytes", hname, got, ph[hash].p_filesz);
			if (got != (int64_t)ph[hash].p_filesz) {
				goto out;
			}
		}
		mb();

		st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), PAS_ID_MODEM, meta_pa, 0, &res);
		LOG("init_image: ret=%llx res=%llx resumes=%u (meta %llu bytes @%llx)", st, res, scm_resumes, meta_size, meta_pa);
		if (st != 0 || res != 0) {
			goto out;
		}
		if (relocate) {
			st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, PAS_ID_MODEM, min_a,
			    ((uint64_t)(max_a - min_a) + 0xfff) & ~0xfffull, &res);
			LOG("mem_setup: ret=%llx res=%llx", st, res);
			if (st != 0 || res != 0) {
				goto shutdown;
			}
		}

		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			char name[16];
			if (!loadable(&ph[i])) {
				continue;
			}
			/* map only while copying: TZ locks the carveout at auth_and_reset */
			uint64_t pa = ph[i].p_paddr & ~0x3fffull, off = ph[i].p_paddr - pa;
			uint64_t len = (off + ph[i].p_memsz + 0x3fff) & ~0x3fffull;
			IOMemoryMap *m = map_phys(pa, len, true);
			if (m == NULL) {
				LOG("segment %u: cannot map %08x+%x", i, ph[i].p_paddr, ph[i].p_memsz);
				goto shutdown;
			}
			uint8_t *dst = (uint8_t *)m->getVirtualAddress() + off;
			int64_t got = 0;
			if (ph[i].p_filesz != 0) {
				snprintf(name, sizeof(name), "modem.b%02u", i);
				got = fw_read(name, dst, ph[i].p_filesz);
				if (got != (int64_t)ph[i].p_filesz) {
					LOG("%s: %lld of %u bytes", name, got, ph[i].p_filesz);
					m->release();
					goto shutdown;
				}
			}
			if (ph[i].p_memsz > got) {
				bzero(dst + got, ph[i].p_memsz - got);
			}
			mb();
			m->release();
		}
		LOG("segments loaded");

		st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, PAS_ID_MODEM, 0, 0, &res);
		LOG("auth_and_reset: ret=%llx res=%llx resumes=%u", st, res, scm_resumes);
		if (st != 0 || res != 0) {
			goto shutdown;
		}
		ok = true;
		goto out;
	}
shutdown:
	st = scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, PAS_ID_MODEM, 0, 0, &res);
	LOG("shutdown after failure: ret=%llx res=%llx", st, res);
out:
	if (mdt != NULL) {
		IOFree(mdt, mdt_size);
	}
	return ok;
}

/*
 * Wait for the firmware partition (sweet_ufs comes up with the root device) and map SMEM, the
 * pool, APCS and the TCSR mutex. Once, for whichever of TopazModem / TopazAudio comes first;
 * the mappings stay for the session.
 */
extern "C" bool
topaz_soc_map(void)
{
	static IOLock *once;
	static int state;                           /* 0 not yet, 1 ok, -1 failed */
	IOMemoryMap *smem_map, *pool_map;

	if (once == NULL) {
		IOLock *l = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, l, (void * volatile *)&once)) {
			IOLockFree(l);
		}
	}
	IOLockLock(once);
	if (state != 0) {
		IOLockUnlock(once);
		return state > 0;
	}
	state = -1;
	soc_lock = IOLockAlloc();
	(void)PE_parse_boot_argn("modem_part", fw_part, sizeof(fw_part));
	uint8_t probe[16];
	int err = EAGAIN;
	for (int i = 0; i < 120 && err == EAGAIN; i++) {
		err = sweet_ufs_read_part(fw_part, fw_files[0].off, probe, sizeof(probe));
		if (err == EAGAIN) {
			IOSleep(500);
		}
	}
	smem_map = err == 0 ? map_phys(SMEM_PA, SMEM_SIZE, true) : NULL;
	pool_map = err == 0 ? map_phys(MODEM_POOL_BASE, MODEM_POOL_SIZE, true) : NULL;
	apcs = (volatile uint8_t *)ml_io_map(APCS_PA, 0x1000);
	tcsr_mutex = (volatile uint8_t *)ml_io_map(TCSR_MUTEX_PA, 0x4000);
	if (err != 0) {
		LOG("partition %s: %d", fw_part, err);
	} else if (smem_map == NULL || pool_map == NULL || apcs == NULL || tcsr_mutex == NULL || soc_lock == NULL) {
		LOG("mappings failed");
	} else {
		smem = (uint8_t *)smem_map->getVirtualAddress();
		pool = (uint8_t *)pool_map->getVirtualAddress();
		state = 1;
	}
	IOLockUnlock(once);
	return state > 0;
}

/* ---------------------------------------------------------------- shared with TopazAudio */

extern "C" void *
topaz_smem_partition(uint16_t a, uint16_t b)
{
	return smem_partition(a, b);
}

extern "C" void *
topaz_smem_get(void *part, uint16_t item, uint32_t *size)
{
	return smem_get((struct smem_part_hdr *)part, item, size);
}

extern "C" void *
topaz_smem_alloc(void *part, uint16_t item, uint32_t size)
{
	return smem_alloc((struct smem_part_hdr *)part, item, size);
}

extern "C" void
topaz_apcs_kick(unsigned bit)
{
	mb();
	*(volatile uint32_t *)(apcs + APCS_IPC) = 1u << bit;
	mb();
}

extern "C" uint64_t
topaz_scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1)
{
	return scm(fn, arginfo, a, b, c, res1);
}

extern "C" void *
topaz_pool_alloc(uint64_t size, uint64_t *pa)
{
	return pool_alloc(size, pa);
}

/* a file of the modem partition by its path ("image/adsp.mdt"): size, or -1 */
extern "C" int64_t
topaz_part_fw_size(const char *path)
{
	int i = fw_all_find(path);
	return i < 0 ? -1 : (int64_t)fw_all[i].size;
}

extern "C" bool
topaz_part_fw_read(const char *path, uint64_t off, void *dst, uint64_t n)
{
	int i = fw_all_find(path);
	return i >= 0 && off + n <= fw_all[i].size && fw_all_read(i, off, dst, n);
}

extern "C" bool
topaz_smmu_identity(uint32_t sid, uint32_t mask, const char *name)
{
	return smmu_identity(sid, mask, name);
}

static void
topaz_modem_thread(void *arg __unused, wait_result_t wr __unused)
{
	IOMemoryMap *smem_map, *pool_map;
	unsigned watch = 0;                         /* 0: serve the modem for the whole session */

	(void)PE_parse_boot_argn("modem_watch", &watch, sizeof(watch));
	(void)smem_map;
	(void)pool_map;
	if (!topaz_soc_map()) {
		thread_terminate(current_thread());
		return;
	}
	LOG("SMEM version %u", *(volatile uint32_t *)(smem + 64 + 7 * 4) >> 16);
	if (wifi_enabled() && !smmu_identity(WLAN_SID, WLAN_SID_MASK, "WLAN")) {
		LOG("wifi: no SMMU stream for WLAN, the radio stays off");
		wifi_on = 0;
	}
	if (modem_boot()) {
		modem_watch(smem_partition(0, 1), watch);
	}
	/* keep the mappings: the modem uses SMEM from now on */
	thread_terminate(current_thread());
}

extern "C" void topaz_sta_init(void);

extern "C" void
topaz_modem_init(void)
{
	static bool started;
	uint32_t on = 0;
	thread_t thread;

	if (started || !PE_parse_boot_argn("modem", &on, sizeof(on)) || on == 0) {
		return;
	}
	started = true;
	topaz_sta_init();                           /* private: bsd/dev/topaz_sta.c (en0 + Apple80211 for wifid) */
	if (kernel_thread_start(topaz_modem_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

#else
extern "C" void topaz_modem_init(void) {}
#endif
