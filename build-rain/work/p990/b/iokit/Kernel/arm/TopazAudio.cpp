/*
 * topaz (SM6225): sound, step 1 - boot the ADSP (LPASS Hexagon) and reach AudioReach (SPF).
 * Port of MakrSas/topaz-woa drivers/TopazAudio (Pas.c, Glink.c, Qrtr.c, Gpr.c), where the
 * whole speaker chain works on Windows (docs/P9_audio.md there):
 *   TrustZone PAS (id 1): init_image (.mdt metadata) -> mem_setup -> copy the adsp.bNN
 *   segments into the carveout 0x53800000+0x2300000 -> auth_and_reset -> SMP2P: our item 429,
 *   the ADSP's item 443 with "slave-kernel" READY -> GLINK on the "lpass" edge (apps<->adsp
 *   SMEM partition, items 478/479/480, APCS IPC bit 8) -> QRTR on IPCRTR (+ pd-mapper for the
 *   ADSP's protection domains) -> GPR on adsp_apps: APM GET_SPF_STATE until it says 1.
 *
 * Boot-arg adsp=1. Firmware: image/adsp.* from the modem partition (TopazModem's reader).
 * SMEM, the APCS doorbell, SCM and the pool are TopazModem's (topaz_* calls). Polling only.
 * Lesson from Windows: never boot the ADSP over SMEM state of an earlier run (the SoC reset):
 * if item 443 or 480 exists, refuse.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <sys/param.h>

#define LOG(fmt, ...)       IOLog("TopazAudio: " fmt "\n", ##__VA_ARGS__)

extern "C" bool     topaz_soc_map(void);
extern "C" void    *topaz_smem_partition(uint16_t a, uint16_t b);
extern "C" void    *topaz_smem_get(void *part, uint16_t item, uint32_t *size);
extern "C" void    *topaz_smem_alloc(void *part, uint16_t item, uint32_t size);
extern "C" void     topaz_apcs_kick(unsigned bit);
extern "C" uint64_t topaz_scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1);
extern "C" void    *topaz_pool_alloc(uint64_t size, uint64_t *pa);
extern "C" int64_t  topaz_part_fw_size(const char *path);
extern "C" bool     topaz_part_fw_read(const char *path, uint64_t off, void *dst, uint64_t n);

#define ADSP_PID            2                   /* SMEM host / SMP2P / GLINK remote pid */
#define ADSP_PAS_ID         1
#define ADSP_CARVE_PA       0x53800000u         /* DT pil_adsp_region */
#define ADSP_CARVE_SIZE     0x02300000u
#define ITEM_SMP2P_OUT      429                 /* apps -> adsp (DT qcom,smem = <443 429>: in, out) */
#define ITEM_SMP2P_IN       443                 /* adsp -> apps */
#define ITEM_ADSP_CRASH     423
#define ADSP_SMP2P_BIT      10
#define ADSP_GLINK_BIT      8
#define SMEM_GLOBAL_HOST    0xFFFE

#define SCM_FN(svc, cmd)    (0x42000000u | ((uint32_t)(svc) << 8) | (uint32_t)(cmd))
#define SCM_SVC_PIL         0x02
#define PIL_INIT_IMAGE      0x01
#define PIL_MEM_SETUP       0x02
#define PIL_AUTH_RESET      0x05
#define PIL_SHUTDOWN        0x06
#define SCM_ARG_RW          2

#define PT_LOAD             1
#define MDT_TYPE_MASK       (7u << 24)
#define MDT_TYPE_HASH       (2u << 24)
#define MDT_RELOCATABLE     (1u << 27)

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
struct smp2p_hdr { uint32_t magic; uint8_t version, features[3]; uint16_t local_pid, remote_pid, total, valid; uint32_t flags; } __attribute__((packed));
struct smp2p_entry { char name[16]; uint32_t value; } __attribute__((packed));

static inline void mb(void) { __asm__ volatile ("dsb sy" ::: "memory"); }

static uint64_t t0;
static unsigned
ms(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	return (unsigned)(ns / 1000000);
}
#define T   ms() / 1000, ms() % 1000

static void *part;                              /* apps<->adsp SMEM partition */

/* ---------------------------------------------------------------- SMP2P */

static struct smp2p_hdr *sp_out, *sp_in;
static volatile uint32_t *stop_entry, *adsp_state;

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
smp2p_add(struct smp2p_hdr *o, const char *name, uint32_t value)
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
	mb();
	return &e[o->valid - 1].value;
}

static bool
smp2p_out_init(void)
{
	sp_out = (struct smp2p_hdr *)topaz_smem_alloc(part, ITEM_SMP2P_OUT,
	    sizeof(struct smp2p_hdr) + SMP2P_MAX_ENTRY * sizeof(struct smp2p_entry));
	if (sp_out == NULL) {
		return false;
	}
	if (sp_out->version == 0) {
		sp_out->magic = SMP2P_MAGIC;
		sp_out->local_pid = 0;
		sp_out->remote_pid = ADSP_PID;
		sp_out->total = SMP2P_MAX_ENTRY;
		sp_out->valid = 0;
		sp_out->features[0] = 1;            /* SSR_ACK */
		mb();
		sp_out->version = 1;
	}
	stop_entry = smp2p_add(sp_out, "master-kernel", 0);    /* bit0 = stop request */
	smp2p_add(sp_out, "sleepstate", 1);                    /* bit0 = apps awake */
	topaz_apcs_kick(ADSP_SMP2P_BIT);
	return stop_entry != NULL;
}

static void
crash_reason(void)
{
	uint32_t len = 0;
	const char *why = (const char *)topaz_smem_get(topaz_smem_partition(SMEM_GLOBAL_HOST, SMEM_GLOBAL_HOST),
	    ITEM_ADSP_CRASH, &len);
	char msg[160];

	if (why == NULL || len == 0 || why[0] == 0) {
		LOG("crash reason (%u): %s", ITEM_ADSP_CRASH, why == NULL ? "item absent" : "empty");
		return;
	}
	strlcpy(msg, why, MIN((size_t)len + 1, sizeof(msg)));
	LOG("crash reason (%u): %s", ITEM_ADSP_CRASH, msg);
}

/* ---------------------------------------------------------------- PAS load */

static bool
phdr_loadable(const struct elf32_phdr *p)
{
	return p->p_type == PT_LOAD && p->p_memsz != 0 && (p->p_flags & MDT_TYPE_MASK) != MDT_TYPE_HASH;
}

static bool
adsp_load(void)
{
	int64_t mdt_size = topaz_part_fw_size("image/adsp.mdt");
	uint8_t *mdt = NULL, *meta;
	uint64_t meta_pa = 0, st, res = 0;
	uint32_t min_addr = 0xFFFFFFFFu, max_addr = 0, hash_idx = 0xFFFFFFFFu;
	bool relocate = false, ok = false;
	char name[32];

	if (mdt_size <= 0 || mdt_size > 0x100000 || (mdt = (uint8_t *)IOMalloc((vm_size_t)mdt_size)) == NULL ||
	    !topaz_part_fw_read("image/adsp.mdt", 0, mdt, (uint64_t)mdt_size)) {
		LOG("image/adsp.mdt: not readable (%lld bytes)", mdt_size);
		goto out;
	}
	{
		struct elf32_ehdr *eh = (struct elf32_ehdr *)mdt;
		if ((uint64_t)mdt_size < sizeof(*eh) || memcmp(eh->e_ident, "\177ELF", 4) != 0 ||
		    eh->e_phoff + (uint64_t)eh->e_phnum * sizeof(struct elf32_phdr) > (uint64_t)mdt_size) {
			LOG("adsp.mdt: bad ELF header");
			goto out;
		}
		struct elf32_phdr *ph = (struct elf32_phdr *)(mdt + eh->e_phoff);
		for (uint32_t i = 0; i < eh->e_phnum; i++) {
			if ((ph[i].p_flags & MDT_TYPE_MASK) == MDT_TYPE_HASH) {
				hash_idx = i;
			}
			if (!phdr_loadable(&ph[i])) {
				continue;
			}
			relocate = relocate || (ph[i].p_flags & MDT_RELOCATABLE) != 0;
			min_addr = MIN(min_addr, ph[i].p_paddr);
			max_addr = MAX(max_addr, ph[i].p_paddr + ph[i].p_memsz);
		}
		LOG("phdrs=%u hash=%u region %08x..%08x relocate=%u", eh->e_phnum, hash_idx, min_addr, max_addr, relocate);
		/* never write outside the DT carveout */
		if (hash_idx == 0xFFFFFFFFu || min_addr < ADSP_CARVE_PA || max_addr > ADSP_CARVE_PA + ADSP_CARVE_SIZE) {
			LOG("no hash segment, or segments outside the carveout %08x+%x", ADSP_CARVE_PA, ADSP_CARVE_SIZE);
			goto out;
		}

		/* metadata = ELF header + phdrs + hash segment, physically contiguous below 4 GiB */
		uint64_t hsz = eh->e_phoff + (uint64_t)eh->e_phnum * sizeof(struct elf32_phdr);
		uint64_t meta_size = hsz + ph[hash_idx].p_filesz;
		meta = (uint8_t *)topaz_pool_alloc(meta_size, &meta_pa);
		if (meta == NULL) {
			LOG("no pool memory for %llu bytes of metadata", meta_size);
			goto out;
		}
		memcpy(meta, mdt, hsz);
		if (ph[hash_idx].p_offset + ph[hash_idx].p_filesz <= (uint64_t)mdt_size) {
			memcpy(meta + hsz, mdt + ph[hash_idx].p_offset, ph[hash_idx].p_filesz);
		} else {
			snprintf(name, sizeof(name), "image/adsp.b%02u", hash_idx);
			if (topaz_part_fw_size(name) != (int64_t)ph[hash_idx].p_filesz ||
			    !topaz_part_fw_read(name, 0, meta + hsz, ph[hash_idx].p_filesz)) {
				LOG("%s: hash segment not readable", name);
				goto out;
			}
		}
		mb();

		st = topaz_scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), ADSP_PAS_ID, meta_pa, 0, &res);
		LOG("PAS init_image: ret=%llx res=%llx (meta %llu bytes @%llx)", st, res, meta_size, meta_pa);
		if (st != 0 || res != 0) {
			goto out;
		}
		if (relocate) {
			st = topaz_scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, ADSP_PAS_ID, min_addr,
			    ((uint64_t)(max_addr - min_addr) + 0xfff) & ~0xfffull, &res);
			LOG("PAS mem_setup: ret=%llx res=%llx", st, res);
		}

		/* the carveout, write-combined, only while copying */
		IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(min_addr, max_addr - min_addr, kIODirectionInOut);
		IOMemoryMap *map = md != NULL ? md->map(kIOMapWriteCombineCache) : NULL;
		OSSafeReleaseNULL(md);
		if (map == NULL) {
			LOG("cannot map the carveout");
			goto shutdown;
		}
		uint8_t *carve = (uint8_t *)map->getVirtualAddress();
		bool loaded = true;
		for (uint32_t i = 0; i < eh->e_phnum && loaded; i++) {
			if (!phdr_loadable(&ph[i])) {
				continue;
			}
			uint8_t *dst = carve + (ph[i].p_paddr - min_addr);
			if (ph[i].p_filesz != 0) {
				snprintf(name, sizeof(name), "image/adsp.b%02u", i);
				if (topaz_part_fw_size(name) != (int64_t)ph[i].p_filesz || !topaz_part_fw_read(name, 0, dst, ph[i].p_filesz)) {
					LOG("%s: not readable (%lld of %u bytes)", name, topaz_part_fw_size(name), ph[i].p_filesz);
					loaded = false;
					break;
				}
			}
			if (ph[i].p_memsz > ph[i].p_filesz) {
				bzero(dst + ph[i].p_filesz, ph[i].p_memsz - ph[i].p_filesz);
			}
		}
		mb();
		map->release();
		if (!loaded) {
			goto shutdown;
		}
		LOG("segments loaded");

		st = topaz_scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, ADSP_PAS_ID, 0, 0, &res);
		LOG("PAS auth_and_reset: ret=%llx res=%llx", st, res);
		ok = st == 0 && res == 0;
		if (ok) {
			goto out;
		}
	}
shutdown:
	st = topaz_scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, ADSP_PAS_ID, 0, 0, &res);
	LOG("PAS shutdown after failure: ret=%llx res=%llx", st, res);
out:
	if (mdt != NULL) {
		IOFree(mdt, (vm_size_t)mdt_size);
	}
	return ok;
}

/* Wait for "slave-kernel" READY (bit 1) or FATAL (bit 0) in the ADSP's item 443. */
static uint32_t
wait_ready(unsigned limit_ms)
{
	uint32_t last = 0xFFFFFFFFu, sz = 0;
	bool negotiated = false;

	while (ms() < limit_ms) {
		if (sp_in == NULL) {
			sp_in = (struct smp2p_hdr *)topaz_smem_get(part, ITEM_SMP2P_IN, &sz);
			if (sp_in == NULL) {
				IOSleep(10);
				continue;
			}
			LOG("t=%u.%03u adsp SMP2P item %u appeared (%u bytes)", T, ITEM_SMP2P_IN, sz);
		}
		if (!negotiated && sp_in->version == sp_out->version) {
			sp_out->features[0] &= sp_in->features[0];
			negotiated = true;
			topaz_apcs_kick(ADSP_SMP2P_BIT);
		}
		if (adsp_state == NULL) {
			adsp_state = smp2p_find(sp_in, "slave-kernel");
			if (adsp_state == NULL) {
				IOSleep(10);
			}
			continue;
		}
		if (*adsp_state != last) {
			last = *adsp_state;
			LOG("t=%u.%03u slave-kernel %08x%s%s%s%s", T, last, (last & 1) ? " FATAL" : "", (last & 2) ? " READY" : "",
			    (last & 4) ? " handover" : "", (last & 8) ? " stop-ack" : "");
			if (last & 3) {
				break;
			}
		}
		IOSleep(10);
	}
	return adsp_state != NULL ? *adsp_state : 0;
}

/* ---------------------------------------------------------------- GLINK (lpass edge) */

#define ITEM_GLINK_DESC     478
#define ITEM_GLINK_FIFO_TX  479
#define ITEM_GLINK_FIFO_RX  480
#define GLINK_TX_SIZE       0x4000

enum {
	CMD_VERSION = 0, CMD_VERSION_ACK = 1, CMD_OPEN = 2, CMD_CLOSE = 3, CMD_OPEN_ACK = 4, CMD_INTENT = 5,
	CMD_RX_DONE = 6, CMD_RX_INTENT_REQ = 7, CMD_RX_INTENT_REQ_ACK = 8, CMD_TX_DATA = 9, CMD_CLOSE_ACK = 11,
	CMD_TX_DATA_CONT = 12, CMD_READ_NOTIF = 13, CMD_RX_DONE_W_REUSE = 14, CMD_SIGNALS = 15,
};

#define MAX_CHANS           8
#define OUR_INTENTS         8
#define OUR_INTENT_SIZE     0x4400
#define MAX_RINTENTS        256
#define TXQ_LEN             32

struct glink_msg { uint16_t cmd, p1; uint32_t p2; } __attribute__((packed));
struct glink_data_hdr { struct glink_msg m; uint32_t chunk, left; } __attribute__((packed));

typedef void (*glink_rx_t)(const uint8_t *data, uint32_t len);

struct rintent { uint32_t id, size; bool used; };
struct chan {
	char       name[32];
	glink_rx_t rx;                          /* NULL: not ours, only acked */
	uint16_t   lcid;
	uint32_t   rcid;
	bool       remote_open, open_sent, open_acked, intents_sent, intent_req_sent;
	struct rintent ri[MAX_RINTENTS];
	uint32_t   ri_count;
	uint8_t   *asm_buf;                     /* reassembly, OUR_INTENT_SIZE */
	uint32_t   asm_len;
	void      *txq[TXQ_LEN];
	uint32_t   txq_len[TXQ_LEN];
	uint32_t   txq_head, txq_tail, txq_drops;
	uint32_t   rx_pkts, tx_pkts, ri_lost, intent_reqs;
};

static volatile uint32_t *g_desc;               /* [0] tx tail [1] tx head [2] rx tail [3] rx head */
static uint8_t *g_tx, *g_rx;
static uint32_t g_rx_len, g_features = 1, g_next_intent = 1, g_chans;
static struct chan *g_ch;                       /* MAX_CHANS */
static uint32_t g_rx_msgs, g_tx_msgs;
static bool g_dead;

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
	uint32_t head = 0, tail, avail, n;

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
		LOG("glink TX FIFO full");
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
	topaz_apcs_kick(ADSP_GLINK_BIT);
	g_tx_msgs++;
	return true;
}

static void
send_cmd(uint16_t cmd, uint16_t p1, uint32_t p2)
{
	struct glink_msg m = { cmd, p1, p2 };
	tx_write(&m, sizeof(m), NULL, 0);
}

static struct chan *
by_name(const char *name)
{
	for (uint32_t i = 0; i < g_chans; i++) {
		if (strcmp(g_ch[i].name, name) == 0) {
			return &g_ch[i];
		}
	}
	return NULL;
}

static struct chan *
by_rcid(uint32_t rcid)
{
	for (uint32_t i = 0; i < g_chans; i++) {
		if (g_ch[i].remote_open && g_ch[i].rcid == rcid) {
			return &g_ch[i];
		}
	}
	return NULL;
}

static struct chan *
add_chan(const char *name)
{
	if (g_chans == MAX_CHANS) {
		return NULL;
	}
	struct chan *c = &g_ch[g_chans];
	bzero(c, sizeof(*c));
	strlcpy(c->name, name, sizeof(c->name));
	c->lcid = (uint16_t)(g_chans + 1);
	g_chans++;
	return c;
}

static void
glink_register(const char *name, glink_rx_t rx)
{
	struct chan *c = by_name(name);
	if (c == NULL) {
		c = add_chan(name);
	}
	if (c != NULL) {
		c->rx = rx;
		c->asm_buf = (uint8_t *)IOMallocZero(OUR_INTENT_SIZE);
	}
}

static void
send_open(struct chan *c)
{
	struct { struct glink_msg m; char name[32]; } req;
	uint32_t len = (uint32_t)strlen(c->name) + 1;

	bzero(&req, sizeof(req));
	req.m.cmd = CMD_OPEN;
	req.m.p1 = c->lcid;
	req.m.p2 = len;
	memcpy(req.name, c->name, len);
	tx_write(&req, (8 + len + 7) & ~7u, NULL, 0);
	c->open_sent = true;
	LOG("t=%u.%03u glink OPEN \"%s\" lcid %u", T, c->name, c->lcid);
}

static void
advertise_intents(struct chan *c, uint32_t count, uint32_t size)
{
	struct { struct glink_msg m; uint32_t pair[2 * OUR_INTENTS]; } m;

	count = MIN(count, (uint32_t)OUR_INTENTS);
	m.m.cmd = CMD_INTENT;
	m.m.p1 = c->lcid;
	m.m.p2 = count;
	for (uint32_t i = 0; i < count; i++) {
		m.pair[2 * i] = size;
		m.pair[2 * i + 1] = g_next_intent++;
	}
	tx_write(&m, sizeof(struct glink_msg) + 8 * count, NULL, 0);
}

static bool
chan_up(const struct chan *c)
{
	return c->remote_open && c->open_acked;
}

static bool
glink_chan_up(const char *name)
{
	struct chan *c = by_name(name);
	return c != NULL && chan_up(c);
}

static bool
try_send(struct chan *c, const void *data, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < c->ri_count; i++) {
		if (!c->ri[i].used && c->ri[i].size >= len) {
			break;
		}
	}
	if (i == c->ri_count) {
		if (!c->intent_req_sent) {
			send_cmd(CMD_RX_INTENT_REQ, c->lcid, len);
			c->intent_req_sent = true;
			c->intent_reqs++;
		}
		return false;
	}
	struct glink_data_hdr h = { { CMD_TX_DATA, c->lcid, c->ri[i].id }, len, 0 };
	c->ri[i].used = true;
	c->tx_pkts++;
	return tx_write(&h, sizeof(h), data, len);
}

static void
pump(struct chan *c)
{
	while (chan_up(c) && c->txq_head != c->txq_tail &&
	    try_send(c, c->txq[c->txq_head % TXQ_LEN], c->txq_len[c->txq_head % TXQ_LEN])) {
		IOFree(c->txq[c->txq_head % TXQ_LEN], c->txq_len[c->txq_head % TXQ_LEN]);
		c->txq_head++;
	}
}

/* queue a packet: it goes out when the channel is up and the ADSP has an intent for it */
static bool
glink_send(const char *name, const void *data, uint32_t len)
{
	struct chan *c = by_name(name);
	void *copy;

	if (c == NULL || c->txq_tail - c->txq_head >= TXQ_LEN || len == 0 || (copy = IOMalloc(len)) == NULL) {
		if (c != NULL) {
			c->txq_drops++;
		}
		return false;
	}
	memcpy(copy, data, len);
	c->txq[c->txq_tail % TXQ_LEN] = copy;
	c->txq_len[c->txq_tail % TXQ_LEN] = len;
	c->txq_tail++;
	pump(c);
	return true;
}

static bool
rx_one(void)
{
	struct glink_msg m;
	uint32_t avail = rx_avail(), n;
	struct chan *c;

	if (avail < sizeof(m)) {
		return false;
	}
	rx_peek(0, &m, sizeof(m));
	g_rx_msgs++;

	switch (m.cmd) {
	case CMD_VERSION:
		LOG("t=%u.%03u glink VERSION %u features %x", T, m.p1, m.p2);
		rx_advance(8);
		if (m.p1 != 0) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION_ACK, 1, g_features);
		}
		break;
	case CMD_VERSION_ACK:
		LOG("t=%u.%03u glink VERSION_ACK %u features %x", T, m.p1, m.p2);
		rx_advance(8);
		if (m.p1 != 0 && m.p2 != g_features) {
			g_features &= m.p2;
			send_cmd(CMD_VERSION, 1, g_features);
		}
		break;
	case CMD_OPEN: {
		char name[33];
		n = (8 + m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		bzero(name, sizeof(name));
		rx_peek(8, name, MIN(m.p2, 32u));
		rx_advance(n);
		c = by_name(name);
		if (c == NULL) {
			c = add_chan(name);
		}
		LOG("t=%u.%03u glink ADSP OPEN \"%s\" rcid %u%s", T, name, m.p1, (c != NULL && c->rx != NULL) ? "" : " (no client)");
		if (c != NULL) {
			c->rcid = m.p1;
			c->remote_open = true;
			send_cmd(CMD_OPEN_ACK, (uint16_t)c->rcid, 0);
			if (c->rx != NULL && !c->open_sent) {
				send_open(c);
			}
		}
		break;
	}
	case CMD_OPEN_ACK:
		rx_advance(8);
		for (uint32_t i = 0; i < g_chans; i++) {
			if (g_ch[i].lcid == m.p1) {
				g_ch[i].open_acked = true;
				LOG("t=%u.%03u glink OPEN_ACK \"%s\"", T, g_ch[i].name);
			}
		}
		break;
	case CMD_CLOSE:
		rx_advance(8);
		c = by_rcid(m.p1);
		LOG("t=%u.%03u glink ADSP CLOSE rcid %u \"%s\"", T, m.p1, c != NULL ? c->name : "?");
		send_cmd(CMD_CLOSE_ACK, (uint16_t)m.p1, 0);
		if (c != NULL) {
			c->remote_open = false;
			c->ri_count = 0;
		}
		break;
	case CMD_INTENT:
		n = (8 + 8 * m.p2 + 7) & ~7u;
		if (avail < n) {
			return false;
		}
		c = by_rcid(m.p1);
		for (uint32_t i = 0; c != NULL && i < m.p2; i++) {
			uint32_t pair[2];
			rx_peek(8 + 8 * i, pair, 8);
			if (c->ri_count < MAX_RINTENTS) {
				c->ri[c->ri_count].size = pair[0];
				c->ri[c->ri_count].id = pair[1];
				c->ri[c->ri_count].used = false;
				c->ri_count++;
			} else {
				c->ri_lost++;
			}
		}
		rx_advance(n);
		if (c != NULL) {
			c->intent_req_sent = false;
			pump(c);
		}
		break;
	case CMD_RX_DONE:
	case CMD_RX_DONE_W_REUSE:
		rx_advance(8);
		c = by_rcid(m.p1);
		for (uint32_t i = 0; c != NULL && i < c->ri_count; i++) {
			if (c->ri[i].id == m.p2) {
				if (m.cmd == CMD_RX_DONE_W_REUSE) {
					c->ri[i].used = false;
				} else {
					c->ri[i] = c->ri[--c->ri_count];
				}
				break;
			}
		}
		if (c != NULL) {
			pump(c);
		}
		break;
	case CMD_RX_INTENT_REQ:
		rx_advance(8);
		c = by_rcid(m.p1);
		LOG("t=%u.%03u glink \"%s\" wants an intent of %u bytes", T, c != NULL ? c->name : "?", m.p2);
		if (c != NULL) {
			advertise_intents(c, 1, MAX(m.p2, (uint32_t)OUR_INTENT_SIZE));
			send_cmd(CMD_RX_INTENT_REQ_ACK, c->lcid, 1);
		}
		break;
	case CMD_RX_INTENT_REQ_ACK:
		rx_advance(8);
		c = by_rcid(m.p1);
		if (c != NULL && m.p2 == 0) {
			c->intent_req_sent = false;
		}
		break;
	case CMD_CLOSE_ACK:
	case CMD_SIGNALS:
		rx_advance(8);
		break;
	case CMD_READ_NOTIF:
		rx_advance(8);
		topaz_apcs_kick(ADSP_GLINK_BIT);
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
		c = by_rcid(m.p1);
		if (c != NULL && c->asm_buf != NULL) {
			if (m.cmd == CMD_TX_DATA) {
				c->asm_len = 0;
			}
			if (c->asm_len + hdr[0] <= OUR_INTENT_SIZE) {
				rx_peek(16, c->asm_buf + c->asm_len, hdr[0]);
				c->asm_len += hdr[0];
			}
		}
		rx_advance(n);
		if (hdr[1] == 0 && c != NULL) {
			c->rx_pkts++;
			if (c->rx != NULL) {
				c->rx(c->asm_buf, c->asm_len);
			}
			if (g_features & 1) {
				send_cmd(CMD_RX_DONE_W_REUSE, c->lcid, m.p2);
			} else {
				send_cmd(CMD_RX_DONE, c->lcid, m.p2);
				advertise_intents(c, 1, OUR_INTENT_SIZE);
			}
		}
		break;
	}
	default:
		LOG("t=%u.%03u glink unknown cmd %u (%x %x), RX stopped", T, m.cmd, m.p1, m.p2);
		g_dead = true;
		return false;
	}
	return true;
}

static bool
glink_init(void)
{
	bool existed = topaz_smem_get(part, ITEM_GLINK_DESC, NULL) != NULL;

	g_ch = (struct chan *)IOMallocZero(MAX_CHANS * sizeof(struct chan));
	g_desc = (volatile uint32_t *)topaz_smem_alloc(part, ITEM_GLINK_DESC, 32);
	g_tx = (uint8_t *)topaz_smem_alloc(part, ITEM_GLINK_FIFO_TX, GLINK_TX_SIZE);
	if (g_ch == NULL || g_desc == NULL || g_tx == NULL) {
		LOG("glink: no memory (desc %p tx %p)", g_desc, g_tx);
		g_dead = true;
		return false;
	}
	LOG("glink desc %s", existed ? "existed" : "ours");
	g_desc[2] = 0;                              /* rx tail */
	g_desc[1] = 0;                              /* tx head */
	mb();
	send_cmd(CMD_VERSION, 1, g_features);
	return true;
}

static bool
glink_poll(void)
{
	uint32_t rx0 = g_rx_msgs, tx0 = g_tx_msgs, sz = 0, i;

	if (g_dead || g_desc == NULL) {
		return false;
	}
	if (g_rx == NULL) {
		g_rx = (uint8_t *)topaz_smem_get(part, ITEM_GLINK_FIFO_RX, &sz);
		if (g_rx == NULL) {
			return false;
		}
		g_rx_len = sz;
		LOG("t=%u.%03u ADSP RX FIFO, %u bytes", T, sz);
	}
	for (i = 0; i < 64 && !g_dead && rx_one(); i++) {
	}
	for (i = 0; i < g_chans; i++) {
		struct chan *c = &g_ch[i];
		if (c->rx == NULL) {
			continue;
		}
		/* the ADSP opens its channels first; if it has not after 8 s, open from our side */
		if (!c->open_sent && ms() >= 8000) {
			send_open(c);
		}
		if (chan_up(c) && !c->intents_sent) {
			advertise_intents(c, OUR_INTENTS, OUR_INTENT_SIZE);
			c->intents_sent = true;
			LOG("t=%u.%03u \"%s\" open both ways", T, c->name);
		}
		pump(c);
	}
	return g_rx_msgs != rx0 || g_tx_msgs != tx0;
}

static void
glink_summary(void)
{
	LOG("glink: rx %u msgs, tx %u%s", g_rx_msgs, g_tx_msgs, g_dead ? " (DEAD)" : "");
	for (uint32_t i = 0; i < g_chans; i++) {
		struct chan *c = &g_ch[i];
		LOG(" chan \"%s\" lcid %u rcid %u %s%s rx %u tx %u, remote intents %u, txq %u drops %u", c->name, c->lcid, c->rcid,
		    chan_up(c) ? "UP" : "down", c->rx == NULL ? " (no client)" : "", c->rx_pkts, c->tx_pkts, c->ri_count,
		    c->txq_tail - c->txq_head, c->txq_drops);
	}
}

/* ---------------------------------------------------------------- QRTR + pd-mapper */

#define QCHAN               "IPCRTR"
#define QRTR_NODE_APPS      1
#define QRTR_PORT_CTRL      0xFFFFFFFEu
enum {
	QRTR_TYPE_DATA = 1, QRTR_TYPE_HELLO = 2, QRTR_TYPE_BYE = 3, QRTR_TYPE_NEW_SERVER = 4, QRTR_TYPE_DEL_SERVER = 5,
	QRTR_TYPE_DEL_CLIENT = 6, QRTR_TYPE_RESUME_TX = 7, QRTR_TYPE_EXIT = 8, QRTR_TYPE_PING = 9,
	QRTR_TYPE_NEW_LOOKUP = 10, QRTR_TYPE_DEL_LOOKUP = 11,
};
#define SVC_PDM             0x40
#define PORT_PDM            0x4001
#define QMI_REQ             0
#define QMI_RESP            2

struct qrtr_v1 { uint32_t version, type, src_node, src_port, confirm_rx, size, dst_node, dst_port; } __attribute__((packed));
struct qrtr_v2 { uint8_t version, type, flags, optlen; uint32_t size; uint16_t src_node, src_port, dst_node, dst_port; } __attribute__((packed));
struct qrtr_ctrl { uint32_t cmd, a, b, c, d; } __attribute__((packed));

static uint32_t q_ver = 1, q_node = 0xFFFFFFFFu;
static bool q_hello_rx, q_hello_sent, q_announced;
static uint32_t q_services, q_lookups, q_pdreqs, q_other, q_resumes;

static void
qrtr_send(uint32_t type, uint32_t src_port, uint32_t dst_node, uint32_t dst_port, const void *payload, uint32_t len)
{
	uint8_t buf[sizeof(struct qrtr_v1) + 512];
	uint32_t n;

	if (len > 512) {
		return;
	}
	bzero(buf, sizeof(buf));
	if (q_ver == 2) {
		struct qrtr_v2 *h = (struct qrtr_v2 *)buf;
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
		struct qrtr_v1 *h = (struct qrtr_v1 *)buf;
		h->version = 1;
		h->type = type;
		h->src_node = QRTR_NODE_APPS;
		h->src_port = src_port;
		h->confirm_rx = type == QRTR_TYPE_DATA ? 1 : 0;
		h->size = len;
		h->dst_node = dst_node;
		h->dst_port = dst_port;
		n = sizeof(*h);
	}
	memcpy(buf + n, payload, len);
	glink_send(QCHAN, buf, (n + len + 3) & ~3u);
}

static void
send_ctrl(uint32_t type, uint32_t dst_node, uint32_t dst_port, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
	struct qrtr_ctrl m = { type, a, b, c, d };
	qrtr_send(type, QRTR_PORT_CTRL, dst_node, dst_port, &m, sizeof(m));
}

static const struct { const char *svc, *domain; } pd_map[] = {
	{ "tms/servreg", "msm/adsp/root_pd" },
	{ "tms/servreg", "msm/adsp/sensor_pd" },
	{ "tms/servreg", "msm/adsp/audio_pd" },
	{ "avs/audio", "msm/adsp/audio_pd" },
};

static const uint8_t *
qm_find(const uint8_t *d, uint32_t len, uint8_t type, uint16_t *tlen)
{
	uint32_t o = 7;

	while (o + 3 <= len) {
		uint16_t l = *(const uint16_t *)(d + o + 1);
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

static uint32_t
qm_tlv(uint8_t *m, uint32_t n, uint8_t type, const void *v, uint16_t len)
{
	m[n] = type;
	*(uint16_t *)(m + n + 1) = len;
	memcpy(m + n + 3, v, len);
	return n + 3 + len;
}

static void
pdm_rx(uint32_t node, uint32_t port, uint16_t txn, uint16_t msg, const uint8_t *d, uint32_t len)
{
	uint8_t m[400], list[300];
	char name[80];
	uint16_t r[2] = { 0, 0 }, v, l = 0;
	const uint8_t *s;
	uint32_t n = 7, ln = 1, cnt = 0, inst = 74, zero = 0;

	m[0] = QMI_RESP;
	*(uint16_t *)(m + 1) = txn;
	*(uint16_t *)(m + 3) = msg;
	name[0] = 0;
	s = qm_find(d, len, 1, &l);
	if (s != NULL) {
		l = (uint16_t)MIN((size_t)l, sizeof(name) - 1);
		memcpy(name, s, l);
		name[l] = 0;
	}
	n = qm_tlv(m, n, 2, r, 4);
	if (msg == 0x21) {                          /* GET_DOMAIN_LIST */
		for (unsigned i = 0; i < sizeof(pd_map) / sizeof(pd_map[0]); i++) {
			if (strcmp(pd_map[i].svc, name) != 0) {
				continue;
			}
			uint32_t dl = (uint32_t)strlen(pd_map[i].domain);
			list[ln++] = (uint8_t)dl;
			memcpy(list + ln, pd_map[i].domain, dl);
			ln += dl;
			memcpy(list + ln, &inst, 4);
			ln += 4;
			list[ln++] = 0;                 /* service_data_valid */
			memcpy(list + ln, &zero, 4);
			ln += 4;
			cnt++;
		}
		list[0] = (uint8_t)cnt;
		v = (uint16_t)cnt;
		n = qm_tlv(m, n, 0x10, &v, 2);          /* total_domains */
		v = 1;
		n = qm_tlv(m, n, 0x11, &v, 2);          /* db_rev_count */
		if (cnt != 0) {
			n = qm_tlv(m, n, 0x12, list, (uint16_t)ln);
		}
	}
	*(uint16_t *)(m + 5) = (uint16_t)(n - 7);
	qrtr_send(QRTR_TYPE_DATA, PORT_PDM, node, port, m, n);
	q_pdreqs++;
	LOG("t=%u.%03u pd-mapper: msg %x \"%s\" -> %u domain(s)", T, msg, name, cnt);
}

static void
qrtr_rx(const uint8_t *p, uint32_t len)
{
	uint32_t type, src_node, src_port, dst_port, size, hlen, confirm;

	if (len >= sizeof(struct qrtr_v1) && *(const uint32_t *)p == 1) {
		const struct qrtr_v1 *h = (const struct qrtr_v1 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port; dst_port = h->dst_port;
		size = h->size; hlen = sizeof(*h); confirm = h->confirm_rx;
		q_ver = 1;
	} else if (len >= sizeof(struct qrtr_v2) && p[0] == 2) {
		const struct qrtr_v2 *h = (const struct qrtr_v2 *)p;
		type = h->type; src_node = h->src_node; src_port = h->src_port; dst_port = h->dst_port;
		size = h->size; hlen = sizeof(*h) + h->optlen * 4; confirm = h->flags & 1;
		src_port = src_port == 0xFFFE ? QRTR_PORT_CTRL : src_port;
		dst_port = dst_port == 0xFFFE ? QRTR_PORT_CTRL : dst_port;
		q_ver = 2;
	} else {
		LOG("t=%u.%03u qrtr: bad packet, %u bytes", T, len);
		return;
	}
	if (hlen + size > len) {
		LOG("t=%u.%03u qrtr: truncated (%u + %u > %u)", T, hlen, size, len);
		return;
	}
	struct qrtr_ctrl c;
	bzero(&c, sizeof(c));
	memcpy(&c, p + hlen, MIN((uint32_t)sizeof(c), size));

	switch (type) {
	case QRTR_TYPE_HELLO:
		q_node = src_node;
		q_hello_rx = true;
		q_hello_sent = false;                   /* (re)answer with our HELLO */
		LOG("t=%u.%03u qrtr v%u HELLO from node %u", T, q_ver, src_node);
		break;
	case QRTR_TYPE_NEW_SERVER:
		if (c.a == 0 && c.b == 0) {
			break;
		}
		q_services++;
		LOG("t=%u.%03u ADSP service %x v%u inst %x node %u port %x", T, c.a, c.b & 0xFF, c.b >> 8, c.c, c.d);
		break;
	case QRTR_TYPE_NEW_LOOKUP:
		q_lookups++;
		if (c.a == 0 || c.a == SVC_PDM) {
			send_ctrl(QRTR_TYPE_NEW_SERVER, src_node, src_port, SVC_PDM, 0x101, QRTR_NODE_APPS, PORT_PDM);
		}
		send_ctrl(QRTR_TYPE_NEW_SERVER, src_node, src_port, 0, 0, 0, 0);
		break;
	case QRTR_TYPE_DATA:
		if (confirm) {
			struct qrtr_ctrl r = { QRTR_TYPE_RESUME_TX, QRTR_NODE_APPS, dst_port, 0, 0 };
			qrtr_send(QRTR_TYPE_RESUME_TX, dst_port, src_node, src_port, &r, sizeof(r));
		}
		if (dst_port == PORT_PDM && size >= 7 && p[hlen] == QMI_REQ) {
			pdm_rx(src_node, src_port, *(const uint16_t *)(p + hlen + 1), *(const uint16_t *)(p + hlen + 3), p + hlen, size);
		} else if (q_other++ < 16) {
			LOG("t=%u.%03u qrtr DATA %u:%x -> port %x, %u B", T, src_node, src_port, dst_port, size);
		}
		break;
	case QRTR_TYPE_RESUME_TX:
		q_resumes++;
		break;
	default:
		if (q_other++ < 16) {
			LOG("t=%u.%03u qrtr type %u from %u:%x", T, type, src_node, src_port);
		}
		break;
	}
}

static void
qrtr_poll(void)
{
	if (!glink_chan_up(QCHAN)) {
		return;
	}
	if (!q_hello_sent) {
		send_ctrl(QRTR_TYPE_HELLO, q_node, QRTR_PORT_CTRL, 0, 0, 0, 0);
		q_hello_sent = true;
		LOG("t=%u.%03u sent qrtr HELLO to node %x", T, q_node);
	}
	if (q_hello_rx && !q_announced) {
		send_ctrl(QRTR_TYPE_NEW_SERVER, q_node, QRTR_PORT_CTRL, SVC_PDM, 0x101, QRTR_NODE_APPS, PORT_PDM);
		q_announced = true;
		LOG("t=%u.%03u announced pd-mapper 0x40", T);
	}
}

/* ---------------------------------------------------------------- GPR (AudioReach) */

#define GCHAN               "adsp_apps"
#define GPR_DOMAIN_ADSP     2
#define GPR_DOMAIN_APPS     3
#define GPR_HDR_WORDS       6
#define GPR_OUR_PORT        3                   /* spf_core: gpr service reg = <3> */
#define APM_MODULE_IID      1
#define GPR_BASIC_RSP_RESULT        0x02001005u
#define APM_CMD_GET_SPF_STATE       0x01001021u
#define APM_CMD_RSP_GET_SPF_STATE   0x02001007u

struct gpr_hdr {
	uint32_t word0;                         /* version:4 hdr_size:4 (words) pkt_size:24 (bytes) */
	uint8_t  dest_domain, src_domain;
	uint16_t rsvd;
	uint32_t src_port, dest_port, token, opcode;
} __attribute__((packed));

static bool gpr_reply(const struct gpr_hdr *h, const uint8_t *data, uint32_t len);
static uint32_t gpr_token = 0x100, spf_state = 0xFFFFFFFFu, gpr_asks, gpr_rxn;
static unsigned gpr_last_ask, spf_ready_ms;

static void
gpr_send(uint32_t dest_port, uint32_t opcode, const void *payload, uint32_t len)
{
	uint8_t buf[sizeof(struct gpr_hdr) + 256];
	struct gpr_hdr *h = (struct gpr_hdr *)buf;

	if (len > 256) {
		return;
	}
	bzero(buf, sizeof(buf));
	h->word0 = (GPR_HDR_WORDS << 4) | ((uint32_t)(sizeof(struct gpr_hdr) + len) << 8);
	h->dest_domain = GPR_DOMAIN_ADSP;
	h->src_domain = GPR_DOMAIN_APPS;
	h->src_port = GPR_OUR_PORT;
	h->dest_port = dest_port;
	h->token = gpr_token++;
	h->opcode = opcode;
	memcpy(buf + sizeof(struct gpr_hdr), payload, len);
	glink_send(GCHAN, buf, (uint32_t)sizeof(struct gpr_hdr) + len);
}

static void
gpr_rx(const uint8_t *data, uint32_t len)
{
	const struct gpr_hdr *h = (const struct gpr_hdr *)data;

	gpr_rxn++;
	if (len < sizeof(struct gpr_hdr)) {
		LOG("t=%u.%03u gpr: short packet %u B", T, len);
		return;
	}
	uint32_t hlen = ((h->word0 >> 4) & 0xF) * 4;
	uint32_t p0 = len >= hlen + 4 ? *(const uint32_t *)(data + hlen) : 0;
	uint32_t p1 = len >= hlen + 8 ? *(const uint32_t *)(data + hlen + 4) : 0;

	if (h->opcode == APM_CMD_RSP_GET_SPF_STATE && len >= hlen + 4) {
		if (p0 != spf_state) {
			LOG("t=%u.%03u *** APM SPF state = %u%s (after %u asks) ***", T, p0, p0 == 1 ? " READY" : "", gpr_asks);
		}
		spf_state = p0;
		if (spf_state == 1 && spf_ready_ms == 0) {
			spf_ready_ms = ms();
		}
		return;
	}
	if (h->opcode == GPR_BASIC_RSP_RESULT && p0 == APM_CMD_GET_SPF_STATE && h->dest_port == GPR_OUR_PORT) {
		return;                                 /* "not ready yet" */
	}
	if (gpr_reply(h, data, len)) {
		return;
	}
	if (gpr_rxn <= 32) {
		LOG("t=%u.%03u gpr rx %u B: port %x->%x token %x opcode %08x payload %08x %08x", T, len, h->src_port,
		    h->dest_port, h->token, h->opcode, p0, p1);
	}
}

static void
gpr_poll(void)
{
	if (!glink_chan_up(GCHAN) || spf_state == 1 || gpr_asks >= 120) {
		return;
	}
	if (gpr_asks == 0 || ms() - gpr_last_ask >= 1000) {
		gpr_last_ask = ms();
		gpr_asks++;
		gpr_send(APM_MODULE_IID, APM_CMD_GET_SPF_STATE, NULL, 0);
	}
}

/* ---------------------------------------------------------------- GPR calls (wait for the reply) */

extern "C" bool topaz_rpm_write(uint32_t set, uint32_t type, uint32_t id, uint32_t key, uint32_t value);
extern "C" bool topaz_smmu_identity(uint32_t sid, uint32_t mask, const char *name);
extern "C" int  sweet_i2c1_init(void);
extern "C" int  sweet_i2c1_write(uint8_t addr, uint8_t reg, uint8_t val);
extern "C" int  sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len);
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

static uint32_t call_token, call_op, call_words, call_rsp[16];
static bool call_done;

/* called from gpr_rx for every packet that is not our SPF polling */
static bool
gpr_reply(const struct gpr_hdr *h, const uint8_t *data, uint32_t len)
{
	uint32_t hlen = ((h->word0 >> 4) & 0xF) * 4;

	if (call_token == 0 || h->token != call_token || len < hlen) {
		return false;
	}
	call_op = h->opcode;
	call_words = MIN((len - hlen) / 4, (uint32_t)(sizeof(call_rsp) / 4));
	memcpy(call_rsp, data + hlen, call_words * 4);
	call_done = true;
	return true;
}

/*
 * Send one GPR packet to port dst (from port src), optionally with an in-band apm_cmd_header
 * (payload_size), and pump GLINK until the reply with the same token. TopazSpeaker ta.c TaGpr.
 */
static bool
gpr_call(uint32_t dst, uint32_t src, uint32_t opcode, bool apm_hdr, const uint32_t *dw, uint32_t n, unsigned timeout_ms)
{
	uint32_t pkt[96], k = 6;

	if (n + 10 > sizeof(pkt) / 4) {
		return false;
	}
	bzero(pkt, sizeof(pkt));
	if (apm_hdr) {
		pkt[k + 3] = n * 4;                     /* apm_cmd_header.payload_size */
		k += 4;
	}
	for (uint32_t i = 0; i < n; i++) {
		pkt[k++] = dw[i];
	}
	call_token = gpr_token++;
	call_done = false;
	pkt[0] = (GPR_HDR_WORDS << 4) | ((k * 4) << 8);
	pkt[1] = GPR_DOMAIN_ADSP | (GPR_DOMAIN_APPS << 8);
	pkt[2] = src;
	pkt[3] = dst;
	pkt[4] = call_token;
	pkt[5] = opcode;
	glink_send(GCHAN, pkt, k * 4);
	for (unsigned t = ms(); !call_done && ms() - t < timeout_ms;) {
		if (!glink_poll()) {
			IOSleep(1);
		}
	}
	call_token = 0;
	if (!call_done) {
		LOG("gpr %08x: no reply", opcode);
	}
	return call_done;
}

/* APM command: true if GPR_BASIC_RSP_RESULT says 0 (or another reply came) */
static bool
apm(uint32_t opcode, bool apm_hdr, const uint32_t *dw, uint32_t n)
{
	if (!gpr_call(1, 1, opcode, apm_hdr, dw, n, 1000)) {
		return false;
	}
	if (call_op == GPR_BASIC_RSP_RESULT && call_words >= 2 && call_rsp[1] != 0) {
		LOG("apm %08x: status %u", opcode, call_rsp[1]);
		return false;
	}
	return true;
}

/* PRM (module iid 2) PRM_CMD_REQUEST_HW_RSC */
static bool
prm(const uint32_t *dw, uint32_t n)
{
	if (!gpr_call(2, 2, 0x0100100Fu, true, dw, n, 1000)) {
		return false;
	}
	if (call_words >= 2 && call_rsp[1] != 0) {
		LOG("prm %08x: status %u", dw[1], call_rsp[1]);
		return false;
	}
	return true;
}

/* ---------------------------------------------------------------- codec (WCD937x AUX + AW87390 on rain) */

#define LPASS_PA            0x0A600000ull       /* RX/TX/VA macros, SoundWire masters, LPI TLMM, CGCRs */
#define LPASS_SIZE          0x00360000u
#define RX_MACRO            0x0A600000u
#define TX_MACRO            0x0A620000u
#define VA_MACRO            0x0A730000u
#define RX_SWR              0x0A610000u
#define VA_SWR              0x0A740000u
#define LPI_TLMM            0x0A7C0000u
#define TLMM_GPIO92_PA      0x0055C000ull       /* WCD937x reset, TLMM WEST */
#define AMP_I2C             0x58                /* rain: AW87390 on QUP0 SE1 (DT aw87xxx_pa_58@58; topaz: sia8159 0x2b) */

static volatile uint8_t *lpass, *gpio92;
static uint32_t swr_id;

static void
lw(uint32_t pa, uint32_t v)
{
	mb();
	*(volatile uint32_t *)(lpass + (pa - LPASS_PA)) = v;
	mb();
}

static uint32_t
lr(uint32_t pa)
{
	uint32_t v = *(volatile uint32_t *)(lpass + (pa - LPASS_PA));
	mb();
	return v;
}

/* SoundWire command FIFO write: SWR_REG_VAL_PACK reg | id << 16 | dev << 20 | data << 24 */
static void
swr_wr(uint32_t master, uint32_t dev, uint32_t reg, uint32_t val)
{
	swr_id = (swr_id % 14) + 1;
	lw(master + 0x300, (val << 24) | (dev << 20) | (swr_id << 16) | reg);
	IOSleep(1);
}

static void
swr_master_init(uint32_t m)
{
	lw(m + 0x008, 1); lw(m + 0x008, 1); lw(m + 0x1044, 1); IOSleep(2);
	lw(m + 0x101C, 0x2F0008); lw(m + 0x500, 1);
	uint32_t cfg = lr(m + 0x1048);
	lw(m + 0x1048, (cfg & ~0x3E0000u) | (0x1Fu << 17));
	lw(m + 0x314, 3); lw(m + 0x1044, 2);
	lw(m + 0x04, 2); lw(m + 0x208, 0xFFFFFFFF); lw(m + 0x204, 0x1FDFD); lw(m + 0x210, 0x1FDFD);
	lw(m + 0x04, 3); lw(m + 0x314, 0x80000003);
}

/* RX macro registers from the stock Android dump while playing (topaz-woa rxtab.h) */
static const uint16_t rx_stock[][2] = {
	{ 0x070, 0x11 }, { 0x074, 0x20 }, { 0x080, 0x11 }, { 0x084, 0x20 }, { 0x090, 0x0c }, { 0x094, 0x0c },
	{ 0x098, 0x0c }, { 0x110, 0x08 }, { 0x144, 0x38 }, { 0x190, 0x07 }, { 0x200, 0x01 }, { 0x204, 0x03 },
	{ 0x208, 0x02 }, { 0x20c, 0x1c }, { 0x210, 0x39 }, { 0x214, 0x0c }, { 0x218, 0x0c }, { 0x21c, 0x01 },
	{ 0x228, 0x80 }, { 0x238, 0x07 }, { 0x244, 0x02 }, { 0x248, 0x9a }, { 0x24c, 0x10 }, { 0x284, 0x10 },
	{ 0x290, 0x04 }, { 0x294, 0xe0 }, { 0x298, 0x01 }, { 0x29c, 0x40 }, { 0x2a0, 0x2a }, { 0x2ac, 0x18 },
	{ 0x2b0, 0x18 }, { 0x2b4, 0x03 }, { 0x2b8, 0x01 }, { 0x2c4, 0x64 }, { 0x2c8, 0x01 }, { 0x2d8, 0x0c },
	{ 0x2e0, 0x77 }, { 0x2e4, 0x01 }, { 0x2ec, 0x4b }, { 0x2f4, 0x01 }, { 0x300, 0x04 }, { 0x304, 0x08 },
	{ 0x308, 0x0c }, { 0x30c, 0xe2 }, { 0x314, 0x04 }, { 0x318, 0x3e }, { 0x360, 0xff }, { 0x380, 0xff },
	{ 0x408, 0x64 }, { 0x40c, 0x8f }, { 0x410, 0x02 }, { 0x418, 0x04 }, { 0x41c, 0x7e }, { 0x424, 0x08 },
	{ 0x434, 0x02 }, { 0x438, 0x08 }, { 0x440, 0x08 }, { 0x450, 0x55 }, { 0x454, 0x55 }, { 0x458, 0x55 },
	{ 0x488, 0x64 }, { 0x48c, 0x8f }, { 0x490, 0x02 }, { 0x498, 0x04 }, { 0x49c, 0x7e }, { 0x4a4, 0x08 },
	{ 0x4b4, 0x02 }, { 0x4b8, 0x08 }, { 0x4c0, 0x08 }, { 0x4d0, 0x55 }, { 0x4d4, 0x55 }, { 0x4d8, 0x55 },
	{ 0x504, 0x18 }, { 0x508, 0x64 }, { 0x50c, 0x8f }, { 0x510, 0x02 }, { 0x518, 0x04 }, { 0x51c, 0x7e },
	{ 0x524, 0x04 }, { 0x528, 0x08 }, { 0x540, 0x02 }, { 0x544, 0x08 }, { 0x54c, 0x01 }, { 0x784, 0x07 },
	{ 0x788, 0x3c }, { 0x800, 0x60 }, { 0x804, 0xdb }, { 0x808, 0xff }, { 0x80c, 0x35 }, { 0x810, 0xff },
	{ 0x818, 0x01 }, { 0x81c, 0x28 }, { 0x840, 0x60 }, { 0x844, 0xdb }, { 0x848, 0xff }, { 0x84c, 0x35 },
	{ 0x850, 0xff }, { 0x858, 0x01 }, { 0x85c, 0x28 }, { 0xa24, 0x40 }, { 0xaa4, 0x40 }, { 0xb40, 0x04 },
	{ 0xb50, 0x04 }, { 0xc04, 0x01 }, { 0xc44, 0x01 }, { 0xc84, 0x01 }, { 0xd0c, 0xa8 }, { 0xd4c, 0xa8 },
	{ 0xd8c, 0xa8 }, { 0xf08, 0x62 }, { 0xf0c, 0x96 }, { 0xf88, 0x62 }, { 0xf8c, 0x96 },
};

static int vol_db = -30;
static bool codec_up, ports_up, amp_init;

/*
 * Once per boot, after SPF is ready: L14 (WCD937x VDD_BUCK) through the RPM, then topaz-woa
 * TopazSpeaker hw.c CodecBringUp(): PRM DCODEC vote + only the TX core/NPL clocks (RX and VA
 * run on the TX MCLK; asking for their own clocks left the SoundWire masters dead), LPI pins,
 * macros, both SoundWire masters with the codec in reset, SMMU identity for the audio DMA
 * stream 0x1C1, the stock RX macro registers, WCD937x init + AUX DAC.
 */
static bool
codec_bringup(void)
{
	static const uint32_t pins[6] = { 0xD04, 0xD06, 0xD06, 0xD04, 0xD06, 0xD06 };
	uint32_t dw[9];
	bool ok = true;

	for (uint32_t set = 0; set <= 1; set++) {         /* "ldoa" 14: "uv" 1.8 V, "swen" 1 */
		ok &= topaz_rpm_write(set, 0x616F646Cu, 14, 0x00007675u, 1800000);
		ok &= topaz_rpm_write(set, 0x616F646Cu, 14, 0x6E657773u, 1);
	}
	LOG("t=%u.%03u RPM L14 on: %s", T, ok ? "OK" : "FAILED");
	lpass = (volatile uint8_t *)ml_io_map(LPASS_PA, LPASS_SIZE);
	gpio92 = (volatile uint8_t *)ml_io_map(TLMM_GPIO92_PA, 0x1000);
	if (lpass == NULL || gpio92 == NULL) {
		LOG("LPASS map failed");
		return false;
	}
	/* 1. PRM: DCODEC HW vote, TX core + TX NPL clocks 19.2 MHz */
	dw[0] = 2; dw[1] = 0x08001032; dw[2] = 4; dw[3] = 0; dw[4] = 2;
	ok = prm(dw, 5);
	for (uint32_t i = 0; i < 2 && ok; i++) {
		dw[0] = 2; dw[1] = 0x0800102C; dw[2] = 20; dw[3] = 0; dw[4] = 1;
		dw[5] = 0x30C + i; dw[6] = 19200000; dw[7] = 1; dw[8] = 0;
		ok = prm(dw, 9);
	}
	LOG("t=%u.%03u PRM vote + clocks: %s", T, ok ? "OK" : "FAILED");
	if (!ok) {
		return false;
	}
	/* 2. LPI pins (SWR clk/data, func1) + slew */
	for (uint32_t i = 0; i < 6; i++) {
		lw(LPI_TLMM + 0x1000 * i, pins[i]);
	}
	lw(0x0A95A000, 0x3F3F);
	/* 3. VA macro fs-gen + SWR clock */
	lw(VA_MACRO, 1); lw(VA_MACRO + 4, 3); lw(VA_MACRO + 4, 1); lw(VA_MACRO + 0x80, 2);
	lw(VA_MACRO + 8, 2); lw(VA_MACRO + 8, 3); lw(VA_MACRO + 8, 1);
	/* 4. RX macro MCLK, FS, SWR clock; RX CGCR */
	lw(RX_MACRO + 0x100, 3); lw(RX_MACRO + 0x104, 3); lw(RX_MACRO + 0x104, 1);
	lw(RX_MACRO + 0x108, 2); lw(RX_MACRO + 0x108, 3); lw(RX_MACRO + 0x108, 1);
	lw(0x0A6A9098, 3); IOSleep(1); lw(0x0A6A9098, 1);
	/* 5. TX macro MCLK, FS, SWR clock; VA/TX CGCR */
	lw(TX_MACRO, 1); lw(TX_MACRO + 4, 3); lw(TX_MACRO + 4, 1);
	lw(TX_MACRO + 8, 2); lw(TX_MACRO + 8, 3); lw(TX_MACRO + 8, 1);
	lw(0x0A7EC100, 3); IOSleep(1); lw(0x0A7EC100, 1);
	/* 6. WCD937x reset (gpio92) low while both masters start, then high */
	*(volatile uint32_t *)(gpio92 + 4) = 0; mb();
	*(volatile uint32_t *)(gpio92 + 0) = 0x200; mb();
	swr_master_init(RX_SWR);
	swr_master_init(VA_SWR);
	IOSleep(20);
	*(volatile uint32_t *)(gpio92 + 4) = 2; mb();
	IOSleep(300);
	LOG("t=%u.%03u SoundWire: slaves rx %x va %x, ids %08x %08x", T, lr(RX_SWR + 0x1090), lr(VA_SWR + 0x1090),
	    lr(RX_SWR + 0x538), lr(VA_SWR + 0x538));
	/* 7. audio DMA (apps SMMU stream 0x1C1) passes addresses through */
	if (!topaz_smmu_identity(0x1C1, 0, "audio")) {
		return false;
	}
	/* 8. RX macro as on stock; RX0..2 paths muted */
	for (uint32_t i = 0; i < sizeof(rx_stock) / sizeof(rx_stock[0]); i++) {
		lw(RX_MACRO + rx_stock[i][0], rx_stock[i][1]);
	}
	lw(RX_MACRO + 0x400, 4); lw(RX_MACRO + 0x480, 4); lw(RX_MACRO + 0x500, 4);
	/* 9. WCD937x init + RX clocks + AUX DAC (its TX slave on the VA bus, dev 1) */
	swr_wr(VA_SWR, 1, 0x3103, 0xD6); IOSleep(2);
	swr_wr(VA_SWR, 1, 0x312A, 0x00);
	swr_wr(VA_SWR, 1, 0x3029, 0x85);
	swr_wr(VA_SWR, 1, 0x3001, 0xC0); IOSleep(10); swr_wr(VA_SWR, 1, 0x3001, 0x80);
	swr_wr(VA_SWR, 1, 0x30E2, 0xD9);
	swr_wr(VA_SWR, 1, 0x30A5, 0xEB);
	swr_wr(VA_SWR, 1, 0x3409, 0x08); swr_wr(VA_SWR, 1, 0x3408, 0x01); swr_wr(VA_SWR, 1, 0x3008, 0x01);
	swr_wr(VA_SWR, 1, 0x340D, 0xBC); swr_wr(VA_SWR, 1, 0x340E, 0xBC); swr_wr(VA_SWR, 1, 0x340F, 0xBC);
	swr_wr(VA_SWR, 1, 0x3408, 0x03);
	swr_wr(VA_SWR, 1, 0x3408, 0x07); swr_wr(VA_SWR, 1, 0x3409, 0x0C); swr_wr(VA_SWR, 1, 0x344F, 0x01);
	swr_wr(VA_SWR, 1, 0x3009, 0x0C); swr_wr(VA_SWR, 1, 0x300A, 0x40); swr_wr(VA_SWR, 1, 0x300B, 0x12);
	swr_wr(VA_SWR, 1, 0x3008, 0x41); swr_wr(VA_SWR, 1, 0x3008, 0xC1);
	lw(RX_MACRO + 0x514, (uint32_t)vol_db & 0xFF);      /* RX_RX2_RX_VOL_CTL, s8 dB */
	codec_up = true;
	LOG("t=%u.%03u codec ready", T);
	return true;
}

/* SoundWire data ports + WCD937x AUX PA, after the first graph start (ports.ps1, wcd_auxpa.ps1) */
static void
ports_and_pa(void)
{
	static const uint32_t slv[][2] = { { 0x203, 7 }, { 0x230, 1 }, { 0x232, 0x1F }, { 0x233, 0 }, { 0x234, 0 },
		                           { 0x236, 0x36 }, { 0x430, 1 }, { 0x432, 7 }, { 0x433, 0 }, { 0x434, 1 } };

	lw(RX_SWR + 0x1264, 0x0100001F); lw(RX_SWR + 0x127C, 0); lw(RX_SWR + 0x122C, 7); lw(RX_SWR + 0x1274, 0x63);
	lw(RX_SWR + 0x1464, 0x01000107); lw(RX_SWR + 0x147C, 0); lw(RX_SWR + 0x1474, 0);
	for (uint32_t i = 0; i < sizeof(slv) / sizeof(slv[0]); i++) {
		swr_wr(RX_SWR, 1, slv[i][0], slv[i][1]);
	}
	lw(RX_SWR + 0x105C, 0x5000F);
	lw(RX_SWR + 0x300, (0x0Fu << 24) | (15u << 20) | (15u << 16) | 0x70);   /* broadcast bank switch */
	IOSleep(5);
	swr_wr(VA_SWR, 1, 0x3467, 0x05);
	swr_wr(VA_SWR, 1, 0x3128, 0x80);
	swr_wr(VA_SWR, 1, 0x346B, 0x4C);
	swr_wr(VA_SWR, 1, 0x346C, 0x7F);
	swr_wr(VA_SWR, 1, 0x346D, 0x0F);
	LOG("t=%u.%03u ports: rx frame b1 %08x, irq %08x", T, lr(RX_SWR + 0x105C), lr(RX_SWR + 0x200));
}

/*
 * rain: the speaker amp is an Awinic AW87390 (chip id 0x76 at register 0). Its settings are
 * profile 0 ("Music" in Awinic's aw87xxx profile enum) of the stock /vendor/firmware/aw87xxx_acf.bin:
 * register/value pairs, the last one (0x01 = 0x07) switches it on. Off: SYSCTRL (0x01) = 0
 * (Linux aw87390.c). Byte pairs at register 0xfe/0xff are delays/terminators there: skipped.
 */
static void
amp_on(bool on)
{
	static const uint8_t music[][2] = {
		{ 0x67, 0x23 }, { 0x02, 0x07 }, { 0x02, 0x00 }, { 0x02, 0x1c }, { 0x03, 0x08 }, { 0x04, 0x05 },
		{ 0x05, 0x0a }, { 0x06, 0x0e }, { 0x07, 0x4e }, { 0x08, 0x09 }, { 0x09, 0x08 }, { 0x0a, 0x4b },
		{ 0x61, 0xb3 }, { 0x62, 0x24 }, { 0x63, 0x09 }, { 0x64, 0x24 }, { 0x65, 0x15 }, { 0x79, 0x7a },
		{ 0x7a, 0x6c }, { 0x78, 0x80 }, { 0x66, 0x30 }, { 0x78, 0x00 }, { 0x68, 0x1b }, { 0x69, 0x5b },
		{ 0x70, 0x1c }, { 0x71, 0x00 }, { 0x72, 0xfe }, { 0x73, 0x4f }, { 0x74, 0x24 }, { 0x75, 0x02 },
		{ 0x01, 0x07 },
	};
	int err = 0;
	uint8_t id = 0;

	if (on) {
		err = amp_init ? 0 : sweet_i2c1_init();
		if (err == 0 && !amp_init) {
			err = sweet_i2c1_read(AMP_I2C, 0x00, &id, 1);
			if (err == 0 && id != 0x76) {
				LOG("t=%u.%03u amp: chip id %#x is not an AW87390: left alone", T, id);
				return;
			}
			amp_init = err == 0;
		}
		err = err ? err : sweet_i2c1_write(AMP_I2C, 0x01, 0x00);          /* power down, then the profile */
		for (uint32_t i = 0; i < sizeof(music) / sizeof(music[0]) && err == 0; i++) {
			err = sweet_i2c1_write(AMP_I2C, music[i][0], music[i][1]);
		}
	} else if (amp_init) {
		err = sweet_i2c1_write(AMP_I2C, 0x01, 0x00);
	}
	LOG("t=%u.%03u amp %s: %d (id %#x)", T, on ? "on" : "off", err, id);
}

/* ---------------------------------------------------------------- AudioReach graph */

#define SG                  0x4001
#define CONT                0x4101
#define DMA                 0x7001
#define PULL                0x7002
#define APM_GRAPH_OPEN      0x01001000u
#define APM_GRAPH_PREPARE   0x01001001u
#define APM_GRAPH_START     0x01001002u
#define APM_GRAPH_STOP      0x01001003u
#define APM_GRAPH_CLOSE     0x01001004u
#define APM_SET_CFG         0x01001006u
#define APM_MEM_MAP         0x0100100Cu
#define APM_MEM_UNMAP       0x0100100Du
#define APM_RSP_MEM_MAP     0x02001001u

static uint32_t ring_map, pos_map;

/* APM_CMD_SHARED_MEM_MAP_REGIONS, raw (no apm_cmd_header); msw 1 = the SMMU SID offset */
static bool
mem_map(uint64_t pa, uint32_t bytes, uint32_t prop, uint32_t *handle)
{
	uint32_t base = (uint32_t)pa & ~0xFFFu, end = ((uint32_t)pa + bytes + 0xFFF) & ~0xFFFu;
	uint32_t dw[5] = { 3 | (1u << 16), prop, base, 1, end - base };

	*handle = 0;
	if (!gpr_call(1, 1, APM_MEM_MAP, false, dw, 5, 1000) || call_op != APM_RSP_MEM_MAP || call_words < 1) {
		LOG("mem map %x: reply %08x status %u", base, call_op, call_words >= 2 ? call_rsp[1] : 0);
		return false;
	}
	*handle = call_rsp[0];
	return true;
}

static bool
sg_cmd(uint32_t opcode)
{
	uint32_t dw[6] = { 1, 0x08001005, 8, 0, 1, SG };
	return apm(opcode, true, dw, 6);
}

/*
 * Graph SH_MEM_PULL_MODE (the DSP reads the ring itself) -> CODEC_DMA_SINK on RX_CODEC_DMA_RX_1
 * (AIF2_PB -> RX2 -> INT2 -> AUX), 48 kHz 16-bit stereo; TopazSpeaker HwStreamPrepare().
 */
static bool
graph_open(uint64_t ring_pa, uint32_t ring_bytes, uint64_t pos_pa)
{
	static const uint32_t open[] = {
		1, 0x08001001, 48, 0, 1, SG, 3, 0x0800100E, 4, 2, 0x0800100F, 4, 1, 0x08001010, 4, 1,
		1, 0x08001000, 64, 0, 1, CONT, 4, 0x08001011, 8, 1, 3, 0x08001012, 4, 1, 0x08001013, 4, 8192, 0x08001014, 4, 2,
		1, 0x08001002, 32, 0, 1, SG, CONT, 2, 0x07001006, PULL, 0x07001023, DMA,
		1, 0x08001003, 56, 0, 2, PULL, 1, 0x08001015, 8, 0, 1, DMA, 1, 0x08001015, 8, 1, 0, 0,
		1, 0x08001004, 24, 0, 1, PULL, 1, DMA, 2, 0
	};
	const uint32_t ch = 2, rate = 48000;
	uint32_t mf[12] = { PULL, 0x0800100C, 32, 0, 1, 0x09001000, 20, rate, 16 | (1u << 16), 16 | (15u << 16),
		            1 | (ch << 16), 1 | (2u << 8) };
	bool ok = mem_map(ring_pa, ring_bytes, 0, &ring_map) && mem_map(pos_pa, 0x1000, 2, &pos_map) &&
	    apm(APM_GRAPH_OPEN, true, open, sizeof(open) / 4);
	uint32_t pc[12] = { PULL, 0x0800100A, 28, 0, (uint32_t)ring_pa, 1, ring_bytes, ring_map,
		            (uint32_t)pos_pa, 1, pos_map, 0 };
	uint32_t ep[8] = { DMA, 0x08001017, 12, 0, rate, 16 | (ch << 16), 1, 0 };
	uint32_t intf[8] = { DMA, 0x08001063, 12, 0, 1, 2, 3, 0 };          /* RXTX, RX_1, channels 0x3 */

	ok = ok && apm(APM_SET_CFG, true, mf, 12) && apm(APM_SET_CFG, true, pc, 12) && apm(APM_SET_CFG, true, ep, 8) &&
	    apm(APM_SET_CFG, true, intf, 8) && sg_cmd(APM_GRAPH_PREPARE);
	LOG("t=%u.%03u graph open/prepare: %s (maps %x %x)", T, ok ? "OK" : "FAILED", ring_map, pos_map);
	return ok;
}

/* boot-arg adsp_tone=N: N seconds of 1 kHz from the speaker, once the codec is up */
static void
tone_test(unsigned seconds)
{
	const uint32_t frames = 48000, bytes = frames * 4;  /* 1 s, a whole number of 1 kHz periods */
	uint64_t ring_pa = 0, pos_pa = 0;
	int16_t *ring = (int16_t *)topaz_pool_alloc(bytes, &ring_pa);
	volatile uint32_t *pos = (volatile uint32_t *)topaz_pool_alloc(0x1000, &pos_pa);
	/* sin(2 pi k / 48) * 0.1 * 32767, one period */
	static const int16_t q[13] = { 0, 428, 848, 1253, 1638, 1997, 2317, 2604, 2838, 3020, 3165, 3240, 3277 };

	if (ring == NULL || pos == NULL) {
		LOG("tone: no pool memory");
		return;
	}
	for (uint32_t f = 0; f < frames; f++) {
		uint32_t k = f % 48, a = k % 24;
		int16_t v = a <= 12 ? q[a] : q[24 - a];
		v = k < 24 ? v : (int16_t)-v;
		ring[2 * f] = v;
		ring[2 * f + 1] = v;
	}
	mb();
	if (!graph_open(ring_pa, bytes, pos_pa) || !sg_cmd(APM_GRAPH_START)) {
		LOG("tone: graph failed");
		return;
	}
	if (!ports_up) {
		ports_and_pa();
		ports_up = true;
	}
	amp_on(true);
	lw(RX_MACRO + 0x514, (uint32_t)vol_db & 0xFF);
	lw(RX_MACRO + 0x518, 0x04);
	lw(RX_MACRO + 0x500, 0x24);                         /* RX2 path on */
	LOG("t=%u.%03u tone: playing %u s at %d dB", T, seconds, vol_db);
	for (unsigned t = ms(); ms() - t < seconds * 1000;) {
		if (!glink_poll()) {
			IOSleep(2);
		}
	}
	LOG("t=%u.%03u tone: DSP position frame %u index %u", T, pos[0], pos[1]);
	amp_on(false);
	lw(RX_MACRO + 0x500, 0x04);
	(void)sg_cmd(APM_GRAPH_STOP);
	(void)sg_cmd(APM_GRAPH_CLOSE);
	LOG("t=%u.%03u tone: stopped", T);
}

/* ---------------------------------------------------------------- the stream iOS plays into */

/*
 * One graph for the session, opened after the codec bring-up: the DSP pulls a 1 s ring of
 * 48 kHz 16-bit stereo from the pool. /dev/topaz-audio write() puts frames ahead of the DSP's
 * read index (pos buffer word 1, bytes) and never waits; the audio thread starts the graph,
 * the SoundWire ports, the amp and the RX2 path when frames come, and stops them 0.5 s after
 * the last write. ioctl _IOW('r', 1, {below, muted}): the volume, eighths of a dB under the
 * loudest (boot-arg adsp_vol, dB of the RX2 digital gain).
 */
#define RING_BYTES          (48000u * 4)
#define LEAD_BYTES          (48u * 4 * 40)          /* 40 ms ahead of the DSP */
#define QUIET_MS            500

static uint8_t *ring;
static volatile uint32_t *pos_buf;
static uint64_t ring_pa, pos_pa;
static bool stream_ready, stream_ran;
static volatile bool playing;
static volatile uint32_t write_off;
static volatile uint64_t last_write;            /* mach_absolute_time() */
static volatile uint32_t vol_below, vol_muted, vol_gen, vol_applied;
static uint64_t frames_in, frames_dropped, underruns;
static IOLock *stream_lock;
static bool stream_wake_flag;

static bool
stream_open(void)
{
	ring = (uint8_t *)topaz_pool_alloc(RING_BYTES, &ring_pa);
	pos_buf = (volatile uint32_t *)topaz_pool_alloc(0x1000, &pos_pa);
	stream_lock = IOLockAlloc();
	if (ring == NULL || pos_buf == NULL || stream_lock == NULL) {
		LOG("stream: no memory");
		return false;
	}
	stream_ready = graph_open(ring_pa, RING_BYTES, pos_pa);
	return stream_ready;
}

static void
apply_volume(void)
{
	int db = vol_muted ? -84 : vol_db - (int)(vol_below / 8);
	if (db < -84) {
		db = -84;
	}
	lw(RX_MACRO + 0x514, (uint32_t)db & 0xFF);          /* RX_RX2_RX_VOL_CTL, s8 dB */
	vol_applied = vol_gen;
}

static void
stream_start(void)
{
	if (stream_ran) {
		(void)sg_cmd(APM_GRAPH_PREPARE);
	}
	if (!sg_cmd(APM_GRAPH_START)) {
		LOG("stream: start failed");
		return;
	}
	stream_ran = true;
	if (!ports_up) {
		ports_and_pa();
		ports_up = true;
	}
	amp_on(true);
	apply_volume();
	lw(RX_MACRO + 0x518, 0x04);
	lw(RX_MACRO + 0x500, 0x24);                         /* RX2 path on */
	playing = true;
	LOG("t=%u.%03u stream: playing", T);
}

static void
stream_stop(void)
{
	playing = false;
	lw(RX_MACRO + 0x514, (uint32_t)-84 & 0xFF);         /* mute first, then the amp: no pop */
	IOSleep(30);
	amp_on(false);
	IOSleep(10);
	lw(RX_MACRO + 0x500, 0x04);
	(void)sg_cmd(APM_GRAPH_STOP);
	bzero(ring, RING_BYTES);
	LOG("t=%u.%03u stream: stopped (%llu frames in, %llu dropped, %llu underruns)", T, frames_in, frames_dropped,
	    underruns);
}

/* the audio thread: start / stop / volume, between GLINK polls */
static void
stream_poll(void)
{
	uint64_t now = mach_absolute_time(), quiet;

	if (!stream_ready) {
		return;
	}
	nanoseconds_to_absolutetime((uint64_t)QUIET_MS * 1000000, &quiet);
	if (!playing && last_write != 0 && now - last_write < quiet) {
		stream_start();
	} else if (playing && now - last_write >= quiet) {
		stream_stop();
	}
	if (playing && vol_applied != vol_gen) {
		apply_volume();
	}
}

/* frames from /dev/topaz-audio: never waits; returns how many were taken */
extern "C" uint32_t
topaz_audio_write(const int16_t *frames, uint32_t count)
{
	if (!stream_ready || count == 0) {
		return 0;
	}
	IOLockLock(stream_lock);
	uint32_t bytes = MIN(count * 4, RING_BYTES / 2);
	uint32_t r = playing ? (pos_buf[1] % RING_BYTES) & ~3u : 0;
	uint32_t w = write_off;
	uint32_t ahead = (w + RING_BYTES - r) % RING_BYTES;
	if (!playing || ahead > RING_BYTES / 2) {           /* not started yet, or the DSP passed us */
		if (playing) {
			underruns++;
		}
		w = (r + LEAD_BYTES) % RING_BYTES;
		ahead = LEAD_BYTES;
	}
	if (ahead + bytes > RING_BYTES - 4096) {            /* too far ahead: drop the rest */
		uint32_t room = RING_BYTES - 4096 > ahead ? (RING_BYTES - 4096 - ahead) & ~3u : 0;
		frames_dropped += (bytes - room) / 4;
		bytes = room;
	}
	uint32_t n1 = MIN(bytes, RING_BYTES - w);
	memcpy(ring + w, frames, n1);
	memcpy(ring, (const uint8_t *)frames + n1, bytes - n1);
	/* silence after the data: when iOS stops writing the DSP must not replay the ring (a click at the end) */
	uint32_t z = (w + bytes) % RING_BYTES, zl = 2 * LEAD_BYTES, z1 = MIN(zl, RING_BYTES - z);
	bzero(ring + z, z1);
	bzero(ring, zl - z1);
	mb();
	write_off = (w + bytes) % RING_BYTES;
	frames_in += bytes / 4;
	last_write = mach_absolute_time();
	IOLockUnlock(stream_lock);
	if (!playing) {
		thread_wakeup((event_t)&stream_wake_flag);
	}
	return count;
}

extern "C" void
topaz_audio_volume(uint32_t below, uint32_t muted)
{
	vol_below = below;
	vol_muted = muted;
	vol_gen++;
}

/* ---------------------------------------------------------------- /dev/topaz-audio */

#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/ioccom.h>
#include <miscfs/devfs/devfs.h>

struct topaz_audio_volume_arg { uint32_t below, muted; };
#define TOPAZ_AUDIO_SET_VOLUME  _IOW('r', 1, struct topaz_audio_volume_arg)

static int
ta_open(dev_t, int, int, struct proc *)
{
	return 0;
}

static int
ta_close(dev_t, int, int, struct proc *)
{
	return 0;
}

static int
ta_write(dev_t, struct uio *uio, int)
{
	int16_t buf[1024 * 2];

	while (uio_resid(uio) >= 4) {
		user_ssize_t n = MIN(uio_resid(uio), (user_ssize_t)sizeof(buf)) & ~3;
		int err = uiomove((char *)buf, (int)n, uio);
		if (err != 0) {
			return err;
		}
		(void)topaz_audio_write(buf, (uint32_t)(n / 4));
	}
	return 0;
}

static int
ta_ioctl(dev_t, u_long cmd, caddr_t data, int, struct proc *)
{
	if (cmd != TOPAZ_AUDIO_SET_VOLUME) {
		return ENOTTY;
	}
	const struct topaz_audio_volume_arg *v = (const struct topaz_audio_volume_arg *)data;
	topaz_audio_volume(v->below, v->muted);
	return 0;
}

static const struct cdevsw ta_cdevsw = {
	.d_open = ta_open, .d_close = ta_close, .d_read = eno_rdwrt, .d_write = ta_write, .d_ioctl = ta_ioctl,
	.d_stop = eno_stop, .d_reset = eno_reset, .d_ttys = NULL, .d_select = eno_select, .d_mmap = eno_mmap,
	.d_strategy = eno_strat, .d_reserved_1 = eno_getc, .d_reserved_2 = eno_putc, .d_type = 0,
};

/* a private patch may add a second name for the same device (a strong definition wins) */
extern "C" __attribute__((weak)) void
topaz_audio_dev_alias(int major __unused)
{
}

static void
audio_dev_init(void)
{
	int major = cdevsw_add(-1, &ta_cdevsw);
	if (major < 0) {
		LOG("/dev/topaz-audio: no major");
		return;
	}
	void *node = devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "topaz-audio");
	LOG("/dev/topaz-audio: %s", node != NULL ? "ready" : "devfs failed");
	if (node != NULL) {
		topaz_audio_dev_alias(major);
	}
}

/* ---------------------------------------------------------------- thread */

static void
topaz_audio_thread(void *arg __unused, wait_result_t wr __unused)
{
	uint32_t st;
	unsigned last_status = 0;
	bool codec_tried = false;

	if (!topaz_soc_map()) {
		LOG("no SoC mappings (see TopazModem)");
		goto out;
	}
	part = topaz_smem_partition(0, ADSP_PID);
	LOG("apps<->adsp SMEM partition %p", part);
	if (part == NULL) {
		goto out;
	}
	crash_reason();
	if (topaz_smem_get(part, ITEM_SMP2P_IN, NULL) != NULL || topaz_smem_get(part, ITEM_GLINK_FIFO_RX, NULL) != NULL) {
		LOG("ADSP state from an earlier run is in SMEM (item %u or %u): not booting it again", ITEM_SMP2P_IN,
		    ITEM_GLINK_FIFO_RX);
		goto out;
	}
	if (!smp2p_out_init()) {
		LOG("SMP2P item %u: no SMEM", ITEM_SMP2P_OUT);
		goto out;
	}
	*stop_entry = 0;
	topaz_apcs_kick(ADSP_SMP2P_BIT);
	t0 = mach_absolute_time();
	if (!adsp_load()) {
		goto out;
	}
	t0 = mach_absolute_time();
	st = wait_ready(20000);
	if ((st & 3) != 2) {
		LOG("ADSP did not report READY (slave-kernel %08x)", st);
		crash_reason();
		goto out;
	}
	if (!glink_init()) {
		goto out;
	}
	glink_register(QCHAN, qrtr_rx);
	glink_register(GCHAN, gpr_rx);
	for (;;) {
		bool busy = glink_poll();
		qrtr_poll();
		gpr_poll();
		stream_poll();
		if (spf_state == 1 && !codec_tried) {
			uint32_t tone = 0;
			codec_tried = true;
			(void)PE_parse_boot_argn("adsp_vol", &vol_db, sizeof(vol_db));
			if (codec_bringup()) {
				if (PE_parse_boot_argn("adsp_tone", &tone, sizeof(tone)) && tone != 0) {
					tone_test(MIN(tone, 10u));
				}
				if (stream_open()) {
					audio_dev_init();
				}
			}
		}
		if (*adsp_state & 1) {
			LOG("t=%u.%03u *** ADSP FATAL (slave-kernel %08x) ***", T, *adsp_state);
			crash_reason();
			break;
		}
		if (ms() - last_status >= 60000) {
			last_status = ms();
			LOG("t=%u.%03u status: slave-kernel %08x, qrtr node %x hello %u, %u services, %u lookups, pd-mapper %u, "
			    "SPF %d", T, *adsp_state, q_node, q_hello_rx, q_services, q_lookups, q_pdreqs, (int)spf_state);
			glink_summary();
		}
		if (busy) {
			IODelay(200);
		} else if (stream_ready && !playing) {
			assert_wait_timeout((event_t)&stream_wake_flag, THREAD_UNINT, 20, 1000 * NSEC_PER_USEC);
			thread_block(THREAD_CONTINUE_NULL);
		} else {
			IOSleep(2);
		}
	}
out:
	thread_terminate(current_thread());
}

extern "C" void
topaz_audio_init(void)
{
	static bool started;
	uint32_t on = 0;
	thread_t thread;

	if (started || !PE_parse_boot_argn("adsp", &on, sizeof(on)) || on == 0) {
		return;
	}
	started = true;
	if (kernel_thread_start(topaz_audio_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

#else
extern "C" void topaz_audio_init(void) {}
#endif
