/*
 * topaz: the Adreno 610 of the Redmi Note 12 4G (SM6225 "khaje") for programs
 * to draw with, behind the hardware interface (redwood_gpu_*) that
 * RedwoodGPU.cpp's msm DRM user client speaks (from vcvkk/darwin-redwood,
 * HackiOS; reused with the author's permission).
 *
 * The hardware sequences are MakrSas/topaz-woa drivers/TopazGpuW/hw.c, proven
 * under Windows with Mesa freedreno: power (GCC branches, CX and GX GDSCs,
 * gfx3d from GPLL0), the zap shader through TrustZone (PAS 13), the GPU SMMU
 * with stage-1 tables (4 KB granule, 4 GB GPU VA, one context bank), CP start
 * (SQE microcode, ME_INIT, SET_SECURE_MODE 0), IB submission and fences. The
 * A610 has no GMU. Under Windows GPU memory stayed below 4 GiB; here the
 * kernel has no RAM below 4 GiB at all, and the SMMU maps 36-bit addresses.
 *
 * Boot-arg gpu=1: a thread waits for the root filesystem, loads the firmware
 * (/usr/share/firmware/topaz/{a610_zap.mdt,a610_zap.b02,a630_sqe.fw}), starts
 * the GPU and publishes the RedwoodGPU service. gpu_mhz=600 runs the core at
 * 600 MHz instead of 300.
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLocks.h>
#include <pexpert/pexpert.h>
#include <kern/thread.h>
#include <mach/mach_time.h>

extern "C" {
vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
void sweet_dcache_clean(vm_offset_t addr, unsigned length);
void sweet_dcache_flush(vm_offset_t addr, unsigned length);   /* osfmk/arm/caches.c: real dc civac */
int topaz_fw_read(const char *path, void **data, size_t *size);
void topaz_fw_free(void *data, size_t size);
int topaz_root_mounted(void);
void RedwoodGPUStart(void);
void topaz_adreno_init(void);
extern const uint8_t topaz_fw_zap_mdt[], topaz_fw_zap_b02[], topaz_fw_sqe_fw[];
extern const size_t topaz_fw_zap_mdt_size, topaz_fw_zap_b02_size, topaz_fw_sqe_fw_size;

bool redwood_gpu_ready(void);
bool redwood_gpu_parameter(uint32_t which, uint64_t *value);
uint64_t redwood_gpu_address(uint64_t size);
void redwood_gpu_address_free(uint64_t address, uint64_t size);
bool redwood_gpu_map(uint64_t address, uint64_t phys, bool read_only);
void redwood_gpu_mapped(void);
void redwood_gpu_unmap(uint64_t address, uint64_t size);
void redwood_gpu_to_memory(void *start, uint64_t size);
uint32_t redwood_gpu_submit(const uint64_t *addresses, const uint32_t *words, uint32_t count);
bool redwood_gpu_done(uint32_t fence);
uint32_t redwood_gpu_last(void);
void redwood_gpu_worked(uint64_t *microseconds, uint32_t *submissions, uint32_t by[7]);
void redwood_gpu_report(const char *why);
void redwood_gpu_stop(void);
void redwood_tl(const char *format, ...) __printflike(1, 2);
uint32_t gpu_log;
}

#define GCC_BASE        0x01400000ull
#define GCC_SIZE        0x80000
#define GPUCC_BASE      0x05990000ull
#define GPUCC_SIZE      0x9000
#define GPU_BASE        0x05900000ull
#define GPU_SIZE        0x40000
#define SMMU_BASE       0x059A0000ull
#define SMMU_SIZE       0x10000
#define SMMU_CB0        0x8000
#define ZAP_REGION      0x56615000ull     /* rain: pil_gpu_mem (topaz: 0x55B15000) */
#define ZAP_REGION_SIZE 0x2000
#define GPU_PAS_ID      13

#define CBCR_EN         (1u << 0)
#define CBCR_OFF        (1u << 31)
#define GDSC_COLLAPSE   (1u << 0)
#define GDSC_PWR_ON     (1u << 31)
#define RCG_UPDATE      (1u << 0)
#define SRC_GPLL0       5

#define SCM_FN(svc, cmd)    (0x42000000u | ((uint32_t)(svc) << 8) | (uint32_t)(cmd))
#define SCM_SVC_PIL         0x02
#define PIL_INIT_IMAGE      0x01
#define PIL_MEM_SETUP       0x02
#define PIL_AUTH_RESET      0x05
#define PIL_SHUTDOWN        0x06
#define SCM_ARG_RW          2

#define RB_BYTES            0x8000
#define RB_DWORDS           (RB_BYTES / 4)
#define CP_INDIRECT_BUFFER  0x3f
#define CP_EVENT_WRITE      0x46
#define CP_ME_INIT          0x48
#define CP_SET_SECURE_MODE  0x66
#define CACHE_FLUSH_TS      4

#define VA_USER_START       0x01000000ull
#define VA_KERNEL           0xF0000000ull
#define GMEM_SIZE           0x21000             /* 128K + 4K (bengal), TopazGpuW */
#define GMEM_BASE           0x100000
/* topaz: all kernel RAM is the upper DDR bank (0x1_40000000+): the SMMU takes 36-bit output addresses */
#define LOW_MASK            0x0000000ffffff000ull

#define FW_DIR              "/usr/share/firmware/topaz/"

static volatile uint8_t *gcc, *gpucc, *gpu, *smmu;
static bool ready, failed, smmu_on;
static uint32_t wptr, seqno, core_mhz = 300;
static IOLock *hw_lock;

struct kbuf {
	IOBufferMemoryDescriptor *md;
	uint8_t *va;
	uint64_t pa, iova, size;
};
static struct kbuf ring, sqe, mem;
static uint64_t kernel_va_next = VA_KERNEL;

/* ---------------------------------------------------------------- MMIO */

static inline uint32_t rd(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void wr(volatile uint8_t *b, uint32_t o, uint32_t v)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(b + o) = v;
}
static inline void rmw(volatile uint8_t *b, uint32_t o, uint32_t clr, uint32_t set) { wr(b, o, (rd(b, o) & ~clr) | set); }
static inline void gpu_wr(uint32_t reg, uint32_t v) { wr(gpu, 4 * reg, v); }
static inline uint32_t gpu_rd(uint32_t reg) { return rd(gpu, 4 * reg); }
static inline void gpu_wr64(uint32_t reg, uint64_t v) { gpu_wr(reg, (uint32_t)v); gpu_wr(reg + 1, (uint32_t)(v >> 32)); }

static bool
poll(volatile uint8_t *b, uint32_t o, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t i = 0; i <= us; i++) {
		if ((rd(b, o) & mask) == want) {
			return true;
		}
		IODelay(1);
	}
	return false;
}

/* ---------------------------------------------------------------- memory below 4 GiB */

/*
 * Ordinary (not contiguous) kernel memory: contiguous allocations come from the
 * lower DDR bank here, which the kernel cannot map back (phystokv panics). The
 * SMMU maps every 4 KB page on its own (kbuf_map); b->pa is only meaningful
 * within one 16 KB page (page tables, the zap metadata).
 */
static uint64_t
kbuf_pa(struct kbuf *b, uint64_t off)
{
	return b->md != NULL ? b->md->getPhysicalSegment(off, NULL, kIOMemoryMapperNone) : b->pa + off;
}

/*
 * The GPU (its SMMU's walks and its fetches) does not reach DRAM above 4 GiB,
 * as TopazGpuW found under Windows, and this kernel only manages the upper
 * bank. The middle bank (0x80000000-0x13fffffff) is RAM nothing here uses (no
 * reserved regions in the stock DT, not given to the kernel): a pool of it is
 * the GPU's, mapped write-combined (Normal non-cacheable: no cache upkeep).
 */
#define GPU_POOL_BASE       0x90000000ull
/*
 * rain: 768 MB, 0x90000000-0xC0000000, the rest of the 0x80000000 bank. The kernel's RAM is the
 * 0xC0000000 bank (mkadt.py --midmem with MIDBASE=0xc0000000 MIDSIZE=0x80000000); topaz's 1 GB
 * ran into it (pmap_enter_pv: unexpected PV head on a managed page).
 */
#define GPU_POOL_SIZE       0x30000000ull
#define GPU_POOL_UNIT       0x10000ull          /* 64 KB */
#define GPU_POOL_CHUNK      0x4000000ull        /* 64 MB: mapped on first use (1 GB at once is more IO space than there is) */
#define GPU_POOL_CHUNKS     (GPU_POOL_SIZE / GPU_POOL_CHUNK)
static uint8_t *gpu_chunk_va[GPU_POOL_CHUNKS];
static bool gpu_pool_ready;
static struct { uint64_t start, size; } pool_free[2048];
static unsigned pool_free_count;

static void
range_free(void *table, unsigned *count, unsigned max, uint64_t address, uint64_t size)
{
	struct r { uint64_t start, size; } *f = (struct r *)table;
	unsigned at;

	for (at = 0; at < *count && f[at].start < address; at++) {
	}
	if (at > 0 && f[at - 1].start + f[at - 1].size == address) {
		f[at - 1].size += size;
		if (at < *count && address + size == f[at].start) {
			f[at - 1].size += f[at].size;
			memmove(&f[at], &f[at + 1], (--*count - at) * sizeof(f[0]));
		}
	} else if (at < *count && address + size == f[at].start) {
		f[at].start = address;
		f[at].size += size;
	} else if (*count < max) {
		memmove(&f[at + 1], &f[at], (*count - at) * sizeof(f[0]));
		(*count)++;
		f[at].start = address;
		f[at].size = size;
	}
}

static uint64_t
range_alloc(void *table, unsigned *count, uint64_t size)
{
	struct r { uint64_t start, size; } *f = (struct r *)table;

	for (unsigned i = 0; i < *count; i++) {
		if (f[i].size >= size) {
			uint64_t address = f[i].start;
			f[i].start += size;
			f[i].size -= size;
			if (f[i].size == 0) {
				memmove(&f[i], &f[i + 1], (--*count - i) * sizeof(f[0]));
			}
			return address;
		}
	}
	return 0;
}

/*
 * A descriptor for pool memory. The cache mode goes into the descriptor itself:
 * for a task other than the kernel, physical memory is mapped through a device
 * pager whose cache mode is the descriptor's, and the default for RAM the
 * kernel does not manage is the IO one (Device memory: Mesa's dc zva and
 * unaligned accesses there were SIGBUS). The kIOMapWriteCombineCache option of
 * the mapping alone does not change it.
 */
extern "C" IOMemoryDescriptor *topaz_gpu_pool_md(uint64_t pa, uint64_t size);
IOMemoryDescriptor *
topaz_gpu_pool_md(uint64_t pa, uint64_t size)
{
	return IOMemoryDescriptor::withAddressRange(pa, size,
	           kIODirectionInOut | (kIOWriteCombineCache << kIOMemoryBufferCacheShift), TASK_NULL);
}

/* boot-arg gpu_midmem=1: the ADT gives the kernel RAM below 4 GiB (mkadt.py --midmem), GPU memory is the kernel's */
static uint32_t gpu_midmem;
extern "C" bool topaz_gpu_midmem(void);
bool
topaz_gpu_midmem(void)
{
	return gpu_midmem != 0;
}

/* Contiguous GPU memory below 4 GiB: physical address, kernel address (write-combined). 0: none left. */
extern "C" uint64_t topaz_gpu_pool_alloc(uint64_t size, void **va);
uint64_t
topaz_gpu_pool_alloc(uint64_t size, void **va)
{
	uint64_t pa;

	size = (size + GPU_POOL_UNIT - 1) & ~(GPU_POOL_UNIT - 1);
	if (size > GPU_POOL_CHUNK - GPU_POOL_UNIT) {
		return 0;
	}
	if (!gpu_pool_ready) {
		/* one range per chunk, a unit short so that neighbours never merge: no buffer crosses a chunk */
		for (uint64_t c = 0; c < GPU_POOL_CHUNKS; c++) {
			range_free(pool_free, &pool_free_count, 2048, GPU_POOL_BASE + c * GPU_POOL_CHUNK, GPU_POOL_CHUNK - GPU_POOL_UNIT);
		}
		gpu_pool_ready = true;
	}
	pa = range_alloc(pool_free, &pool_free_count, size);
	if (pa == 0) {
		return 0;
	}
	unsigned c = (unsigned)((pa - GPU_POOL_BASE) / GPU_POOL_CHUNK);
	if (gpu_chunk_va[c] == NULL) {
		/* in the kernel map, not the small IO submap ml_io_map uses (64 MB there failed) */
		IOMemoryDescriptor *md = topaz_gpu_pool_md(GPU_POOL_BASE + c * GPU_POOL_CHUNK, GPU_POOL_CHUNK);
		IOMemoryMap *map = md != NULL ? md->map(kIOMapWriteCombineCache) : NULL;
		if (map != NULL) {
			gpu_chunk_va[c] = (uint8_t *)map->getVirtualAddress();     /* (kept for good, with md) */
		}
		if (gpu_chunk_va[c] == NULL) {
			OSSafeReleaseNULL(map);
			OSSafeReleaseNULL(md);
			range_free(pool_free, &pool_free_count, 2048, pa, size);
			return 0;
		}
	}
	if (va != NULL) {
		*va = gpu_chunk_va[c] + (pa - GPU_POOL_BASE - c * GPU_POOL_CHUNK);
	}
	return pa;
}

static void pool_give_back(uint64_t pa, uint64_t size);
extern "C" void topaz_gpu_pool_free(uint64_t pa, uint64_t size);
void
topaz_gpu_pool_free(uint64_t pa, uint64_t size)
{
	pool_give_back(pa, size);
}

static void
pool_give_back(uint64_t pa, uint64_t size)
{
	size = (size + GPU_POOL_UNIT - 1) & ~(GPU_POOL_UNIT - 1);
	if (pa >= GPU_POOL_BASE && pa + size <= GPU_POOL_BASE + GPU_POOL_SIZE) {
		range_free(pool_free, &pool_free_count, 2048, pa, size);
	}
}
/* boot-arg gpu_identity (default 1): the SMMU passes addresses through (TopazGpuW G1), GPU VA = PA */
static uint32_t gpu_identity = 1;

static bool
kbuf_alloc_low(struct kbuf *b, uint64_t size)
{
	void *va = NULL;

	size = (size + PAGE_SIZE - 1) & ~(uint64_t)(PAGE_SIZE - 1);
	b->md = NULL;
	b->pa = topaz_gpu_pool_alloc(size, &va);
	if (b->pa == 0) {
		return false;
	}
	b->va = (uint8_t *)va;
	b->size = size;
	for (uint64_t i = 0; i < size; i += 8) {
		*(volatile uint64_t *)(b->va + i) = 0;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	return true;
}

/*
 * With the ADT's RAM from 0x80000000 (mkadt.py --midmem) the kernel has memory
 * below 4 GiB of its own: GPU memory is ordinary IOKit memory, contiguous (the
 * SMMU passes addresses through), below 4 GiB, write-combined.
 */
static bool
kbuf_alloc(struct kbuf *b, uint64_t size)
{
	size = (size + PAGE_SIZE - 1) & ~(uint64_t)(PAGE_SIZE - 1);
	if (!gpu_midmem) {
		return kbuf_alloc_low(b, size);         /* (without --midmem such allocations come from the lower bank: phystokv panics) */
	}
	b->md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous | kIOMapWriteCombineCache, size, 0x00000000fffff000ull);
	if (b->md == NULL) {
		return kbuf_alloc_low(b, size);         /* (an ADT without the middle bank: the pool) */
	}
	b->va = (uint8_t *)b->md->getBytesNoCopy();
	b->pa = b->md->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	b->size = size;
	bzero(b->va, size);
	return true;
}

__unused static bool
kbuf_alloc_high(struct kbuf *b, uint64_t size)
{
	size = (size + PAGE_SIZE - 1) & ~(uint64_t)(PAGE_SIZE - 1);
	/* no cache mode: IOBMD makes such buffers contiguous, from the lower DDR bank */
	b->md = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, size, PAGE_SIZE);
	if (b->md == NULL) {
		return false;
	}
	b->va = (uint8_t *)b->md->getBytesNoCopy();
	b->pa = kbuf_pa(b, 0);
	b->size = size;
	bzero(b->va, size);
	sweet_dcache_clean((vm_offset_t)b->va, (unsigned)size);
	return true;
}

/* ---------------------------------------------------------------- gpu_cc_pll0 (topaz-woa TopazGpuW v0.43) */

/*
 * Zonda PLL at GPU CC + 0, sequence from the stock khaje kernel (gpucc-khaje.c, clk-alpha-pll.c):
 * f = 19.2 MHz * (L + alpha / 2^16) at the stock OPPs. The GX voltage corner is not voted (RPM):
 * high rates run on whatever the bootloader left and may hang (gpu_mhz is opt-in).
 */
#define ZP_MODE        0x00
#define ZP_L           0x04
#define ZP_ALPHA       0x08
#define ZP_USER        0x0c
#define ZP_CFG         0x10
#define ZP_CFG_U       0x14
#define ZP_CFG_U1      0x18
#define ZP_OPMODE      0x28
#define ZP_BYPASSNL    (1u << 1)
#define ZP_RESET_N     (1u << 2)
#define ZP_OUTCTRL     (1u << 0)
#define ZP_LOCK        (1u << 31)

static bool
zonda_la(uint32_t mhz, uint32_t *l, uint32_t *a)
{
	static const uint32_t t[][3] = {
		{ 320, 17, 0xAAAB }, { 465, 24, 0x3800 }, { 600, 31, 0x4000 }, { 785, 41, 0xE2AA }, { 820, 43, 0xB555 },
		{ 980, 51, 0x0AAB }, { 1025, 53, 0x62AA }, { 1100, 57, 0x4AAB }, { 1260, 66, 0xA000 },
	};
	for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
		if (t[i][0] == mhz) {
			*l = t[i][1];
			*a = t[i][2];
			return true;
		}
	}
	return false;
}

static bool
gpu_pll_start(uint32_t mhz)
{
	uint32_t l, a;

	if (!zonda_la(mhz, &l, &a)) {
		IOLog("TopazGPU: gpu_mhz=%u is not a stock OPP (320 465 600 785 820 980 1025 1100 1260)\n", mhz);
		return false;
	}
	rmw(gpucc, ZP_MODE, ZP_OUTCTRL, 0);
	wr(gpucc, ZP_OPMODE, 0);
	wr(gpucc, ZP_L, l);
	wr(gpucc, ZP_ALPHA, a);
	wr(gpucc, ZP_CFG, 0x08200800);
	wr(gpucc, ZP_CFG_U, 0x05022001);
	wr(gpucc, ZP_CFG_U1, 0x00000010);
	wr(gpucc, ZP_USER, 0x01000101);
	rmw(gpucc, ZP_MODE, ZP_BYPASSNL, 0);
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	rmw(gpucc, ZP_MODE, 0, ZP_BYPASSNL);
	IODelay(1);
	rmw(gpucc, ZP_MODE, 0, ZP_RESET_N);
	wr(gpucc, ZP_OPMODE, 1);
	if (!poll(gpucc, ZP_MODE, ZP_LOCK, ZP_LOCK, 1000)) {
		IOLog("TopazGPU: gpu_cc_pll0: no lock at %u MHz (mode %08x)\n", mhz, rd(gpucc, ZP_MODE));
		return false;
	}
	rmw(gpucc, ZP_USER, 0, 0xF);
	rmw(gpucc, ZP_MODE, 0, ZP_OUTCTRL);
	IODelay(100);
	IOLog("TopazGPU: gpu_cc_pll0 locked at %u MHz (l %u alpha %04x)\n", mhz, l, a);
	return true;
}

/* ---------------------------------------------------------------- power (TopazGpuW PowerUp) */

static bool
branch_on(volatile uint8_t *b, uint32_t o, const char *name)
{
	rmw(b, o, 0, CBCR_EN);
	bool ok = poll(b, o, CBCR_OFF, 0, 200);
	if (!ok) {
		IOLog("TopazGPU: %s stays off (%08x)\n", name, rd(b, o));
	}
	return ok;
}

static bool
rcg_set(uint32_t cmd, uint32_t src, uint32_t div)
{
	wr(gpucc, cmd + 4, (src << 8) | (2 * div - 1));
	rmw(gpucc, cmd, 0, RCG_UPDATE);
	return poll(gpucc, cmd, RCG_UPDATE, 0, 500);
}

static bool
gdsc_on(uint32_t gdscr, uint32_t bcr, uint32_t clamp, uint32_t status)
{
	if (bcr != 0) {
		rmw(gpucc, bcr, 0, 1);
		IODelay(1);
		rmw(gpucc, bcr, 1, 0);
	}
	if (clamp != 0) {
		rmw(gpucc, clamp, 1, 0);
	}
	rmw(gpucc, gdscr, GDSC_COLLAPSE, 0);
	IODelay(1);
	return poll(gpucc, status, GDSC_PWR_ON, GDSC_PWR_ON, 500);
}

static void
power_cycle_if_on(void)
{
	if ((rd(gpucc, 0x100c) & GDSC_PWR_ON) == 0 && (rd(gpucc, 0x1540) & GDSC_PWR_ON) == 0) {
		return;
	}
	IOLog("TopazGPU: already powered (GX %08x CX %08x): power cycling\n", rd(gpucc, 0x100c), rd(gpucc, 0x1540));
	if (rd(gpucc, 0x100c) & GDSC_PWR_ON) {
		wr(gpu, 4 * 0x808, 0);                  /* CP_SQE_CNTL: stop */
	}
	rmw(gpucc, 0x1054, CBCR_EN, 0);
	rmw(gpucc, 0x100c, 0, GDSC_COLLAPSE);
	rmw(gpucc, 0x1508, 0, 1);
	poll(gpucc, 0x100c, GDSC_PWR_ON, 0, 1000);
	rmw(gpucc, 0x1098, CBCR_EN, 0);
	rmw(gpucc, 0x106c, 0, GDSC_COLLAPSE);
	IODelay(1000);
}

static bool
power_up(void)
{
	power_cycle_if_on();
	rmw(gcc, 0x79004, 0, 1u << 15);                 /* GPLL0 to the GPU CC */
	branch_on(gcc, 0x71154, "gcc_bimc_gpu_axi");
	branch_on(gcc, 0x3600c, "gcc_gpu_memnoc_gfx");
	branch_on(gcc, 0x36018, "gcc_gpu_snoc_dvm_gfx");
	if (!gdsc_on(0x106c, 0, 0, 0x1540)) {
		IOLog("TopazGPU: CX GDSC: no PWR_ON\n");
		return false;
	}
	if (!branch_on(gpucc, 0x109c, "gpucc_cxo") || !branch_on(gpucc, 0x1078, "gpucc_ahb")) {
		return false;
	}
	rcg_set(0x1120, SRC_GPLL0, 3);                  /* gmu 200 MHz */
	rmw(gpucc, 0x1098, 0xFF0, 0xFF0);
	branch_on(gpucc, 0x1098, "gpucc_cx_gmu");
	branch_on(gpucc, 0x5000, "gpucc_hlos1_vote_gpu_smmu");
	if (!gdsc_on(0x100c, 0x1008, 0x1508, 0x100c)) {
		IOLog("TopazGPU: GX GDSC: no PWR_ON\n");
		return false;
	}
	/* gfx3d: 300/600 MHz from GPLL0 (/2, /1), other stock OPPs from gpu_cc_pll0 (the GX corner is not voted) */
	if (core_mhz != 300 && core_mhz != 600 && gpu_pll_start(core_mhz)) {
		if (!rcg_set(0x101c, 1, 1)) {                   /* parent 1 = GPU_CC_PLL0_OUT_MAIN */
			return false;
		}
	} else {
		if (core_mhz != 300 && core_mhz != 600) {
			core_mhz = 300;
		}
		if (!rcg_set(0x101c, SRC_GPLL0, core_mhz >= 600 ? 1 : 2)) {
			return false;
		}
	}
	rmw(gpucc, 0x1054, 0, (1u << 14) | (1u << 13));
	if (!branch_on(gpucc, 0x1054, "gpucc_gx_gfx3d")) {
		return false;
	}
	branch_on(gpucc, 0x1060, "gpucc_gx_cxo");
	return true;
}

/* ---------------------------------------------------------------- TrustZone: the zap shader */

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

static uint64_t
scm(uint32_t fn, uint64_t arginfo, uint64_t a, uint64_t b, uint64_t c, uint64_t *res1)
{
	uint64_t args[8] = { fn, arginfo, a, b, c, 0, 0, 0 };
	smc(args);
	for (int n = 0; args[0] == 1 && n < 100000; n++) {     /* interrupted: resume with the session in x6 */
		uint64_t session = args[6];
		uint64_t again[8] = { 1, arginfo, a, b, c, 0, session, 0 };
		memcpy(args, again, sizeof(args));
		smc(args);
	}
	if (res1 != NULL) {
		*res1 = args[1];
	}
	return args[0];
}

static bool
zap_load(void)
{
	void *mdt = NULL, *seg = NULL;
	size_t mdt_size = 0, seg_size = 0;
	struct kbuf meta = {};
	bool ok = false;

	IOLog("TopazGPU: zap: embedded firmware (%zu + %zu bytes)\n", topaz_fw_zap_mdt_size, topaz_fw_zap_b02_size);
	mdt = (void *)topaz_fw_zap_mdt;
	mdt_size = topaz_fw_zap_mdt_size;
	seg = (void *)topaz_fw_zap_b02;
	seg_size = topaz_fw_zap_b02_size;
	{
		const uint8_t *m = (const uint8_t *)mdt;
		uint32_t phoff = *(const uint32_t *)(m + 28), phnum = *(const uint16_t *)(m + 44);
		uint32_t min_a = 0xffffffff, max_a = 0, seg_paddr = 0;
		uint64_t st, res = 0;

		for (uint32_t i = 0; i < phnum && phoff + 32 * (i + 1) <= mdt_size; i++) {
			const uint8_t *ph = m + phoff + 32 * i;
			uint32_t paddr = *(const uint32_t *)(ph + 12), memsz = *(const uint32_t *)(ph + 20);
			uint32_t flags = *(const uint32_t *)(ph + 24);
			if (*(const uint32_t *)ph == 1 && memsz != 0 && ((flags >> 24) & 7) != 2) {
				min_a = paddr < min_a ? paddr : min_a;
				uint32_t end = (paddr + memsz + 0xfff) & ~0xfffu;
				max_a = end > max_a ? end : max_a;
				seg_paddr = paddr;
			}
		}
		if (max_a <= min_a || max_a - min_a > ZAP_REGION_SIZE) {
			IOLog("TopazGPU: zap layout unexpected\n");
			goto out;
		}
		if (mdt_size > PAGE_SIZE || !kbuf_alloc(&meta, mdt_size)) {
			IOLog("TopazGPU: zap metadata of %zu bytes does not fit one page\n", mdt_size);
			goto out;
		}
		IOLog("TopazGPU: zap: metadata at %#llx\n", meta.pa);
		memcpy(meta.va, mdt, mdt_size);
		sweet_dcache_clean((vm_offset_t)meta.va, (unsigned)mdt_size);
		__asm__ volatile ("dsb sy" ::: "memory");
		scm(SCM_FN(SCM_SVC_PIL, PIL_SHUTDOWN), 1, GPU_PAS_ID, 0, 0, &res);
		st = scm(SCM_FN(SCM_SVC_PIL, PIL_INIT_IMAGE), 2 | (SCM_ARG_RW << 6), GPU_PAS_ID, meta.pa, 0, &res);
		if (st != 0 || res != 0) {
			IOLog("TopazGPU: zap init_image: %llx/%llx\n", st, res);
			goto out;
		}
		st = scm(SCM_FN(SCM_SVC_PIL, PIL_MEM_SETUP), 3, GPU_PAS_ID, ZAP_REGION, max_a - min_a, &res);
		if (st != 0 || res != 0) {
			IOLog("TopazGPU: zap mem_setup: %llx/%llx\n", st, res);
			goto out;
		}
		IOLog("TopazGPU: zap: init_image and mem_setup ok, copying the segment\n");
		volatile uint8_t *dst = (volatile uint8_t *)ml_io_map(ZAP_REGION, ZAP_REGION_SIZE);
		if (dst == NULL) {
			goto out;
		}
		for (uint32_t i = 0; i < ZAP_REGION_SIZE; i += 4) {
			*(volatile uint32_t *)(dst + i) = 0;
		}
		uint32_t at = seg_paddr - min_a, n = (uint32_t)(seg_size < ZAP_REGION_SIZE - at ? seg_size : ZAP_REGION_SIZE - at);
		for (uint32_t i = 0; i < n; i++) {
			dst[at + i] = ((const uint8_t *)seg)[i];
		}
		__asm__ volatile ("dsb sy" ::: "memory");
		st = scm(SCM_FN(SCM_SVC_PIL, PIL_AUTH_RESET), 1, GPU_PAS_ID, 0, 0, &res);
		IOLog("TopazGPU: zap auth_and_reset: %llx/%llx\n", st, res);
		ok = st == 0 && res == 0;
	}
out:
	OSSafeReleaseNULL(meta.md);
	return ok;
}

/* ---------------------------------------------------------------- SMMU stage-1 tables (4 KB granule) */

#define PTE_VALID       (1ull << 0)
#define PTE_TABLE       (1ull << 1)
#define PTE_AP_UNPRIV   (1ull << 6)
#define PTE_AP_RO       (1ull << 7)
#define PTE_SH_OUTER    (2ull << 8)
#define PTE_AF          (1ull << 10)
#define PTE_PAGE        (PTE_VALID | PTE_TABLE | PTE_AP_UNPRIV | PTE_SH_OUTER | PTE_AF)    /* AttrIndx 0 */

struct table { uint64_t *va; uint64_t pa; };
static struct table l1, l2[4];
static struct table *l3[2048];          /* one per 2 MB of GPU VA */
static struct kbuf table_chunk;         /* tables are carved from 16 KB pages */
static uint32_t table_chunk_used = 4;
static bool tables_dirty;

static bool
table_alloc(struct table *t)
{
	if (table_chunk_used == PAGE_SIZE / 4096) {
		struct kbuf b = {};
		if (!kbuf_alloc(&b, PAGE_SIZE)) {
			return false;
		}
		table_chunk = b;                        /* (never freed: tables live as long as the GPU) */
		table_chunk_used = 0;
	}
	t->va = (uint64_t *)(table_chunk.va + 4096 * table_chunk_used);
	t->pa = table_chunk.pa + 4096 * table_chunk_used;
	table_chunk_used++;
	return true;
}

static uint64_t *
l3_for(uint64_t va, bool create)
{
	uint32_t i1 = (uint32_t)(va >> 30), i2 = (uint32_t)((va >> 21) & 511), i3 = (uint32_t)(va >> 21);

	if (i1 >= 4) {
		return NULL;
	}
	if (l2[i1].va == NULL) {
		if (!create || !table_alloc(&l2[i1])) {
			return NULL;
		}
		l1.va[i1] = l2[i1].pa | PTE_VALID | PTE_TABLE;
		sweet_dcache_clean((vm_offset_t)&l1.va[i1], 8);
	}
	if (l3[i3] == NULL) {
		struct table *t = (struct table *)IOMalloc(sizeof(*t));
		if (!create || t == NULL || !table_alloc(t)) {
			if (t != NULL) {
				IOFree(t, sizeof(*t));
			}
			return NULL;
		}
		l3[i3] = t;
		l2[i1].va[i2] = t->pa | PTE_VALID | PTE_TABLE;
		sweet_dcache_clean((vm_offset_t)&l2[i1].va[i2], 8);
	}
	return l3[i3]->va;
}

static void
tlb_flush(void)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	wr(smmu, SMMU_CB0 + 0x618, 0);                  /* CB_TLBIALL */
	wr(smmu, SMMU_CB0 + 0x7F0, 0);                  /* CB_TLBSYNC */
	poll(smmu, SMMU_CB0 + 0x7F4, 1, 0, 1000);
}

static bool
mmu_map(uint64_t va, uint64_t pa, bool ro)
{
	uint64_t *t = l3_for(va, true);
	if (t == NULL || (pa >> 36) != 0) {
		return false;
	}
	t[(va >> 12) & 511] = pa | PTE_PAGE | (ro ? PTE_AP_RO : 0);
	sweet_dcache_clean((vm_offset_t)&t[(va >> 12) & 511], 8);
	tables_dirty = true;
	return true;
}

static void
smmu_fault(const char *tag)
{
	IOLog("TopazGPU: [%s] SMMU gfsr %08x CB0 FSR %08x FAR %08x%08x FSYNR0 %08x\n", tag, rd(smmu, 0x48),
	    rd(smmu, SMMU_CB0 + 0x58), rd(smmu, SMMU_CB0 + 0x64), rd(smmu, SMMU_CB0 + 0x60), rd(smmu, SMMU_CB0 + 0x68));
}

static bool
smmu_setup(void)
{
	IOLog("TopazGPU: smmu: tables\n");
	if (l1.va == NULL && !table_alloc(&l1)) {
		return false;
	}
	wr(smmu, SMMU_CB0 + 0x000, 0);                  /* SCTLR off while programming */
	wr(smmu, SMMU_CB0 + 0x058, 0xFFFFFFFF);         /* FSR clear */
	wr(smmu, 0x1000 + 0x800, 1);                    /* CBA2R0: AArch64 descriptors */
	wr(smmu, 0x1000, 0x0001f000);                   /* CBAR0: S1 translate, S2 bypass */
	wr(smmu, SMMU_CB0 + 0x020, (uint32_t)l1.pa);
	wr(smmu, SMMU_CB0 + 0x024, (uint32_t)(l1.pa >> 32));
	wr(smmu, SMMU_CB0 + 0x030, 32 | (1u << 23));    /* TCR: T0SZ 32, 4 KB, NC walks, EPD1 */
	wr(smmu, SMMU_CB0 + 0x010, 1);                  /* TCR2: PASize 36 bits */
	wr(smmu, SMMU_CB0 + 0x038, 0x44);               /* MAIR0: attr0 = Normal non-cacheable */
	wr(smmu, SMMU_CB0 + 0x03C, 0);
	wr(smmu, SMMU_CB0 + 0x000, gpu_identity ? 0xE0 : 0x67);    /* SCTLR: M=0 pass-through, or CFIE CFRE AFE TRE M */
	wr(smmu, 0xC00, 0);                             /* S2CR0 -> CB0 */
	wr(smmu, 0x800, (1u << 31) | (1u << 16));       /* SMR0: SID 0 mask 1 */
	smmu_on = true;
	tlb_flush();
	IOLog("TopazGPU: SMMU TTBR0 %08x%08x SCTLR %08x TCR %08x TCR2 %08x IDR0 %08x IDR2 %08x sCR0 %08x%s\n",
	    rd(smmu, SMMU_CB0 + 0x24), rd(smmu, SMMU_CB0 + 0x20), rd(smmu, SMMU_CB0), rd(smmu, SMMU_CB0 + 0x30),
	    rd(smmu, SMMU_CB0 + 0x10), rd(smmu, 0x20), rd(smmu, 0x28), rd(smmu, 0x0), gpu_identity ? " (identity)" : "");
	return true;
}

static bool
kbuf_map(struct kbuf *b)
{
	if (gpu_identity) {
		b->iova = b->pa;                        /* pool memory is contiguous */
		return true;
	}
	b->iova = kernel_va_next;
	kernel_va_next += b->size + 4096;               /* guard page */
	for (uint64_t off = 0; off < b->size; off += 4096) {
		if (!mmu_map(b->iova + off, kbuf_pa(b, off), false)) {
			return false;
		}
	}
	return true;
}

/* ---------------------------------------------------------------- GPU VA allocator (from darwin-redwood) */

static struct { uint64_t start, size; } va_free[1024];
static unsigned va_free_count;

extern "C" bool redwood_gpu_identity(void);
bool
redwood_gpu_identity(void)
{
	return gpu_identity != 0;
}

uint64_t
redwood_gpu_address(uint64_t size)
{
	size = (size + 0xffff) & ~0xffffull;
	for (unsigned i = 0; i < va_free_count; i++) {
		if (va_free[i].size >= size) {
			uint64_t address = va_free[i].start;
			va_free[i].start += size;
			va_free[i].size -= size;
			if (va_free[i].size == 0) {
				memmove(&va_free[i], &va_free[i + 1], (--va_free_count - i) * sizeof(va_free[0]));
			}
			return address;
		}
	}
	return 0;
}

void
redwood_gpu_address_free(uint64_t address, uint64_t size)
{
	unsigned at;

	if (gpu_identity) {
		pool_give_back(address, size);  /* GPU address = physical: the buffer's pool memory */
		return;
	}
	size = (size + 0xffff) & ~0xffffull;
	for (at = 0; at < va_free_count && va_free[at].start < address; at++) {
	}
	if (at > 0 && va_free[at - 1].start + va_free[at - 1].size == address) {
		va_free[at - 1].size += size;
		if (at < va_free_count && address + size == va_free[at].start) {
			va_free[at - 1].size += va_free[at].size;
			memmove(&va_free[at], &va_free[at + 1], (--va_free_count - at) * sizeof(va_free[0]));
		}
	} else if (at < va_free_count && address + size == va_free[at].start) {
		va_free[at].start = address;
		va_free[at].size += size;
	} else if (va_free_count < sizeof(va_free) / sizeof(va_free[0])) {
		memmove(&va_free[at + 1], &va_free[at], (va_free_count++ - at) * sizeof(va_free[0]));
		va_free[at].start = address;
		va_free[at].size = size;
	}
}

/* ---------------------------------------------------------------- CP */

static uint32_t
par(uint32_t v)
{
	return (0x9669 >> (0xF & (v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12) ^ (v >> 16) ^ (v >> 20) ^ (v >> 24) ^ (v >> 28)))) & 1;
}

static uint32_t
pkt7(uint32_t op, uint32_t cnt)
{
	return 0x70000000u | cnt | (par(cnt) << 15) | ((op & 0x7F) << 16) | (par(op) << 23);
}

static void
emit(uint32_t v)
{
	((volatile uint32_t *)ring.va)[wptr % RB_DWORDS] = v;
	wptr++;
}

static void
kick(void)
{
	sweet_dcache_clean((vm_offset_t)ring.va, RB_BYTES);
	gpu_wr(0x807, wptr % RB_DWORDS);                /* CP_RB_WPTR */
}

static uint32_t
ring_free(void)
{
	uint32_t r = gpu_rd(0x806), w = wptr % RB_DWORDS;
	return (r + RB_DWORDS - w - 1) % RB_DWORDS;
}

static bool
ring_idle(uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 100; i++) {
		if (gpu_rd(0x806) == wptr % RB_DWORDS) {
			return true;
		}
		IODelay(10);
	}
	IOLog("TopazGPU: ring stuck: rptr %u wptr %u RBBM_STATUS %08x CP_HW_FAULT %08x\n", gpu_rd(0x806),
	    wptr % RB_DWORDS, gpu_rd(0x210), gpu_rd(0x821));
	smmu_fault("ring");
	return false;
}

static const uint32_t a612_hwcg[][2] = {        /* Linux a6xx_catalog.c a612_hwcg (a610 uses it) */
	{ 0x000B0, 0x22222222 }, { 0x000B4, 0x02222220 }, { 0x000B8, 0x00000081 }, { 0x000BC, 0x0000f3cf },
	{ 0x000C0, 0x22222222 }, { 0x000C4, 0x22222222 }, { 0x000C8, 0x22222222 }, { 0x000CC, 0x00022222 },
	{ 0x000D0, 0x11111111 }, { 0x000D4, 0x11111111 }, { 0x000D8, 0x11111111 }, { 0x000DC, 0x00011111 },
	{ 0x000E0, 0x77777777 }, { 0x000E4, 0x77777777 }, { 0x000E8, 0x77777777 }, { 0x000EC, 0x00077777 },
	{ 0x000F0, 0x22222222 }, { 0x000F4, 0x01202222 }, { 0x000F8, 0x00002220 }, { 0x00100, 0x00040f00 },
	{ 0x00104, 0x05522022 }, { 0x00105, 0x00005555 }, { 0x00106, 0x00000011 }, { 0x00107, 0x00445044 },
	{ 0x00108, 0x04222222 }, { 0x00111, 0x00002222 }, { 0x00114, 0x02222222 }, { 0x00117, 0x00000002 },
	{ 0x0011B, 0x00002222 }, { 0x00109, 0x00004000 }, { 0x00112, 0x00002222 }, { 0x00115, 0x00000200 },
	{ 0x0011C, 0x00000000 }, { 0x0010A, 0x00000000 }, { 0x00113, 0x00000000 }, { 0x00116, 0x04104004 },
	{ 0x0011D, 0x00000000 }, { 0x0010B, 0x22222222 }, { 0x00110, 0x00000004 }, { 0x0010F, 0x00000002 },
	{ 0x00533, 0x00000182 }, { 0x00044, 0x00000000 }, { 0x00042, 0x00000000 }, { 0x00118, 0x00000222 },
	{ 0x00119, 0x00000111 }, { 0x0011A, 0x00000555 },
};

static void
hw_init_regs(void)
{
	static const uint32_t addr_mode[] = { 0x842, 0xC01, 0x8601, 0x8e05, 0x9e01, 0xbe05, 0xa601, 0x9601, 0xE00,
		                              0xae01, 0xb601, 0xF810 };

	gpu_wr(0x3c45, 0);
	(void)gpu_rd(0x3c45);
	gpu_wr(0x16, 0);
	(void)gpu_rd(0x16);
	gpu_wr(0xF803, 0);
	gpu_wr64(0xF800, 0);
	gpu_wr(0xF802, 0);
	for (unsigned i = 0; i < sizeof(addr_mode) / sizeof(addr_mode[0]); i++) {
		gpu_wr(addr_mode[i], 1);
	}
	for (unsigned i = 0; i < sizeof(a612_hwcg) / sizeof(a612_hwcg[0]); i++) {
		gpu_wr(a612_hwcg[i][0], a612_hwcg[i][1]);
	}
	gpu_wr(0xAE, 0xaaa8aa82);                       /* RBBM_CLOCK_CNTL: a610 clock_cntl_on */
	for (unsigned i = 0; i < 4; i++) {
		gpu_wr(0x3c03 + i, 0x00071620);
	}
	gpu_wr(0x11, 3);
	gpu_wr64(0xE05, 0x1fffffffff000ull + 0xfc0);
	gpu_wr64(0xE09, 0x1fffffffff000ull);
	gpu_wr64(0xE07, 0x1fffffffff000ull);
	gpu_wr64(0xE0B, GMEM_BASE);
	gpu_wr64(0xE0D, GMEM_BASE + GMEM_SIZE - 1);
	gpu_wr(0xE18, 0x804);
	gpu_wr(0xE17, 4);
	gpu_wr(0x8c2, 0x00800060);
	gpu_wr(0x8c1, 0x40201b16);
	gpu_wr(0x8C3, 48);
	gpu_wr(0x9e00, 0x00080000);
	gpu_wr(0x98d, 1);
	gpu_wr(0x1f, (1u << 30) | 0x3ffff);
	gpu_wr(0xe19, 0x81);
	gpu_wr(0x500, 1);                               /* RBBM_PERFCTR_CNTL */
	gpu_wr(0x90E, 47);                              /* CP_MEM_POOL_DBG_ADDR */
	gpu_wr(0x50B, 0xffffffff);                      /* RBBM_PERFCTR_GPU_BUSY_MASKED */
	gpu_wr(0x8D0, 0);                               /* CP_PERFCTR_CP_SEL0 */
	gpu_wr(0x8e08, 0x9);                            /* RB_NC_MODE_CNTL */
	gpu_wr(0xb604, 0x9);                            /* TPL1_NC_MODE_CNTL */
	gpu_wr(0xae02, 0x9);                            /* SP_NC_MODE_CNTL */
	gpu_wr(0xE01, 1u << 23);                        /* UCHE_MODE_CNTL */
	gpu_wr(0x534, 0);                               /* RBBM_NC_MODE_CNTL */
}

static bool
cp_start(void)
{
	void *fw = NULL;
	size_t fw_size = 0;

	fw = (void *)topaz_fw_sqe_fw;
	fw_size = topaz_fw_sqe_fw_size;
	IOLog("TopazGPU: cp: SQE %zu bytes, allocating ring\n", fw_size);
	if ((sqe.md == NULL && (!kbuf_alloc(&sqe, fw_size - 4) || !kbuf_map(&sqe))) ||
	    (ring.md == NULL && (!kbuf_alloc(&ring, RB_BYTES) || !kbuf_map(&ring))) ||
	    (mem.md == NULL && (!kbuf_alloc(&mem, PAGE_SIZE) || !kbuf_map(&mem)))) {
		IOLog("TopazGPU: CP buffers failed\n");
		return false;
	}
	memcpy(sqe.va, (const uint8_t *)fw + 4, fw_size - 4);
	sweet_dcache_clean((vm_offset_t)sqe.va, (unsigned)(fw_size - 4));
	IOLog("TopazGPU: cp: SQE at GPU %#llx, ring %#llx\n", sqe.iova, ring.iova);
	tlb_flush();
	hw_init_regs();
	gpu_wr64(0x830, sqe.iova);                      /* CP_SQE_INSTR_BASE */
	gpu_wr64(0x800, ring.iova);                     /* CP_RB_BASE */
	gpu_wr(0x802, 12 | (2 << 8) | (1u << 27));      /* CP_RB_CNTL */
	wptr = 0;
	gpu_wr(0x807, 0);
	__asm__ volatile ("dsb sy" ::: "memory");
	gpu_wr(0x808, 1);                               /* CP_SQE_CNTL */

	emit(pkt7(CP_ME_INIT, 8));
	emit(0x2f);
	emit(3);
	emit(0x20000000);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	emit(0);
	kick();
	if (!ring_idle(100)) {
		return false;
	}
	emit(pkt7(CP_SET_SECURE_MODE, 1));
	emit(0);
	kick();
	if (!ring_idle(100)) {
		return false;
	}
	IOLog("TopazGPU: CP up (ME_INIT, SET_SECURE_MODE 0), core %u MHz\n", core_mhz);
	return true;
}

static bool
hw_start(void)
{
	gcc = (volatile uint8_t *)ml_io_map(GCC_BASE, GCC_SIZE);
	gpucc = (volatile uint8_t *)ml_io_map(GPUCC_BASE, GPUCC_SIZE);
	gpu = (volatile uint8_t *)ml_io_map(GPU_BASE, GPU_SIZE);
	smmu = (volatile uint8_t *)ml_io_map(SMMU_BASE, SMMU_SIZE);
	if (gcc == NULL || gpucc == NULL || gpu == NULL || smmu == NULL) {
		IOLog("TopazGPU: map failed\n");
		return false;
	}
	va_free_count = 0;
	redwood_gpu_address_free(VA_USER_START, VA_KERNEL - VA_USER_START);
	if (!power_up()) {
		return false;
	}
	IOLog("TopazGPU: powered: RBBM_STATUS %08x\n", gpu_rd(0x210));
	return zap_load() && smmu_setup() && cp_start();
}

/* ---------------------------------------------------------------- the interface RedwoodGPU.cpp uses */

bool
redwood_gpu_ready(void)
{
	return ready;
}

bool
redwood_gpu_parameter(uint32_t which, uint64_t *value)
{
	switch (which) {
	case 0x01: *value = 610; return true;                   /* GPU_ID */
	case 0x02: *value = GMEM_SIZE; return true;             /* GMEM_SIZE */
	case 0x03: *value = 0x06010001; return true;            /* CHIP_ID */
	case 0x04: *value = core_mhz * 1000000ull; return true; /* MAX_FREQ */
	case 0x05:                                              /* TIMESTAMP: CP always-on counter */
		*value = ready ? ((uint64_t)gpu_rd(0x981) << 32 | gpu_rd(0x980)) : 0;
		return true;
	case 0x06: *value = GMEM_BASE; return true;             /* GMEM_BASE */
	case 0x07: *value = 1; return true;                     /* PRIORITIES */
	case 0x08: case 0x09: case 0x0a: case 0x11:             /* PP_PGTABLE FAULTS SUSPENDS RAYTRACING */
		*value = 0;
		return true;
	case 0x10: *value = 13; return true;                    /* HIGHEST_BANK_BIT (a6xx_calc_ubwc_config a610) */
	case 0x12: *value = 7; return true;                     /* UBWC_SWIZZLE */
	case 0x13: *value = 0; return true;                     /* MACROTILE_MODE */
	default:
		return false;                                   /* VA_START/VA_SIZE: addresses are ours to give */
	}
}

bool
redwood_gpu_map(uint64_t address, uint64_t phys, bool read_only)
{
	if (gpu_identity) {
		return true;
	}
	return mmu_map(address, phys, read_only);
}

void
redwood_gpu_mapped(void)
{
	if (tables_dirty) {
		tables_dirty = false;
		tlb_flush();
	}
}

void
redwood_gpu_unmap(uint64_t address, uint64_t size)
{
	if (gpu_identity) {
		return;
	}
	for (uint64_t off = 0; off < size; off += 4096) {
		uint64_t *t = l3_for(address + off, false);
		if (t != NULL) {
			t[((address + off) >> 12) & 511] = 0;
			sweet_dcache_clean((vm_offset_t)&t[((address + off) >> 12) & 511], 8);
		}
	}
	tlb_flush();
}

void
redwood_gpu_to_memory(void *start, uint64_t size)
{
	if (start != NULL && size != 0) {
		sweet_dcache_flush((vm_offset_t)start, (unsigned)size);    /* clean + invalidate */
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

bool
redwood_gpu_done(uint32_t fence)
{
	if (mem.va == NULL) {
		return false;
	}
	sweet_dcache_flush((vm_offset_t)mem.va, 64);
	return (int32_t)(*(volatile uint32_t *)mem.va - fence) >= 0;
}

uint32_t
redwood_gpu_last(void)
{
	return seqno;
}

void
redwood_gpu_worked(uint64_t *microseconds, uint32_t *submissions, uint32_t by[7])
{
	*microseconds = 0;
	*submissions = 0;
	bzero(by, 7 * sizeof(uint32_t));
}

/* IBs then CACHE_FLUSH_TS writing the submission's number to mem+0 (TopazGpuW HwSubmit). 0: ring full. */
uint32_t
redwood_gpu_submit(const uint64_t *addresses, const uint32_t *words, uint32_t count)
{
	uint32_t need = 4 * count + 5;

	if (!ready) {
		return 0;
	}
	IOLockLock(hw_lock);
	for (int spin = 0; ring_free() < need && spin < 20000; spin++) {
		IODelay(100);
	}
	if (ring_free() < need) {
		IOLockUnlock(hw_lock);
		return 0;
	}
	for (uint32_t i = 0; i < count; i++) {
		emit(pkt7(CP_INDIRECT_BUFFER, 3));
		emit((uint32_t)addresses[i]);
		emit((uint32_t)(addresses[i] >> 32));
		emit(words[i]);
	}
	if (++seqno == 0) {
		seqno = 1;
	}
	uint32_t fence = seqno;
	emit(pkt7(CP_EVENT_WRITE, 4));
	emit(CACHE_FLUSH_TS);
	emit((uint32_t)mem.iova);
	emit((uint32_t)(mem.iova >> 32));
	emit(fence);
	kick();
	IOLockUnlock(hw_lock);
	return fence;
}

void
redwood_gpu_report(const char *why)
{
	uint64_t ib1 = gpu_rd(0x928) | ((uint64_t)gpu_rd(0x929) << 32);
	IOLog("TopazGPU: %s: fence %u done %u rptr %u wptr %u RBBM_STATUS %08x CP_HW_FAULT %08x IB1 %llx rem %u\n", why,
	    seqno, mem.va ? *(volatile uint32_t *)mem.va : 0, gpu_rd(0x806), wptr % RB_DWORDS, gpu_rd(0x210),
	    gpu_rd(0x821), ib1, gpu_rd(0x92A));
	smmu_fault(why);
}

void
redwood_gpu_stop(void)
{
	if (ready) {
		gpu_wr(0x808, 0);
	}
	ready = false;
}

void
redwood_tl(const char *format, ...)
{
	(void)format;
}

/* ---------------------------------------------------------------- start */

static void
topaz_adreno_thread(__unused void *param, __unused wait_result_t wr_)
{
	for (int i = 0; i < 600 && !topaz_root_mounted(); i++) {
		IOSleep(100);
	}
	IOSleep(500);
	if (!topaz_root_mounted()) {
		IOLog("TopazGPU: no root filesystem: not started\n");
		return;
	}
	IOLockLock(hw_lock);
	bool ok = hw_start();
	IOLockUnlock(hw_lock);
	if (!ok) {
		failed = true;
		IOLog("TopazGPU: THE GPU IS NOT STARTED\n");
		return;
	}
	ready = true;
	RedwoodGPUStart();
	IOLog("TopazGPU: Adreno 610 ready, RedwoodGPU published\n");
}

void
topaz_adreno_init(void)
{
	uint32_t on = 0, mhz = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("gpu", &on, sizeof(on)) || on == 0 || hw_lock != NULL) {
		return;
	}
	(void)PE_parse_boot_argn("gpu_identity", &gpu_identity, sizeof(gpu_identity));
	(void)PE_parse_boot_argn("gpu_midmem", &gpu_midmem, sizeof(gpu_midmem));
	if (PE_parse_boot_argn("gpu_mhz", &mhz, sizeof(mhz)) && mhz >= 300) {
		core_mhz = mhz;                                 /* (checked against the OPP table in power_up) */
	}
	hw_lock = IOLockAlloc();
	if (kernel_thread_start(topaz_adreno_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
