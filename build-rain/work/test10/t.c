#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
typedef uintptr_t vm_offset_t; typedef size_t vm_size_t; typedef uint64_t addr64_t; typedef void *DTEntry;
#define kSuccess 0
#define TRUE 1
#define RAIN_OP_TX 1u
#define RM_TXMAX 0x1100u
#define RAIN_NVT_RST 0x3f0feu
#define RAIN_NVT_RDFAST 0x3f310u
int SecureDTLookupEntry(DTEntry, const char *, DTEntry *);
int SecureDTGetProperty(DTEntry, const char *, void const **, unsigned int *);
vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
void flush_dcache64(addr64_t, unsigned, int);
static void touch_wait(unsigned ms) { (void)ms; }
static struct { int err; const char *stage; } rn;
/* fake chip: record writes */
static unsigned nwr, bytes;
static int rain_spi(uint32_t op, const uint8_t *tx, uint8_t *rx, unsigned n) { (void)op;(void)tx;(void)rx; nwr++; bytes += n; return 0; }
static int rain_nvt_write(uint32_t a, uint8_t v) { (void)a;(void)v; nwr+=2; return 0; }
static int rain_nvt_read(uint32_t a, uint8_t *o, unsigned n) { memset(o, 0, n); if (a == (0x21c00u|0x60u)) o[0] = 0xa0; return 0; }
#include "blk.inc"
int main(int c, char **v) {
	FILE *f = fopen(v[1], "rb"); static uint8_t fw[0x40000]; size_t len = fread(fw, 1, sizeof fw, f);
	int e = rain_nvt_download(fw, (uint32_t)len);
	printf("err=%d parts=%u tries=%u wrote=%#x spi=%u bytes=%#x\n", e, nf.nparts, nf.tries, nf.wrote, nwr, bytes);
	return e != 0;
}
