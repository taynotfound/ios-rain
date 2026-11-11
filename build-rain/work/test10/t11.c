#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
typedef uintptr_t vm_offset_t; typedef size_t vm_size_t; typedef uint64_t addr64_t; typedef void *DTEntry;
#define kSuccess 0
#define TRUE 1
#define RAIN_OP_TX 1u
#define RM_TXMAX 0x1100u
#define RAIN_NVT_RST 0x3f0feu
#define RAIN_NVT_RDFAST 0x3f310u
#define TOUCH_MAX 10u
int SecureDTLookupEntry(DTEntry a, const char *b, DTEntry *c) { return 1; }
int SecureDTGetProperty(DTEntry a, const char *b, void const **c, unsigned int *d) { return 1; }
vm_offset_t ml_io_map(vm_offset_t a, vm_size_t b) { return 0; }
void flush_dcache64(addr64_t a, unsigned b, int c) {}
static struct { int err; const char *stage; } rn;
static struct { int x[TOUCH_MAX], y[TOUCH_MAX]; } st;
static int rain_spi(uint32_t op, const uint8_t *tx, uint8_t *rx, unsigned n) { return 0; }
static int rain_nvt_write(uint32_t a, uint8_t v) { return 0; }
/* frames: finger id 1 at panel (0,45), then id 2 at (719,1603), then nothing, then end */
static int frame;
static void put(uint8_t *f, unsigned id, unsigned x, unsigned y) { f[0] = id << 3 | 1; f[1] = x >> 4; f[2] = y >> 4; f[3] = (x & 15) << 4 | (y & 15); }
static int rain_nvt_read(uint32_t a, uint8_t *o, unsigned n) {
	memset(o, 0, n);
	if (frame == 0) put(o, 1, 0, 45);
	if (frame == 1) put(o + 6, 2, 719, 1603);
	if (frame == 2) put(o, 3, 800, 100);          /* out of range: ignored */
	return 0;
}
static void touch_report(uint32_t mask, const int *x, const int *y) {
	if (frame == 0) assert(mask == 1 && x[0] == 0 && y[0] == 0);
	if (frame == 1) assert(mask == 2 && x[1] == 1078 && y[1] == 2398);
	if (frame == 2) { assert(mask == 0); puts("run loop mapping OK"); exit(0); }
	frame++;
}
static void touch_wait(unsigned ms) {}
#include "blk11.inc"
int main(void) { rain_touch_run(); return 1; }
