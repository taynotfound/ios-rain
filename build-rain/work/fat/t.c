#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static FILE *img;
static int fat_rd(uint64_t off, void *dst, uint64_t len) { return fseek(img, (long)off, SEEK_SET) || fread(dst, 1, len, img) != len; }
static void *fat_alloc(uint32_t n) { return malloc(n); }
static void fat_free(void *p, uint32_t n) { (void)n; free(p); }
#include "fat.inc"
int main(int c, char **v) {
	img = fopen(v[1], "rb");
	for (int i = 2; i + 1 < c; i += 2) {
		uint32_t cl, sz; FILE *ref = fopen(v[i + 1], "rb");
		fseek(ref, 0, SEEK_END); long rl = ftell(ref); rewind(ref);
		char *a = (char *)malloc(rl), *b = (char *)malloc(rl); fread(a, 1, rl, ref);
		if (!fat_lookup(v[i], &cl, &sz) || sz != (uint32_t)rl || !fat_read_file(cl, sz, 0, b, sz) || memcmp(a, b, rl)) { printf("FAIL %s\n", v[i]); return 1; }
		/* a read in the middle, across clusters */
		if (rl > 5000 && (!fat_read_file(cl, sz, 3000, b, 2000) || memcmp(a + 3000, b, 2000))) { printf("FAIL mid %s\n", v[i]); return 1; }
		printf("ok %s (%u bytes, fat%s)\n", v[i], sz, fs.fat32 ? "32" : "16");
	}
	uint32_t cl, sz;
	if (fat_lookup("image/nothere.bin", &cl, &sz) || fat_lookup("image", &cl, &sz)) { puts("FAIL negative"); return 1; }
	puts("negative lookups ok");
	return 0;
}
