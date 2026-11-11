#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "fit.inc"
/* source pixel value encodes its row: then each output row shows which source row it sampled */
int main(void) {
	const uint32_t LW = 1125, LH = 2436, PW = 720, PH = 1650, DS = PW * 4;
	uint32_t *src = malloc((size_t)LW * LH * 4);
	for (uint32_t y = 0; y < LH; y++) for (uint32_t x = 0; x < LW; x++) src[y * LW + x] = 0xff000000u | (y << 8) | 0x01;
	uint32_t *dst = calloc((size_t)PW * PH, 4);
	int32_t xs[1088]; uint16_t xf[1088];
	/* stretch: every panel row written, first row from the top, last row from the bottom */
	sweet_fit_blit((uint8_t *)dst, DS, PW, PH, 0, LW, LH, (const uint8_t *)src, LW * 4, 0, 0, 0, 0, LW, LH, xs, xf);
	for (uint32_t y = 0; y < PH; y++) assert(dst[y * PW + 360] != 0);
	uint32_t r0 = (dst[360] >> 8) & 0xffff, rl = (dst[(PH - 1) * PW + 360] >> 8) & 0xffff;
	printf("stretch: panel row 0 <- source row ~%u, row %u <- ~%u\n", r0, PH - 1, rl);
	assert(r0 <= 2 && rl >= LH - 3);
	/* letterbox (fit=1): 1559 rows from y 45, bars untouched */
	memset(dst, 0, (size_t)PW * PH * 4);
	uint32_t fh = (uint32_t)((uint64_t)LH * PW / LW);
	sweet_fit_blit((uint8_t *)dst, DS, PW, fh, 45, LW, LH, (const uint8_t *)src, LW * 4, 0, 0, 0, 0, LW, LH, xs, xf);
	assert(dst[44 * PW + 360] == 0 && dst[45 * PW + 360] != 0 && dst[(45 + fh - 1) * PW + 360] != 0 && dst[(45 + fh) * PW + 360] == 0);
	/* a sub-rectangle with a source offset stays inside the source */
	sweet_fit_blit((uint8_t *)dst, DS, PW, PH, 0, LW, LH, (const uint8_t *)src, LW * 4, 3, 7, 100, 2000, 400, 436, xs, xf);
	puts("letterbox and sub-rectangle ok");
	return 0;
}
