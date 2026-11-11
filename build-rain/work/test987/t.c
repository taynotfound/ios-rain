#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "fit.inc"
int main(void) {
	const uint32_t LW = 1125, LH = 2436, PW = 720, FH = 1559, TOP = 45, DS = 720 * 4;
	uint32_t *src = malloc((size_t)LW * LH * 4);
	uint8_t *a = calloc((size_t)DS, 1650), *b = calloc((size_t)DS, 1650);
	int32_t xs[1088]; uint16_t xf[1088];
	srand(7);
	for (size_t i = 0; i < (size_t)LW * LH; i++) src[i] = (uint32_t)rand() * 2654435761u;
	/* full frame, then a sub-rectangle with a source offset */
	int r[2][6] = { {0, 0, 0, 0, 1125, 2436}, {17, 33, 100, 200, 400, 900} };
	for (int t = 0; t < 2; t++) {
		sweet_fit_blit_old(a, DS, PW, FH, TOP, LW, (const uint8_t *)src, LW * 4, r[t][0], r[t][1], r[t][2], r[t][3], r[t][4], r[t][5], xs, xf);
		sweet_fit_blit_new(b, DS, PW, FH, TOP, LW, (const uint8_t *)src, LW * 4, r[t][0], r[t][1], r[t][2], r[t][3], r[t][4], r[t][5], xs, xf);
		if (memcmp(a, b, (size_t)DS * 1650)) { printf("MISMATCH case %d\n", t); return 1; }
	}
	puts("staged rows give identical output (full frame and sub-rectangle)");
	return 0;
}
