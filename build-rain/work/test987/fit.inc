/* RAIN-FIT-BEGIN */
/*
 * rain: scale a rectangle of the logical canvas (lx,ly,lw,lh; its pixels start
 * at src + sy*stride + sx*4) into the panel at the canvas-to-panel width ratio,
 * bilinear, BGRA. Panel rows are offset by top. xs/xf: scratch of panelW entries.
 */
static inline uint32_t
sweet_fit_lerp_old(uint32_t a, uint32_t b, uint32_t f)
{
	uint32_t rb = ((a & 0x00ff00ffu) * (256u - f) + (b & 0x00ff00ffu) * f) >> 8;
	uint32_t ag = (((a >> 8) & 0x00ff00ffu) * (256u - f) + ((b >> 8) & 0x00ff00ffu) * f) >> 8;
	return (rb & 0x00ff00ffu) | ((ag & 0x00ff00ffu) << 8);
}

static void
sweet_fit_blit_old(uint8_t *dst, uint32_t dstStride, uint32_t panelW, uint32_t fitH, uint32_t top,
    uint32_t logicalW, const uint8_t *src, uint32_t stride, int32_t sx, int32_t sy,
    int32_t lx, int32_t ly, int32_t lw, int32_t lh, int32_t *xs, uint16_t *xf)
{
	if (lw <= 0 || lh <= 0 || panelW == 0 || logicalW == 0) {
		return;
	}
	/* panel pixels whose centres fall inside the logical rectangle */
	int64_t px0 = ((int64_t)lx * panelW + logicalW - 1) / logicalW;
	int64_t px1 = ((int64_t)(lx + lw) * panelW + logicalW - 1) / logicalW;
	int64_t py0 = ((int64_t)ly * panelW + logicalW - 1) / logicalW;
	int64_t py1 = ((int64_t)(ly + lh) * panelW + logicalW - 1) / logicalW;
	if (px0 < 0) {
		px0 = 0;
	}
	if (py0 < 0) {
		py0 = 0;
	}
	if (px1 > (int64_t)panelW) {
		px1 = panelW;
	}
	if (py1 > (int64_t)fitH) {
		py1 = fitH;
	}
	/* logical coordinate of a panel pixel centre, in 1/256 px: (2p+1)*L*128/P - 128 */
	for (int64_t px = px0; px < px1; px++) {
		int64_t u = ((2 * px + 1) * (int64_t)logicalW * 128) / panelW - 128 - (int64_t)lx * 256;
		if (u < 0) {
			u = 0;
		}
		if (u > (int64_t)(lw - 1) * 256) {
			u = (int64_t)(lw - 1) * 256;
		}
		xs[px] = (int32_t)(u >> 8);
		xf[px] = (uint16_t)(xs[px] + 1 < lw ? (u & 255) : 0);
	}
	for (int64_t py = py0; py < py1; py++) {
		int64_t v = ((2 * py + 1) * (int64_t)logicalW * 128) / panelW - 128 - (int64_t)ly * 256;
		if (v < 0) {
			v = 0;
		}
		if (v > (int64_t)(lh - 1) * 256) {
			v = (int64_t)(lh - 1) * 256;
		}
		int32_t y0 = (int32_t)(v >> 8);
		uint32_t fy = y0 + 1 < lh ? (uint32_t)(v & 255) : 0;
		const uint32_t *r0 = (const uint32_t *)(src + (uint64_t)(sy + y0) * stride) + sx;
		const uint32_t *r1 = fy ? (const uint32_t *)((const uint8_t *)r0 + stride) : r0;
		uint32_t *out = (uint32_t *)(dst + (uint64_t)(top + py) * dstStride);
		for (int64_t px = px0; px < px1; px++) {
			int32_t x = xs[px];
			uint32_t fx = xf[px];
			int32_t x1 = fx ? x + 1 : x;
			uint32_t t = sweet_fit_lerp_old(r0[x], r0[x1], fx);
			uint32_t b = fy ? sweet_fit_lerp_old(r1[x], r1[x1], fx) : t;
			out[px] = fy ? sweet_fit_lerp_old(t, b, fy) : t;
		}
	}
}

/* RAIN-FIT-BEGIN */
/*
 * rain: scale a rectangle of the logical canvas (lx,ly,lw,lh; its pixels start
 * at src + sy*stride + sx*4) into the panel at the canvas-to-panel width ratio,
 * bilinear, BGRA. Panel rows are offset by top. xs/xf: scratch of panelW entries.
 */
static inline uint32_t
sweet_fit_lerp_new(uint32_t a, uint32_t b, uint32_t f)
{
	uint32_t rb = ((a & 0x00ff00ffu) * (256u - f) + (b & 0x00ff00ffu) * f) >> 8;
	uint32_t ag = (((a >> 8) & 0x00ff00ffu) * (256u - f) + ((b >> 8) & 0x00ff00ffu) * f) >> 8;
	return (rb & 0x00ff00ffu) | ((ag & 0x00ff00ffu) << 8);
}

#define SWEET_FIT_ROW_MAX_new 2048         /* source row pixels the staging rows hold */

static void
sweet_fit_blit_new(uint8_t *dst, uint32_t dstStride, uint32_t panelW, uint32_t fitH, uint32_t top,
    uint32_t logicalW, const uint8_t *src, uint32_t stride, int32_t sx, int32_t sy,
    int32_t lx, int32_t ly, int32_t lw, int32_t lh, int32_t *xs, uint16_t *xf)
{
	if (lw <= 0 || lh <= 0 || panelW == 0 || logicalW == 0) {
		return;
	}
	/* panel pixels whose centres fall inside the logical rectangle */
	int64_t px0 = ((int64_t)lx * panelW + logicalW - 1) / logicalW;
	int64_t px1 = ((int64_t)(lx + lw) * panelW + logicalW - 1) / logicalW;
	int64_t py0 = ((int64_t)ly * panelW + logicalW - 1) / logicalW;
	int64_t py1 = ((int64_t)(ly + lh) * panelW + logicalW - 1) / logicalW;
	if (px0 < 0) {
		px0 = 0;
	}
	if (py0 < 0) {
		py0 = 0;
	}
	if (px1 > (int64_t)panelW) {
		px1 = panelW;
	}
	if (py1 > (int64_t)fitH) {
		py1 = fitH;
	}
	/* logical coordinate of a panel pixel centre, in 1/256 px: (2p+1)*L*128/P - 128 */
	for (int64_t px = px0; px < px1; px++) {
		int64_t u = ((2 * px + 1) * (int64_t)logicalW * 128) / panelW - 128 - (int64_t)lx * 256;
		if (u < 0) {
			u = 0;
		}
		if (u > (int64_t)(lw - 1) * 256) {
			u = (int64_t)(lw - 1) * 256;
		}
		xs[px] = (int32_t)(u >> 8);
		xf[px] = (uint16_t)(xs[px] + 1 < lw ? (u & 255) : 0);
	}
	/*
	 * The source is usually a GPU surface in write-combined (uncached) memory, where every
	 * 4-byte read is its own bus transaction: about 4.5 M of them a frame made the GPU
	 * slower than the CPU. Each source row is copied once, with memcpy's wide loads, into
	 * a cached row (two in flight: y0 and y0 + 1 never share a slot), and scaled from there.
	 */
	static uint32_t rows[2][SWEET_FIT_ROW_MAX_new];
	int32_t row_tag[2] = { -1, -1 };
	bool stage = lw <= SWEET_FIT_ROW_MAX_new;
	for (int64_t py = py0; py < py1; py++) {
		int64_t v = ((2 * py + 1) * (int64_t)logicalW * 128) / panelW - 128 - (int64_t)ly * 256;
		if (v < 0) {
			v = 0;
		}
		if (v > (int64_t)(lh - 1) * 256) {
			v = (int64_t)(lh - 1) * 256;
		}
		int32_t y0 = (int32_t)(v >> 8);
		uint32_t fy = y0 + 1 < lh ? (uint32_t)(v & 255) : 0;
		const uint32_t *r0 = (const uint32_t *)(src + (uint64_t)(sy + y0) * stride) + sx;
		const uint32_t *r1 = fy ? (const uint32_t *)((const uint8_t *)r0 + stride) : r0;
		if (stage) {
			for (int32_t k = 0; k < (fy ? 2 : 1); k++) {
				int32_t y = y0 + k, slot = y & 1;
				if (row_tag[slot] != y) {
					memcpy(rows[slot], (const uint8_t *)r0 + (uint64_t)k * stride, (size_t)lw * 4);
					row_tag[slot] = y;
				}
			}
			r0 = rows[y0 & 1];
			r1 = fy ? rows[(y0 + 1) & 1] : r0;
		}
		uint32_t *out = (uint32_t *)(dst + (uint64_t)(top + py) * dstStride);
		for (int64_t px = px0; px < px1; px++) {
			int32_t x = xs[px];
			uint32_t fx = xf[px];
			int32_t x1 = fx ? x + 1 : x;
			uint32_t t = sweet_fit_lerp_new(r0[x], r0[x1], fx);
			uint32_t b = fy ? sweet_fit_lerp_new(r1[x], r1[x1], fx) : t;
			out[px] = fy ? sweet_fit_lerp_new(t, b, fy) : t;
		}
	}
}
