/* ---- C10: NT36525B firmware download (Xiaomi nt36xxx_fw_update.c, hw_crc = 1, NT36672A map) ---- */
#define NVT_EVENT_BUF    0x21c00u
#define NVT_BOOT_RDY     0x3f10du
#define NVT_ILM_LEN      0x3f118u
#define NVT_DLM_LEN      0x3f130u
#define NVT_ILM_DES      0x3f128u
#define NVT_DLM_DES      0x3f12cu
#define NVT_G_ILM_CRC    0x3f100u
#define NVT_G_DLM_CRC    0x3f104u
#define NVT_R_ILM_CRC    0x3f120u
#define NVT_R_DLM_CRC    0x3f124u
#define NVT_BLD_CRC_EN   0x3f30eu
#define NVT_CRC_FLAG     0x3f133u
#define NVT_CHUNK        4096u
#define NVT_MAXPART      24u

static struct {
	uint32_t fwpa, fwlen, nparts, tries, wrote;
	int err;
	uint8_t rst[5], crcflag;
	uint32_t rilm, rdlm;
	uint8_t info[12];
	uint32_t polls, downs, fx, fy, lx, ly;
	uint8_t raw[7];
} nf;

static int
rain_nvt_page(uint32_t addr)
{
	uint8_t page[3] = { 0xff, (uint8_t)(addr >> 15), (uint8_t)(addr >> 7) };
	return rain_spi(RAIN_OP_TX, page, NULL, 3);
}

/* page select, then addr[6:0]|0x80 followed by data (len <= RM_TXMAX - 1) */
static int
rain_nvt_wbytes(uint32_t addr, const uint8_t *data, unsigned len, bool device_src)
{
	static uint8_t buf[RM_TXMAX];
	int e = rain_nvt_page(addr);
	if (e) {
		return e;
	}
	buf[0] = (uint8_t)((addr & 0x7fu) | 0x80u);
	for (unsigned i = 0; i < len; i++) {      /* byte copy: the firmware sits in device-mapped memory */
		buf[1 + i] = device_src ? ((const volatile uint8_t *)data)[i] : data[i];
	}
	return rain_spi(RAIN_OP_TX, buf, NULL, len + 1u);
}

static uint32_t
le32(const volatile uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int
rain_nvt_download(const volatile uint8_t *fw, uint32_t fwlen)
{
	uint32_t sram[NVT_MAXPART], size[NVT_MAXPART], bin[NVT_MAXPART], crc[2];
	uint32_t end = le32(fw), np = 0;
	if (fwlen < 0x100u || end > fwlen || (fw[0x20] & 2u)) {
		return -50;                               /* cascade header: not this chip */
	}
	for (uint32_t i = 0; i < 2; i++) {            /* ILM, DLM */
		bin[np] = le32(fw + 12u * i);
		sram[np] = le32(fw + 4u + 12u * i);
		size[np] = le32(fw + 8u + 12u * i);
		crc[i] = le32(fw + 0x18u + 4u * i);
		np++;
	}
	for (uint32_t pos = 0x30; pos < end && np < NVT_MAXPART; pos += 0x10) {   /* info sections */
		sram[np] = le32(fw + pos);
		size[np] = le32(fw + pos + 4u);
		bin[np] = le32(fw + pos + 8u);
		np++;
	}
	uint32_t novly = (fw[0x28] & 0x10u) ? (fw[0x28] & 0x0fu) : 0;
	for (uint32_t k = 0; k < novly && np < NVT_MAXPART; k++) {             /* overlays, listed at DLM start */
		uint32_t pos = bin[1] + 0x10u * k;
		if (pos + 12u > fwlen) {
			return -51;
		}
		sram[np] = le32(fw + pos);
		size[np] = le32(fw + pos + 4u);
		bin[np] = le32(fw + pos + 8u);
		np++;
	}
	nf.nparts = np;
	for (uint32_t i = 0; i < np; i++) {
		if (size[i] && bin[i] + size[i] + 1u > fwlen) {
			return -52;
		}
	}

	int e = 0;
	for (nf.tries = 1; nf.tries <= 3; nf.tries++) {
		nf.wrote = 0;
		/* bootloader reset */
		e = rain_nvt_write(RAIN_NVT_RST, 0x69);
		touch_wait(5);
		e = e ? e : rain_nvt_write(RAIN_NVT_RDFAST, 0x00);
		/* hw crc banks: destination (3 bytes), length (2 bytes), golden crc (4 bytes) */
		for (uint32_t i = 0; i < 2 && !e; i++) {
			uint8_t des[3] = { (uint8_t)sram[i], (uint8_t)(sram[i] >> 8), (uint8_t)(sram[i] >> 16) };
			uint8_t len[2] = { (uint8_t)size[i], (uint8_t)(size[i] >> 8) };
			uint8_t c[4] = { (uint8_t)crc[i], (uint8_t)(crc[i] >> 8), (uint8_t)(crc[i] >> 16), (uint8_t)(crc[i] >> 24) };
			e = rain_nvt_wbytes(i ? NVT_DLM_DES : NVT_ILM_DES, des, 3, false);
			e = e ? e : rain_nvt_wbytes(i ? NVT_DLM_LEN : NVT_ILM_LEN, len, 2, false);
			e = e ? e : rain_nvt_wbytes(i ? NVT_G_DLM_CRC : NVT_G_ILM_CRC, c, 4, false);
		}
		/* every non-empty partition: size + 1 bytes from bin to sram */
		for (uint32_t i = 0; i < np && !e; i++) {
			uint32_t left = size[i] ? size[i] + 1u : 0, a = sram[i], b = bin[i];
			while (left && !e) {
				uint32_t n = left < NVT_CHUNK ? left : NVT_CHUNK;
				e = rain_nvt_wbytes(a, (const uint8_t *)(fw + b), n, true);
				a += n;
				b += n;
				left -= n;
				nf.wrote += n;
			}
		}
		if (e) {
			return e;
		}
		/* enable bld crc, clear reset status, enable fw crc, boot ready */
		uint8_t v = 0;
		e = rain_nvt_read(NVT_BLD_CRC_EN, &v, 1);
		e = e ? e : rain_nvt_write(NVT_BLD_CRC_EN, (uint8_t)(v | 0x80u));
		e = e ? e : rain_nvt_write(NVT_EVENT_BUF | 0x60u, 0x00);
		e = e ? e : rain_nvt_write(NVT_EVENT_BUF | 0x50u, 0xae);
		e = e ? e : rain_nvt_write(NVT_BOOT_RDY, 1);
		if (e) {
			return e;
		}
		touch_wait(5);
		/* reset state 0xA0..0xAF = firmware up */
		for (unsigned r = 0; r <= 10; r++) {
			e = rain_nvt_read(NVT_EVENT_BUF | 0x60u, nf.rst, 5);
			if (!e && nf.rst[0] >= 0xa0u && nf.rst[0] <= 0xafu) {
				return 0;
			}
			touch_wait(10);
		}
		(void)rain_nvt_read(NVT_CRC_FLAG, &nf.crcflag, 1);
		uint8_t c4[4] = { 0 };
		(void)rain_nvt_read(NVT_R_ILM_CRC, c4, 4);
		nf.rilm = le32(c4);
		(void)rain_nvt_read(NVT_R_DLM_CRC, c4, 4);
		nf.rdlm = le32(c4);
		e = -53;
	}
	return e;
}

static void
rain_nvt_bringup(void)
{
	DTEntry mm;
	void const *prop = NULL;
	unsigned int psize = 0;
	rn.stage = "nvt fw find";
	if (SecureDTLookupEntry(NULL, "/chosen/memory-map", &mm) != kSuccess ||
	    SecureDTGetProperty(mm, "TouchFW", &prop, &psize) != kSuccess || psize < 16) {
		rn.err = -54;                             /* payload built without --blob TouchFW= */
		return;
	}
	nf.fwpa = (uint32_t)((const uint64_t *)prop)[0];
	nf.fwlen = (uint32_t)((const uint64_t *)prop)[1];
	flush_dcache64((addr64_t)((const uint64_t *)prop)[0], nf.fwlen, TRUE);   /* the loader's copy may still be in cache */
	vm_offset_t fw = ml_io_map((vm_offset_t)((const uint64_t *)prop)[0], (vm_size_t)nf.fwlen);
	if (!fw) {
		rn.err = -55;
		return;
	}
	rn.stage = "nvt fw download";
	nf.err = rain_nvt_download((const volatile uint8_t *)fw, nf.fwlen);
	if (nf.err) {
		rn.err = nf.err;
		return;
	}
	rn.stage = "nvt fw info";
	touch_wait(50);
	uint8_t info[40];
	if (!rain_nvt_read(NVT_EVENT_BUF | 0x78u, info, 38)) {
		memcpy(nf.info, info, sizeof(nf.info));   /* ver, ~ver, x_num, y_num, xmax(2), ymax(2) ... */
	}

}

/*
 * C11: the driver loop. 10 fingers of 6 bytes at EVENT_BUF: id (1..10) << 3 | status
 * (1 enter, 2 move), x[11:4], y[11:4], x[3:0] << 4 | y[3:0]. Panel pixels 720x1650.
 * sweet_hid_touch divides by 1080x2400 and iOS is drawn into 720x1559 at y = 45
 * (sweetfb_fit), so x * 3/2 and (y - 45) * 2400/1559.
 */
static void
rain_touch_run(void)
{
	uint8_t pd[60];
	unsigned errs = 0;
	rn.stage = "nvt run";
	printf("RAIN TOUCH: firmware %02x up, %ux%u, polling every 10 ms\n", nf.info[0],
	    ((uint32_t)nf.info[4] << 8) | nf.info[5], ((uint32_t)nf.info[6] << 8) | nf.info[7]);
	for (;;) {
		int x[TOUCH_MAX] = { 0 }, y[TOUCH_MAX] = { 0 };
		uint32_t mask = 0;
		if (rain_nvt_read(NVT_EVENT_BUF, pd, sizeof(pd))) {
			if (++errs == 10) {
				touch_report(0, st.x, st.y);    /* release after persistent bad reads */
			}
			if (errs > 1000) {
				printf("RAIN TOUCH: giving up after %u read errors\n", errs);
				return;
			}
			touch_wait(10);
			continue;
		}
		errs = 0;
		for (unsigned i = 0; i < TOUCH_MAX; i++) {
			const uint8_t *f = pd + 6u * i;
			unsigned id = f[0] >> 3, status = f[0] & 7u;
			unsigned px = ((unsigned)f[1] << 4) | (f[3] >> 4);
			unsigned py = ((unsigned)f[2] << 4) | (f[3] & 0x0fu);
			if (id < 1 || id > TOUCH_MAX || (status != 1 && status != 2) || px > 719 || py > 1649) {
				continue;
			}
			unsigned ly = py < 45 ? 0 : (py - 45u) * 2400u / 1559u;
			x[id - 1] = (int)(px * 3u / 2u);
			y[id - 1] = (int)(ly > 2399 ? 2399 : ly);
			mask |= 1u << (id - 1);
		}
		touch_report(mask, x, y);
		touch_wait(10);
	}
}

