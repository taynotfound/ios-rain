import sys
p=sys.argv[1]; s=open(p).read()
def rep(o,n,cnt=1):
    global s
    assert s.count(o)==cnt,(s.count(o),o[:70]); s=s.replace(o,n)

rep("#include <arm/caches_internal.h>\n", "#include <arm/caches_internal.h>\n#include <pexpert/device_tree.h>\n")

# layout: bigger TX buffer for firmware chunks
rep("#define RM_TXBUF       0x1000u\n#define RM_RXBUF       0x1100u\n",
    "#define RM_TXBUF       0x1000u     /* up to RM_TXMAX bytes */\n#define RM_RXBUF       0x2400u     /* up to RM_RXMAX bytes */\n#define RM_TXMAX       0x1100u\n#define RM_RXMAX       0x400u\n")

# transfer: wrap the rings, size limits
rep("\tif (n == 0 || n > 0x100u || rg.tx_wp + 3u > RG_NTRE || rg.rx_wp + 1u > RG_NTRE) {\n\t\treturn -30;                     /* probe never wraps the rings */\n\t}\n",
    "\tif (n == 0 || n > (cmd == 3u ? RM_RXMAX : RM_TXMAX) || !rg.uc) {\n\t\treturn -30;\n\t}\n")
rep("\t\train_tre_dma(&rxr[rg.rx_wp], rg.pa + RM_RXBUF, n, false);\n",
    "\t\train_tre_dma(&rxr[rg.rx_wp % RG_NTRE], rg.pa + RM_RXBUF, n, false);\n")
rep("\train_tre_config(&txr[rg.tx_wp], 16, rg.loopback);          /* 19.2 MHz / 16 */\n\train_tre_go(&txr[rg.tx_wp + 1u], cmd, cmd == 3u ? n : 0);\n\train_tre_dma(&txr[rg.tx_wp + 2u], rg.pa + RM_TXBUF, n, true);\n",
    "\train_tre_config(&txr[rg.tx_wp % RG_NTRE], 16, rg.loopback);          /* 19.2 MHz / 16 */\n\train_tre_go(&txr[(rg.tx_wp + 1u) % RG_NTRE], cmd, cmd == 3u ? n : 0);\n\train_tre_dma(&txr[(rg.tx_wp + 2u) % RG_NTRE], rg.pa + RM_TXBUF, n, true);\n")
rep("\tclean_dcache64((addr64_t)(rg.pa + RM_TXBUF), 0x100, TRUE);\n\tflush_dcache64((addr64_t)(rg.pa + RM_RXBUF), 0x100, TRUE);\n",
    "\tclean_dcache64((addr64_t)(rg.pa + RM_TXBUF), RM_TXMAX, TRUE);\n\tflush_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);\n")
rep("\t\trg.rx_wp += 1u;\n\t\trg_wr(RG_CH_DB(1), (uint32_t)(rg.pa + RM_RX + rg.rx_wp * 16u));\n",
    "\t\trg.rx_wp = (rg.rx_wp + 1u) % RG_NTRE;\n\t\trg_wr(RG_CH_DB(1), (uint32_t)(rg.pa + RM_RX + rg.rx_wp * 16u));\n")
rep("\trg.tx_wp += 3u;\n\trg_wr(RG_CH_DB(0), (uint32_t)(rg.pa + RM_TX + rg.tx_wp * 16u));\n",
    "\trg.tx_wp = (rg.tx_wp + 3u) % RG_NTRE;\n\trg_wr(RG_CH_DB(0), (uint32_t)(rg.pa + RM_TX + rg.tx_wp * 16u));\n")
# events through the uncached alias, consumed entries zeroed so the ring can wrap
rep("\tstruct rain_tre *ev = (struct rain_tre *)(rg.mem + RM_EV);\n",
    "\tvolatile struct rain_tre *ev = (volatile struct rain_tre *)(rg.uc + RM_EV);   /* uncached: entries are zeroed after use */\n")
rep("\t\tflush_dcache64((addr64_t)(rg.pa + RM_EV), RG_NEV * 16u, TRUE);\n\t\twhile (rg.ev_rp < RG_NEV && ev[rg.ev_rp].w[3] != 0) {\n",
    "\t\twhile (ev[rg.ev_rp].w[3] != 0) {\n")
rep("\t\t\t\t\tmemcpy((void *)(rg.mem + RM_RXBUF), (const void *)&ev[rg.ev_rp].w[0], len > 8u ? 8u : len);\n\t\t\t\t\tclean_dcache64((addr64_t)(rg.pa + RM_RXBUF), 0x100, TRUE);\n",
    "\t\t\t\t\tuint32_t imm[2] = { ev[rg.ev_rp].w[0], ev[rg.ev_rp].w[1] };\n\t\t\t\t\tmemcpy((void *)(rg.mem + RM_RXBUF), imm, len > 8u ? 8u : len);\n\t\t\t\t\tclean_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);\n")
rep("\t\t\trg.ev_rp++;\n",
    "\t\t\tev[rg.ev_rp].w[3] = 0;\n\t\t\trg.ev_rp = (rg.ev_rp + 1u) % RG_NEV;\n")
rep("\t\tflush_dcache64((addr64_t)(rg.pa + RM_RXBUF), 0x100, TRUE);\n\t\tmemcpy(rg.cached, (const void *)(rg.mem + RM_RXBUF), 8);\n",
    "\t\tflush_dcache64((addr64_t)(rg.pa + RM_RXBUF), RM_RXMAX, TRUE);\n\t\tmemcpy(rg.cached, (const void *)(rg.mem + RM_RXBUF), 8);\n")

# reads of any length up to RM_RXMAX - 2
rep("\tuint8_t tx[32] = { 0 }, rx[32] = { 0 };\n", "\tstatic uint8_t tx[RM_RXMAX], rx[RM_RXMAX];\n\tbzero(tx, sizeof(tx));\n")

# firmware download + touch poll, after the ID reads
rep("\trn.err = e1 ? e1 : e2;\n\trn.stage = \"done\";\n}\n",
    "\trn.err = e1 ? e1 : e2;\n\tif (!rn.err && rn.id[0] == 0x0bu && rn.id[3] == 0x25u && rn.id[4] == 0x65u && rn.id[5] == 0x03u) {\n\t\train_nvt_bringup();\n\t\tif (rn.err) {\n\t\t\treturn;\n\t\t}\n\t}\n\trn.stage = \"done\";\n}\n")

BRING = r'''
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

	/* poll the point buffer for 30 s: 65 bytes at EVENT_BUF, finger i at 1 + 6 i */
	rn.stage = "nvt touch poll";
	printf("RAIN TOUCH: firmware up, touch the screen now (30 s)\n");
	uint8_t pd[66];
	for (unsigned t = 0; t < 3000; t++) {
		if (rain_nvt_read(NVT_EVENT_BUF, pd, 65)) {
			break;
		}
		nf.polls++;
		const uint8_t *f = pd + 1;
		uint32_t id = f[0] >> 3, st = f[0] & 7u;
		if (id >= 1 && id <= 10 && (st == 1 || st == 2)) {
			uint32_t x = ((uint32_t)f[1] << 4) | (f[3] >> 4);
			uint32_t y = ((uint32_t)f[2] << 4) | (f[3] & 0x0fu);
			if (!nf.downs) {
				nf.fx = x;
				nf.fy = y;
				memcpy(nf.raw, pd, sizeof(nf.raw));
			}
			nf.lx = x;
			nf.ly = y;
			nf.downs++;
		}
		touch_wait(10);
	}
}
'''
rep("\nstatic void\nrain_touch_probe(void)\n{\n", BRING + "\nstatic void\nrain_touch_probe(void)\n{\n")

rep("\tpanic(\"RAIN TOUCH C9: ", "\tpanic(\"RAIN TOUCH C10: ")
rep("| cbf %d fsr=%#x far=%#x syn=%#x sid=%#x\",",
    "| cbf %d fsr=%#x far=%#x syn=%#x sid=%#x | FW pa=%#x len=%#x parts=%u try=%u wrote=%#x err=%d rst %02x %02x crcflag=%#x rilm=%#x rdlm=%#x | "
    "INFO %02x %02x x%u y%u max %u %u | TOUCH polls=%u downs=%u first %u,%u last %u,%u raw %02x %02x %02x %02x %02x %02x %02x\",")
rep("r9.fcb, r9.ffsr, r9.ffar, r9.fsyn, r9.fsid);",
    "r9.fcb, r9.ffsr, r9.ffar, r9.fsyn, r9.fsid,\n"
    "\t    nf.fwpa, nf.fwlen, nf.nparts, nf.tries, nf.wrote, nf.err, nf.rst[0], nf.rst[1], nf.crcflag, nf.rilm, nf.rdlm,\n"
    "\t    nf.info[0], nf.info[1], nf.info[2], nf.info[3], ((uint32_t)nf.info[4] << 8) | nf.info[5], ((uint32_t)nf.info[6] << 8) | nf.info[7],\n"
    "\t    nf.polls, nf.downs, nf.fx, nf.fy, nf.lx, nf.ly,\n"
    "\t    nf.raw[0], nf.raw[1], nf.raw[2], nf.raw[3], nf.raw[4], nf.raw[5], nf.raw[6]);")
open(p,"w").write(s); print("ok")
