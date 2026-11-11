/*
 * rain: firmware files found by name in the modem partition's FAT (FAT16 or FAT32, 8.3 names),
 * instead of topaz's table of offsets measured on the Redmi Note 12's NON-HLOS.bin.
 * fat_rd(off, dst, len) reads the partition; fat_alloc/fat_free hold the FAT itself.
 * shortcut: 8.3 names only (no long names); enough for image/adsp.*, extend for wlanmdsp.mbn etc.
 */
static struct {
	bool tried, ok, fat32;
	uint32_t bps, spc, clus;            /* bytes a sector, sectors a cluster, bytes a cluster */
	uint64_t fat_off, root_off, data_off;
	uint32_t root_ents, root_clus, nclus;
	uint8_t *fat;
	uint32_t fat_len;
} fs;

static uint32_t fat_le16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t fat_le32(const uint8_t *p) { return fat_le16(p) | (fat_le16(p + 2) << 16); }

static bool
fat_mount(void)
{
	uint8_t bs[512];
	if (fs.tried) {
		return fs.ok;
	}
	fs.tried = true;
	if (fat_rd(0, bs, sizeof(bs)) != 0 || bs[510] != 0x55 || bs[511] != 0xaa) {
		return false;
	}
	fs.bps = fat_le16(bs + 11);
	fs.spc = bs[13];
	uint32_t rsv = fat_le16(bs + 14), nfats = bs[16], fatsz = fat_le16(bs + 22);
	uint32_t tot = fat_le16(bs + 19) ? fat_le16(bs + 19) : fat_le32(bs + 32);
	fs.root_ents = fat_le16(bs + 17);
	if (fs.bps < 512 || fs.bps > 4096 || (fs.bps & (fs.bps - 1)) || fs.spc == 0 || nfats == 0) {
		return false;
	}
	fs.fat32 = fatsz == 0;
	if (fs.fat32) {
		fatsz = fat_le32(bs + 36);
		fs.root_clus = fat_le32(bs + 44);
	}
	fs.clus = fs.bps * fs.spc;
	fs.fat_off = (uint64_t)rsv * fs.bps;
	fs.root_off = fs.fat_off + (uint64_t)nfats * fatsz * fs.bps;
	fs.data_off = fs.root_off + (uint64_t)((fs.root_ents * 32 + fs.bps - 1) / fs.bps) * fs.bps;
	fs.nclus = (uint32_t)((tot - fs.data_off / fs.bps) / fs.spc) + 2;
	fs.fat_len = fatsz * fs.bps;
	if (fs.fat_len == 0 || fs.fat_len > (16u << 20) || (fs.fat = (uint8_t *)fat_alloc(fs.fat_len)) == NULL) {
		return false;
	}
	if (fat_rd(fs.fat_off, fs.fat, fs.fat_len) != 0) {
		fat_free(fs.fat, fs.fat_len);
		fs.fat = NULL;
		return false;
	}
	fs.ok = true;
	return true;
}

static uint32_t
fat_next(uint32_t c)
{
	if (fs.fat32) {
		return (uint64_t)c * 4 + 4 <= fs.fat_len ? fat_le32(fs.fat + c * 4) & 0x0fffffffu : 0x0fffffffu;
	}
	uint32_t v = (uint64_t)c * 2 + 2 <= fs.fat_len ? fat_le16(fs.fat + c * 2) : 0xffffu;
	return v >= 0xfff8u ? 0x0fffffffu : v;
}

static bool
fat_clus_ok(uint32_t c)
{
	return c >= 2 && c < fs.nclus;
}

/* "adsp.b00" -> "ADSP    B00"; false if it is no 8.3 name */
static bool
fat_83(const char *s, size_t n, uint8_t out[11])
{
	memset(out, ' ', 11);
	size_t i = 0, j = 0;
	for (; i < n && s[i] != '.'; i++) {
		if (j >= 8) {
			return false;
		}
		out[j++] = (uint8_t)((s[i] >= 'a' && s[i] <= 'z') ? s[i] - 32 : s[i]);
	}
	if (i < n) {
		for (i++, j = 8; i < n; i++) {
			if (j >= 11 || s[i] == '.') {
				return false;
			}
			out[j++] = (uint8_t)((s[i] >= 'a' && s[i] <= 'z') ? s[i] - 32 : s[i]);
		}
	}
	return n > 0;
}

/* look name up in a directory (dir_clus 0: the FAT16 root) */
static bool
fat_dir_find(uint32_t dir_clus, const uint8_t want[11], uint32_t *clus, uint32_t *size, bool *isdir)
{
	uint8_t e[32];
	bool root16 = dir_clus == 0 && !fs.fat32;
	uint32_t c = dir_clus == 0 && fs.fat32 ? fs.root_clus : dir_clus;
	for (uint32_t guard = 0; guard < 65536; guard++) {
		uint64_t base, len;
		if (root16) {
			base = fs.root_off;
			len = (uint64_t)fs.root_ents * 32;
		} else {
			if (!fat_clus_ok(c)) {
				return false;
			}
			base = fs.data_off + (uint64_t)(c - 2) * fs.clus;
			len = fs.clus;
		}
		for (uint64_t o = 0; o < len; o += 32) {
			if (fat_rd(base + o, e, 32) != 0 || e[0] == 0) {
				return false;               /* end of directory */
			}
			if (e[0] == 0xe5 || e[11] == 0x0f || (e[11] & 0x08)) {
				continue;                   /* deleted, long-name part, volume label */
			}
			if (memcmp(e, want, 11) == 0) {
				*clus = fat_le16(e + 26) | (fs.fat32 ? fat_le16(e + 20) << 16 : 0);
				*size = fat_le32(e + 28);
				*isdir = (e[11] & 0x10) != 0;
				return true;
			}
		}
		if (root16) {
			return false;
		}
		c = fat_next(c);
	}
	return false;
}

/* "image/adsp.b00" -> first cluster and size of that file */
static bool
fat_lookup(const char *path, uint32_t *clus, uint32_t *size)
{
	uint32_t dir = 0;
	bool isdir = true;
	if (!fat_mount()) {
		return false;
	}
	for (const char *p = path; *p;) {
		const char *s = p;
		while (*p && *p != '/') {
			p++;
		}
		uint8_t want[11];
		if (!isdir || !fat_83(s, (size_t)(p - s), want) || !fat_dir_find(dir, want, clus, size, &isdir)) {
			return false;
		}
		dir = *clus;
		if (*p == '/') {
			p++;
		}
	}
	return !isdir;
}

/* read n bytes at off of the file starting at cluster first (size bytes long) */
static bool
fat_read_file(uint32_t first, uint32_t size, uint64_t off, void *dst, uint64_t n)
{
	if (off + n > size) {
		return false;
	}
	uint32_t c = first;
	uint64_t pos = 0, done = 0;
	for (uint32_t guard = 0; done < n && guard < fs.nclus; guard++) {
		if (!fat_clus_ok(c)) {
			return false;
		}
		if (off + done < pos + fs.clus) {
			uint64_t in = off + done - pos, take = fs.clus - in;
			if (take > n - done) {
				take = n - done;
			}
			if (fat_rd(fs.data_off + (uint64_t)(c - 2) * fs.clus + in, (uint8_t *)dst + done, take) != 0) {
				return false;
			}
			done += take;
		}
		pos += fs.clus;
		c = fat_next(c);
	}
	return done == n;
}
