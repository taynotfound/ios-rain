/*
 * Just enough flattened-device-tree parsing to find what ABL put in
 * /chosen (initrd location). Big-endian, 4-byte aligned tokens.
 */
#include "fdt.h"

#define FDT_MAGIC	0xd00dfeed
#define FDT_BEGIN_NODE	1
#define FDT_END_NODE	2
#define FDT_PROP	3
#define FDT_NOP		4
#define FDT_END		9

static uint32_t be32(uintptr_t a)
{
	return __builtin_bswap32(*(volatile uint32_t *)a);
}

static int streq(const char *a, const char *b)
{
	while (*a && *a == *b)
		a++, b++;
	return *a == *b;
}

static uintptr_t align4(uintptr_t a)
{
	return (a + 3) & ~3UL;
}

static uintptr_t skip_str(uintptr_t p)
{
	/* byte loads are fine on Device memory as long as they are aligned to 1 */
	while (*(volatile char *)p)
		p++;
	return p + 1;
}

/*
 * DDR banks ABL wrote into /memory reg (2 address + 2 size cells, as on SM6225).
 * Fills up to max (base, size) pairs; returns how many.
 */
int fdt_memory(uintptr_t fdt, uint64_t *base, uint64_t *size, int max)
{
	if (!fdt || be32(fdt) != FDT_MAGIC)
		return 0;
	uintptr_t p = fdt + be32(fdt + 8);
	uintptr_t strings = fdt + be32(fdt + 12);
	int depth = 0, in_mem = 0, n = 0;

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE: {
			const char *name = (const char *)p;
			depth++;
			/* "memory" or "memory@..." directly under / */
			in_mem = depth == 2 && name[0] == 'm' && name[1] == 'e' && name[2] == 'm' &&
				 name[3] == 'o' && name[4] == 'r' && name[5] == 'y' &&
				 (name[6] == 0 || name[6] == '@');
			p = align4(skip_str(p));
			break;
		}
		case FDT_END_NODE:
			if (in_mem)
				return n;
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			if (in_mem && streq((const char *)(strings + nameoff), "reg")) {
				for (uint32_t o = 0; o + 16 <= len && n < max; o += 16, n++) {
					base[n] = (uint64_t)be32(val + o) << 32 | be32(val + o + 4);
					size[n] = (uint64_t)be32(val + o + 8) << 32 | be32(val + o + 12);
				}
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:
			return n;
		}
	}
}

/*
 * Find needle in /chosen bootargs and copy what follows it (up to a space or
 * comma) into out. Returns 1 if found.
 */
int fdt_bootarg(uintptr_t fdt, const char *needle, char *out, int outlen)
{
	if (!fdt || be32(fdt) != FDT_MAGIC || outlen < 1)
		return 0;
	uintptr_t p = fdt + be32(fdt + 8);
	uintptr_t strings = fdt + be32(fdt + 12);
	int depth = 0, in_chosen = 0;

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE:
			depth++;
			in_chosen = depth == 2 && streq((const char *)p, "chosen");
			p = align4(skip_str(p));
			break;
		case FDT_END_NODE:
			if (in_chosen)
				return 0;
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			if (in_chosen && streq((const char *)(strings + nameoff), "bootargs")) {
				const volatile char *s = (const volatile char *)val;
				for (uint32_t i = 0; i < len && s[i]; i++) {
					uint32_t k = 0;
					while (needle[k] && i + k < len && s[i + k] == needle[k])
						k++;
					if (needle[k])
						continue;
					int n = 0;
					for (uint32_t j = i + k; j < len && s[j] && s[j] != ' ' && s[j] != ',' && s[j] != ':' && n < outlen - 1; j++)
						out[n++] = s[j];
					out[n] = 0;
					return 1;
				}
				return 0;
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:
			return 0;
		}
	}
}

int fdt_chosen_u64(uintptr_t fdt, const char *prop, uint64_t *out)
{
	if (!fdt || be32(fdt) != FDT_MAGIC)
		return 0;
	uintptr_t p = fdt + be32(fdt + 8);		/* off_dt_struct */
	uintptr_t strings = fdt + be32(fdt + 12);	/* off_dt_strings */
	int depth = 0, in_chosen = 0;

	for (;;) {
		uint32_t tok = be32(p);
		p += 4;
		switch (tok) {
		case FDT_BEGIN_NODE: {
			const char *name = (const char *)p;
			depth++;
			in_chosen = depth == 2 && streq(name, "chosen");
			p = align4(skip_str(p));
			break;
		}
		case FDT_END_NODE:
			if (in_chosen)
				return 0;
			depth--;
			break;
		case FDT_PROP: {
			uint32_t len = be32(p), nameoff = be32(p + 4);
			uintptr_t val = p + 8;
			if (in_chosen && streq((const char *)(strings + nameoff), prop)) {
				if (len == 4)
					*out = be32(val);
				else if (len == 8)
					*out = (uint64_t)be32(val) << 32 | be32(val + 4);
				else
					return 0;
				return 1;
			}
			p = align4(val + len);
			break;
		}
		case FDT_NOP:
			break;
		default:	/* FDT_END or garbage */
			return 0;
		}
	}
}
