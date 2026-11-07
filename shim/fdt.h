#pragma once
#include <stdint.h>

/* Read a u32/u64 (cell-size agnostic) property of /chosen. Returns 0 if absent. */
int fdt_chosen_u64(uintptr_t fdt, const char *prop, uint64_t *out);
/* DDR banks from /memory reg; returns the count (at most max). */
int fdt_memory(uintptr_t fdt, uint64_t *base, uint64_t *size, int max);
/* Copy what follows needle in /chosen bootargs (to a space or comma); 1 if found. */
int fdt_bootarg(uintptr_t fdt, const char *needle, char *out, int outlen);
