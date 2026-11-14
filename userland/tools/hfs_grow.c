/*
 * rain: grow the mounted iOS root (journaled HFS+, built as a 16 GiB image) to fill the
 * userdata partition, in place. Run once at boot by launchd (org.rain.hfs-grow); a no-op
 * when the volume already fills the partition. Output goes to /dev/console (kernel log).
 *
 * The partition size comes from the UFS driver (/dev/rufsroot0). The last 16 MiB stay
 * free: the kernel's persistent console log ("sweetlog") owns the last 4 MiB + 4 KiB.
 * HFSIOC_RESIZE_VOLUME = _IOW('h', 2, u_int64_t), handled by hfs_extendfs (hfs_resize.c).
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/disk.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#ifndef HFSIOC_RESIZE_VOLUME
#define HFSIOC_RESIZE_VOLUME _IOW('h', 2, u_int64_t)
#endif
int fsctl(const char *, unsigned long, void *, unsigned int);

#define RESERVE (16ull << 20)
#define SLACK   (64ull << 20)   /* closer than this counts as already grown */

int
main(void)
{
	struct statfs s;
	uint64_t count = 0;
	uint32_t bs = 0;
	int fd = open("/dev/rufsroot0", O_RDONLY);

	if (fd < 0 || ioctl(fd, DKIOCGETBLOCKCOUNT, &count) || ioctl(fd, DKIOCGETBLOCKSIZE, &bs) || statfs("/", &s)) {
		printf("hfs-grow: cannot read sizes: %s\n", strerror(errno));
		return 1;
	}
	close(fd);
	uint64_t part = count * bs, cur = (uint64_t)s.f_blocks * s.f_bsize;
	uint64_t want = (part - RESERVE) / s.f_bsize * s.f_bsize;
	printf("hfs-grow: %s partition %.2f GiB, volume %.2f GiB, target %.2f GiB\n", s.f_fstypename,
	    part / 1073741824.0, cur / 1073741824.0, want / 1073741824.0);
	if (strcmp(s.f_fstypename, "hfs") || part < 2 * RESERVE || want <= cur + SLACK) {
		printf("hfs-grow: nothing to do\n");
		return 0;
	}
	if (fsctl("/", HFSIOC_RESIZE_VOLUME, &want, 0)) {
		printf("hfs-grow: resize failed: %s (%d), volume unchanged\n", strerror(errno), errno);
		return 1;
	}
	statfs("/", &s);
	printf("hfs-grow: done, volume now %.2f GiB, %.2f GiB free\n",
	    (double)s.f_blocks * s.f_bsize / 1073741824.0, (double)s.f_bavail * s.f_bsize / 1073741824.0);
	return 0;
}
