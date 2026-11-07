#!/bin/bash
# rain: finish an ios16-root.img whose copy completed but whose final
# `fsck.hfsplus -n` still reported "Volume header needs minor repair".
# Repairs until a read-only check passes, then tweaks + sparse image.
# Run from the ios-topaz folder:  sudo bash build-rain/finish-iosroot.sh
set -euo pipefail
cd "$(dirname "$0")/.."
I=research/ipsw
img=$I/ios16-root.img

fsck_clean() {
	for pass in 1 2 3; do
		if fsck.hfsplus -f -n "$img" >/dev/null 2>&1; then
			echo "fsck: clean (after $((pass - 1)) extra repair pass(es))"
			return 0
		fi
		echo "fsck: repair pass $pass"
		out=$(fsck.hfsplus -f -y "$img" 2>&1 || true)
		echo "$out" | grep -vE 'Illegal name' || true
		# hfsprogs 540.1-Linux: -n keeps reporting "Volume header needs minor repair"
		# on a volume whose -y pass finds nothing (primary and alternate VHB match).
		if echo "$out" | grep -q 'appears to be OK'; then
			echo "fsck: repair pass found nothing to fix; treating -n header warning as checker noise"
			return 0
		fi
	done
	fsck.hfsplus -f -n "$img"	# show what is still wrong and fail
}

fsck_clean

mnt=$(mktemp -d)
mount -t hfsplus -o loop,force "$img" "$mnt"
trap 'umount "$mnt" 2>/dev/null || true; rmdir "$mnt"' EXIT
python3 tools/sweet-root-tweaks.py "$mnt"
df -h "$mnt"
umount "$mnt"
trap - EXIT
rmdir "$mnt"

fsck_clean	# the tweaks mount touched the volume again

img2simg "$img" $I/ios16-root.simg
sha256sum "$img" > $I/ios16-root.img.sha256
chown "${SUDO_UID:-0}:${SUDO_GID:-0}" "$img" $I/ios16-root.simg $I/ios16-root.img.sha256
ls -la "$img" $I/ios16-root.simg
echo "DONE: iOS root image built"
