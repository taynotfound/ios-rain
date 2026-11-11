#!/bin/bash
# rain: finish an ios16-root.img whose copy completed but whose final
# `fsck.hfsplus -n` still reported "Volume header needs minor repair".
# Repairs until a read-only check passes, then tweaks + sparse image.
# Run from the ios-topaz folder:  sudo bash build-rain/finish-iosroot.sh
set -euo pipefail
cd "$(dirname "$0")/.."
I=research/ipsw
img=$I/ios16-root.img

. build-rain/lib-fsck.sh

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
