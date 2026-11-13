#!/bin/bash
# rain: re-run tools/sweet-root-tweaks.py on the existing iOS root image (idempotent), e.g. after a
# new tweak (bluetoothd removed). Then reflash: fastboot -S 256M flash userdata research/ipsw/ios16-root.img
# Run from the ios-topaz folder:  sudo bash build-rain/apply-root-tweaks.sh
set -euo pipefail
cd "$(dirname "$0")/.."
img=research/ipsw/ios16-root.img
. build-rain/lib-fsck.sh
fsck_clean
mnt=$(mktemp -d)
mount -t hfsplus -o loop,force "$img" "$mnt"
trap 'umount "$mnt" 2>/dev/null || true; rmdir "$mnt"' EXIT
python3 tools/sweet-root-tweaks.py "$mnt"
umount "$mnt"
trap - EXIT
rmdir "$mnt"
fsck_clean
echo "done: $img"
