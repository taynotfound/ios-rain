#!/bin/bash
# rain: build the iOS 16.7.16 (iPhone X) root image for userdata, as in README step 2.
# Run from the ios-topaz folder:  sudo bash build-rain/build-iosroot.sh
set -euo pipefail
cd "$(dirname "$0")/.."
I=research/ipsw
out=$I/ios16-root.img

bash tools/mkiosroot.sh $I/OS.dmg $I/SystemOS.dmg $I/AppOS.dmg "$out" 16G

mnt=$(mktemp -d)
mount -t hfsplus -o loop,force "$out" "$mnt"
trap 'umount "$mnt" 2>/dev/null || true; rmdir "$mnt"' EXIT
python3 tools/sweet-root-tweaks.py "$mnt"
umount "$mnt"
trap - EXIT
rmdir "$mnt"

img2simg "$out" $I/ios16-root.simg
sha256sum "$out" > $I/ios16-root.img.sha256
# hand the results back to the normal user
chown "${SUDO_UID:-0}:${SUDO_GID:-0}" "$out" $I/ios16-root.simg $I/ios16-root.img.sha256
ls -la "$out" $I/ios16-root.simg
echo "DONE: iOS root image built"
