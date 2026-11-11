#!/bin/bash
# rain: put the GPU user space (and the audio plug-in) into the iOS root image.
#   /topaz/lib/{libOSMesa.8,libtopaz-gl,libtopaz-inject}.dylib
#   /usr/libexec/backboardd -> wrapper; the real one moves to /usr/libexec/real/backboardd
#   /System/Library/Audio/Plug-Ins/HAL/RedwoodAudio.driver
# Without boot-arg gpu=1 the loader does nothing, so the image still boots as before.
# Needs userland/out-gpu (artifact "gpu-userland" of the userland workflow).
# Run from the ios-topaz folder:  sudo bash build-rain/install-gpu-root.sh
set -euo pipefail
cd "$(dirname "$0")/.."
img=research/ipsw/ios16-root.img
G=userland/out-gpu
for f in libOSMesa.8.dylib libtopaz-gl.dylib libtopaz-inject.dylib backboardd RedwoodAudio RedwoodAudio.Info.plist; do
	[ -s "$G/$f" ] || { echo "missing $G/$f"; exit 1; }
done
. build-rain/lib-fsck.sh
fsck_clean

mnt=$(mktemp -d)
mount -t hfsplus -o loop,force "$img" "$mnt"
trap 'umount "$mnt" 2>/dev/null || true; rmdir "$mnt"' EXIT
L="$mnt/usr/libexec"
if [ ! -e "$L/real/backboardd" ]; then
	# first run: the file there must be Apple's window server (a Mach-O), not something of ours
	[ "$(head -c4 "$L/backboardd" | xxd -p)" = cffaedfe ] || { echo "unexpected $L/backboardd"; exit 1; }
	install -d -m 755 -o 0 -g 0 "$L/real"
	mv "$L/backboardd" "$L/real/backboardd"
fi
install -m 755 -o 0 -g 0 "$G/backboardd" "$L/backboardd"
install -d -m 755 -o 0 -g 0 "$mnt/topaz" "$mnt/topaz/lib"
install -m 755 -o 0 -g 0 "$G"/*.dylib "$mnt/topaz/lib/"
A="$mnt/System/Library/Audio/Plug-Ins/HAL/RedwoodAudio.driver"
install -d -m 755 -o 0 -g 0 "$A"
install -m 755 -o 0 -g 0 "$G/RedwoodAudio" "$A/RedwoodAudio"
install -m 644 -o 0 -g 0 "$G/RedwoodAudio.Info.plist" "$A/Info.plist"
ls -la "$L/backboardd" "$L/real/backboardd" "$mnt/topaz/lib" "$A"
umount "$mnt"
trap - EXIT
rmdir "$mnt"

fsck_clean	# the mount touched the volume again
sha256sum "$img" > research/ipsw/ios16-root.img.sha256
chown "${SUDO_UID:-0}:${SUDO_GID:-0}" "$img" research/ipsw/ios16-root.img.sha256
echo "DONE: GPU user space installed in $img"
