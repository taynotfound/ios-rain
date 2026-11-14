#!/bin/bash
# rain: put the GPU user space (and the audio plug-in) into the iOS root image.
#   /topaz/lib/{libOSMesa.8,libtopaz-gl,libtopaz-inject}.dylib
#   /usr/libexec/backboardd -> wrapper; the real one moves to /usr/libexec/real/backboardd
#   /System/Library/Audio/Plug-Ins/HAL/RedwoodAudio.driver
#   /topaz/bin/hfs-grow (run once per boot by launchd, see tools/sweet-root-tweaks.py)
#   /usr/share/firmware/topaz/{a610_zap.mdt,a610_zap.b02,a630_sqe.fw}: Adreno firmware from this phone's
#     own vendor image (research/fw-fog/gpu, not in git; the zap shader is signed for the device)
# Without boot-arg gpu=1 the loader does nothing, so the image still boots as before.
# Needs userland/out-gpu (artifact "gpu-userland" of the userland workflow).
# Run from the ios-topaz folder:  sudo bash build-rain/install-gpu-root.sh
set -euo pipefail
cd "$(dirname "$0")/.."
img=research/ipsw/ios16-root.img
G=userland/out-gpu
for f in libOSMesa.8.dylib libtopaz-gl.dylib libtopaz-inject.dylib backboardd RedwoodAudio RedwoodAudio.Info.plist hfs-grow; do
	[ -s "$G/$f" ] || { echo "missing $G/$f"; exit 1; }
done
FW=research/fw-fog/gpu
for f in a610_zap.mdt a610_zap.b02 a630_sqe.fw; do
	[ -s "$FW/$f" ] || { echo "missing $FW/$f (debugfs -R 'dump /firmware/$f ...' vendor.img)"; exit 1; }
done
# an unsigned wrapper makes launchd panic at boot: refuse to install one
xxd -p "$G/backboardd" | tr -d '\n' | grep -q fade0cc0 || { echo "$G/backboardd has no code signature"; exit 1; }
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
install -d -m 755 -o 0 -g 0 "$mnt/topaz/bin"
install -m 755 -o 0 -g 0 "$G/hfs-grow" "$mnt/topaz/bin/hfs-grow"
A="$mnt/System/Library/Audio/Plug-Ins/HAL/RedwoodAudio.driver"
install -d -m 755 -o 0 -g 0 "$A"
install -m 755 -o 0 -g 0 "$G/RedwoodAudio" "$A/RedwoodAudio"
install -m 644 -o 0 -g 0 "$G/RedwoodAudio.Info.plist" "$A/Info.plist"
F="$mnt/usr/share/firmware/topaz"
install -d -m 755 -o 0 -g 0 "$F"
install -m 644 -o 0 -g 0 "$FW"/a610_zap.mdt "$FW"/a610_zap.b02 "$FW"/a630_sqe.fw "$F/"
ls -la "$F" "$L/backboardd" "$L/real/backboardd" "$mnt/topaz/lib" "$A"
umount "$mnt"
trap - EXIT
rmdir "$mnt"

fsck_clean	# the mount touched the volume again
sha256sum "$img" > research/ipsw/ios16-root.img.sha256
chown "${SUDO_UID:-0}:${SUDO_GID:-0}" "$img" research/ipsw/ios16-root.img.sha256
echo "DONE: GPU user space installed in $img"
