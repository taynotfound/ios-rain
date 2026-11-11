#!/bin/bash
# Build a boot image from a kernel and fastboot-boot it once the phone answers.
#   build-rain/boot-probe.sh <github-run-id | kernel file> <patch-no> <tag> [extra cmdline]
# e.g. build-rain/boot-probe.sh 42492125916 0979 C10
#      build-rain/boot-probe.sh ~/Downloads/kernel.development.sweet 0991 D2   (from Codemagic, say)
# Debugging a probe that ends in a panic: pass '-v cpus=1 sweetfb_noconsole=0' as the 4th argument is not
# enough (the first cpus= wins); edit the line below instead.
# Needs research/fw-fog/novatek_ts_fw.bin (vendor firmware, not in git) for the touch stages.
set -e
cd "$(dirname "$0")/.."
P=$PWD R=$1 N=$2 T=$3
if [ -f "$R" ]; then
	K=$(cd "$(dirname "$R")" && pwd)/$(basename "$R")
	mkdir -p "xnu/out-$N" && [ "$K" = "$P/xnu/out-$N/kernel.development.sweet" ] || cp "$K" "xnu/out-$N/kernel.development.sweet"
else
	gh run watch "$R" -R taynotfound/ios-rain --exit-status -i 30 >/dev/null
	rm -rf "xnu/out-$N" && gh run download "$R" -R taynotfound/ios-rain -n xnu-kernel -D "xnu/out-$N"
	(cd "xnu/out-$N" && sha256sum -c SHA256SUMS)
fi
FW=research/fw-fog/novatek_ts_fw.bin
python3 tools/mkpayload.py "xnu/out-$N/kernel.development.sweet" build-rain/rain-ios-2g.adt "build-rain/payload-$T.bin" \
  ${FW:+$( [ -f "$FW" ] && echo --blob TouchFW=$FW )} \
  --cmdline "debug=0xc serial=3 cpus=4 keepsyms=1 -noprogress rd=ufsroot0 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin ufsrw=1 rootrw=1 launchd_unsecure_cache=1 wdt=-1 sweettouch=0 sweetchg=0 sweetfb_noshadow=1 sweetfb_fit=2 raintouch=1 sweettouchdelay=0 raintouchpause=0 sweetfb_noconsole=1 sweetfb_hz=60 gpu=1 gpu_mhz=600 $4" >/dev/null
PATH=$P/build-rain/bin:$PATH MKBOOTIMG=mkbootimg bash shim/build.sh "$P/build-rain/payload-$T.bin" >/dev/null 2>&1
cp shim/out/shim-boot.img "build-rain/shim-xnu-rain-touch$T.img"
# Xiaomi's ABL stops taking data after idling in fastboot: time the transfer out and wait for a fresh session
until
  echo "waiting for fastboot $(date +%T)"
  ok=0; until [ $ok -ge 2 ]; do if timeout 3 fastboot getvar product 2>&1 | grep -q 'product: rain'; then ok=$((ok+1)); else ok=0; fi; sleep 1; done
  timeout 30 fastboot boot "build-rain/shim-xnu-rain-touch$T.img"
do echo "transfer hung: re-enter fastboot (Vol- + Power) $(date +%T)"; sleep 5; done
echo "booted $(date +%T)"
