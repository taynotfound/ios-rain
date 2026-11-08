#!/bin/bash
# Build a boot image from a CI kernel and fastboot-boot it once the phone answers.
#   build-rain/boot-probe.sh <run-id> <patch-no> <tag> [extra cmdline]
# e.g. build-rain/boot-probe.sh 42492125916 0979 C10
# Needs research/fw-fog/novatek_ts_fw.bin (vendor firmware, not in git) for the touch stages.
set -e
cd "$(dirname "$0")/.."
P=$PWD R=$1 N=$2 T=$3
gh run watch "$R" -R taynotfound/ios-topaz --exit-status -i 30 >/dev/null
rm -rf "xnu/out-$N" && gh run download "$R" -R taynotfound/ios-topaz -n xnu-kernel -D "xnu/out-$N"
(cd "xnu/out-$N" && sha256sum -c SHA256SUMS)
FW=research/fw-fog/novatek_ts_fw.bin
python3 tools/mkpayload.py "xnu/out-$N/kernel.development.sweet" build-rain/rain-ios.adt "build-rain/payload-$T.bin" \
  ${FW:+$( [ -f "$FW" ] && echo --blob TouchFW=$FW )} \
  --cmdline "-v debug=0xc serial=3 cpus=1 keepsyms=1 rd=ufsroot0 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin ufsrw=1 rootrw=1 launchd_unsecure_cache=1 wdt=-1 sweettouch=0 sweetchg=0 sweetfb_noshadow=1 sweetfb_fit=1 raintouch=1 sweettouchdelay=0 raintouchpause=0 $4" >/dev/null
PATH=$P/build-rain/bin:$PATH MKBOOTIMG=mkbootimg bash shim/build.sh "$P/build-rain/payload-$T.bin" >/dev/null 2>&1
cp shim/out/shim-boot.img "build-rain/shim-xnu-rain-touch$T.img"
echo "waiting for fastboot $(date +%T)"
ok=0; until [ $ok -ge 2 ]; do if timeout 3 fastboot getvar product 2>&1 | grep -q 'product: rain'; then ok=$((ok+1)); else ok=0; fi; sleep 1; done
fastboot boot "build-rain/shim-xnu-rain-touch$T.img" && echo "booted $(date +%T)"
