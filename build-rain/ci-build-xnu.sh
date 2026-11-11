#!/bin/bash
# The kernel build of .github/workflows/xnu.yml as one script, for any macOS CI with Xcode
# installed (Codemagic: codemagic.yaml). Run from the repository root. Output: out/ with
# kernel.development.sweet and SHA256SUMS, plus buildtree.tgz (sources and generated headers).
#   XCODE_APP=/Applications/Xcode-15.4.app  pick an Xcode (default: the oldest installed;
#                                            Xcode 15.x's iig matches xnu-8796's IORPC ABI)
set -euo pipefail
ROOT=$(pwd)
export DXB_REPO=https://github.com/blacktop/darwin-xnu-build
export DXB_SHA=663906528d1cd5ea8fde7de6f16de73db9511549
export MACOS_VERSION=${MACOS_VERSION:-13.5} KERNEL_CONFIG=${KERNEL_CONFIG:-DEVELOPMENT} MACHINE_CONFIG=${MACHINE_CONFIG:-SWEET}
export CCACHE_DIR=${CCACHE_DIR:-$ROOT/.ccache} CCACHE_BASEDIR=$ROOT CCACHE_COMPILERCHECK=content
export CCACHE_SLOPPINESS=time_macros,include_file_mtime,include_file_ctime,file_macro,pch_defines CCACHE_MAXSIZE=3G
export CCACHE_WRAP=$ROOT/ccache-bin
export MAKEFLAGS=${MAKEFLAGS:--k}

# build script, our patches next to blacktop's
rm -rf dxb && git init -q dxb
git -C dxb fetch -q --depth 1 "$DXB_REPO" "$DXB_SHA"
git -C dxb checkout -q FETCH_HEAD
cp xnu/patches/*.patch xnu/patches/*.sh dxb/patches/
# libfirehose_kernel as plain arm64 for an ARMv8.0 Cortex-A53/A73
sed -i '' 's/ARCHS="x86_64 arm64e" VALID_ARCHS="x86_64 arm64e"/ARCHS="arm64" VALID_ARCHS="arm64" OTHER_CFLAGS="-mkernel -nostdinc -Wno-packed -mcpu=cortex-a53"/' dxb/build.sh
grep -q 'OTHER_CFLAGS="-mkernel -nostdinc -Wno-packed -mcpu=cortex-a53"' dxb/build.sh
sed -i '' 's|export CC="$(xcrun -find clang)"|export CC="$CCACHE_WRAP/clang"|; s|export CXX="$(xcrun -find clang++)"|export CXX="$CCACHE_WRAP/clang++"|' dxb/build.sh
grep -q 'export CC="$CCACHE_WRAP/clang"' dxb/build.sh

# tools
brew install jq gum bash cmake ninja ccache
command -v xcodes >/dev/null || { printf '#!/bin/sh\nexit 0\n' | sudo tee /usr/local/bin/xcodes >/dev/null; sudo chmod +x /usr/local/bin/xcodes; }
XC=${XCODE_APP:-$(ls -d /Applications/Xcode*.app | sort -V | head -1)}
export DEVELOPER_DIR=$XC/Contents/Developer
sudo xcode-select -s "$XC"
xcodebuild -version
mkdir -p "$CCACHE_WRAP"
for t in clang clang++; do
	printf '#!/bin/sh\nexec ccache %s "$@"\n' "$(xcrun -find $t)" > "$CCACHE_WRAP/$t"
	chmod +x "$CCACHE_WRAP/$t"
done

# build (MAKEFLAGS=-k: one run reports every compile error)
ccache -z
(cd dxb && bash build.sh) || status=$?
ccache -s

# build tree for local single-file compiles, then the kernel
SDK=$(xcrun --sdk macosx --show-sdk-path)
mkdir -p sdk-kernel-platform out
cp -R "$SDK/System/Library/Frameworks/Kernel.framework/Versions/A/PrivateHeaders/platform" sdk-kernel-platform/ || true
tar czf buildtree.tgz --exclude='*.o' --exclude='*.cpo' --exclude='*.a' --exclude='*.dSYM' --exclude='*.unstripped' \
	--exclude='kernel.development.*' --exclude='.git' -C dxb build/xnu.obj fakeroot/usr/local/include xnu -C .. sdk-kernel-platform || true
cp -R dxb/fakeroot/System/Library/Kernels/. out/ 2>/dev/null || true
(cd out && shasum -a 256 kernel.* > SHA256SUMS) || true
ls -la out
[ -f out/kernel.development.sweet ] || exit "${status:-1}"
