#!/bin/bash
# Build the GPU user space for the iOS root on a Mac (Xcode): libtopaz-gl.dylib (eagl.c behind
# EAGLContext, on Mesa's libOSMesa), libtopaz-inject.dylib (loaded into backboardd) and the
# backboardd wrapper. Recipe after HakiOS userland/build.sh (GL=1); upstream ios-topaz kept its own private.
#   userland/gpu/build.sh MESA_OUT OUT      MESA_OUT: libOSMesa.8.dylib, gl.xml, exported-gl.txt
set -e
G=$(cd "$(dirname "$0")" && pwd)
M=$(cd "$1" && pwd)
mkdir -p "$2"
O=$(cd "$2" && pwd)
SDK=$(xcrun --sdk iphoneos --show-sdk-path)
MACSDK=$(xcrun --sdk macosx --show-sdk-path)
# the iOS SDK links IOKit but ships no headers for it: borrow macOS's
mkdir -p "$O/inc"
ln -sfn "$MACSDK/System/Library/Frameworks/IOKit.framework/Headers" "$O/inc/IOKit"
CC="$(xcrun --sdk iphoneos -f clang) -target arm64-apple-ios16.0 -isysroot $SDK -I$O/inc -O1"
python3 "$G/mkeagl.py" "$G/eagl_slots.txt" "$M/gl.xml" "$M/exported-gl.txt" "$G/eagl.c" > "$O/eagl_gen.h"
$CC -dynamiclib -install_name /topaz/lib/libtopaz-gl.dylib -I"$O" -I"$G" -framework CoreFoundation \
	-framework IOKit -framework IOSurface -lobjc "$M/libOSMesa.8.dylib" -o "$O/libtopaz-gl.dylib" "$G/eagl.c" "$G/gl_osmesa.c"
$CC -dynamiclib -install_name /topaz/lib/libtopaz-inject.dylib -framework CoreFoundation -framework IOKit -lobjc \
	-o "$O/libtopaz-inject.dylib" "$G/topaz_inject.c"
$CC -o "$O/backboardd" "$G/bbd_wrap.c"
# CoreAudio HAL plug-in (the "Speaker" device writing to the kernel's /dev/redwood-audio)
$CC -bundle -framework CoreFoundation -I"$G/../audio" -o "$O/RedwoodAudio" "$G/../audio/audio_plugin.c"
cp "$G/../audio/audio_plugin.plist" "$O/RedwoodAudio.Info.plist"
cp "$M/libOSMesa.8.dylib" "$O/"
# Ad-hoc signatures: the kernel's AMFI stand-in makes every *signed* binary on the root a platform
# binary, and launchd panics when a system job execs one that is not ("unexpected exec of non-platform binary").
codesign -f -s - "$O/backboardd" "$O/RedwoodAudio" "$O"/*.dylib
rm -rf "$O/inc"
ls -l "$O"
