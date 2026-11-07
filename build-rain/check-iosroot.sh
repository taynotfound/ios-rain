#!/bin/bash
# rain: read-only look at the built iOS root: did sweet-root-tweaks.py apply?
# Run from the ios-topaz folder:  sudo bash build-rain/check-iosroot.sh
set -euo pipefail
cd "$(dirname "$0")/.."
img=research/ipsw/ios16-root.img
mnt=$(mktemp -d)
mount -t hfsplus -o loop,ro "$img" "$mnt"
trap 'umount "$mnt"; rmdir "$mnt"' EXIT
python3 -I - "$mnt" <<'EOF'
import plistlib, sys, os
root = sys.argv[1]
p = plistlib.load(open(os.path.join(root, 'System/Library/xpc/launchd.plist'), 'rb'))
jobs = p.get('LaunchDaemons', {})
print('launchd.plist top-level keys:', sorted(p)[:8])
print('LaunchDaemons jobs:', len(jobs))
print('  with _PanicOnCrash:', sum('_PanicOnCrash' in j for j in jobs.values()))
print('  with _PanicOnCrasX:', sum('_PanicOnCrasX' in j for j in jobs.values()))
print('  CommCenter jobs   :', [k for k, j in jobs.items() if j.get('Label') == 'com.apple.CommCenter'])
for name in ('com.apple.purplebuddy', 'com.apple.springboard'):
    f = os.path.join(root, 'private/var/mobile/Library/Preferences', name + '.plist')
    print(name, 'prefs:', sorted(plistlib.load(open(f, 'rb'))) if os.path.exists(f) else 'MISSING')
print('dyld cache files:', len(os.listdir(os.path.join(root, 'System/Library/Caches/com.apple.dyld'))))
print('launchd present:', os.path.exists(os.path.join(root, 'sbin/launchd')))
EOF
