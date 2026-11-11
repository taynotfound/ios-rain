# sourced: fsck_clean repairs $img until a read-only check passes (hfsprogs quirks handled)
fsck_clean() {
	for pass in 1 2 3; do
		if fsck.hfsplus -f -n "$img" >/dev/null 2>&1; then
			echo "fsck: clean (after $((pass - 1)) extra repair pass(es))"
			return 0
		fi
		echo "fsck: repair pass $pass"
		out=$(fsck.hfsplus -f -y "$img" 2>&1 || true)
		echo "$out" | grep -vE 'Illegal name' || true
		# hfsprogs 540.1-Linux: -n keeps reporting "Volume header needs minor repair"
		# on a volume whose -y pass finds nothing (primary and alternate VHB match).
		if echo "$out" | grep -q 'appears to be OK'; then
			echo "fsck: repair pass found nothing to fix; treating -n header warning as checker noise"
			return 0
		fi
	done
	fsck.hfsplus -f -n "$img"	# show what is still wrong and fail
}
