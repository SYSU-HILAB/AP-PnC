#!/usr/bin/env bash
# Validate every ELF member of an archive before installing the container-local alias.
set -euo pipefail
case "${AP_PNC_DIR:?Set absolute AP_PNC_DIR}" in
    /*) ;;
    *) echo 'AP_PNC_DIR must be absolute' >&2; exit 2 ;;
esac
actual="$(dpkg --print-architecture)"
if [[ -n "${TARGETARCH:-}" && "$actual" != "$TARGETARCH" ]]; then
    echo "Base architecture $actual does not match build target $TARGETARCH" >&2
    exit 2
fi
case "$actual" in
    arm64) arch=aarch64; machine=AArch64 ;;
    amd64) arch=x86_64; machine='Advanced Micro Devices X86-64' ;;
    *) echo 'Unsupported container architecture' >&2; exit 2 ;;
esac
bundle="$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_${arch}.a"
if [[ ! -f "$bundle" ]]; then
    echo "Missing $bundle; run ap-pnc gen-nmpc-lib --arch $arch" >&2
    exit 2
fi
LC_ALL=C readelf -h "$bundle" | awk -F: -v expected="$machine" '
    /Machine:/ {
        sub(/^[ \t]+/, "", $2); sub(/[ \t]+$/, "", $2); count++
        if ($2 != expected) { print "Wrong ELF architecture: " $2 > "/dev/stderr"; bad=1 }
    }
    END { if (!count || bad) exit 1 }
'
cp "$bundle" "$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle.a"
echo "Selected and verified $bundle"
