#!/usr/bin/env bash
# NMPC bundle builder entrypoint — runs INSIDE the container with the repo
# mounted at ${AP_PNC_DIR:-/ws}. Produces:
#   $AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_<arch>.a
# and, when NMPC_BUNDLE_ALIAS=1 (target arch == host native arch), refreshes
# the unsuffixed libnmpc_bundle.a alias.
set -euo pipefail

root="${AP_PNC_DIR:-/ws}"
arch="$(uname -m)"
case "$arch" in
  arm64) arch=aarch64 ;;
  amd64) arch=x86_64 ;;
esac
case "$arch" in
  aarch64 | x86_64) ;;
  *) echo "unsupported target arch: $arch" >&2; exit 2 ;;
esac

export AP_PNC_DIR="$root"
export ACADOS_SOURCE_DIR=/opt/acados
export ACADOS_PYTHON_INTERFACE_PATH=/opt/acados_src/interfaces/acados_template
export NMPC_ACADOS_STAGING=/opt/acados
export NMPC_ACADOS_STATIC=/opt/acados_static
export LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# 1) OCP codegen (rendered C + json land under .artifacts/c_generated_code)
gen_dir="$root/.artifacts/c_generated_code"
mkdir -p "$gen_dir"
( cd "$gen_dir" && PYTHONPATH="$root" python3 "$root/tooling/nmpc_gen/create_ocp.py" )

# 2) Wrapper .a + merge with static acados (repo logic; reads NMPC_ACADOS_*)
PYTHONPATH="$root" python3 -c "\
from pathlib import Path
from tooling.commands.gen_nmpc_lib import _build_wrapper_and_bundle
_build_wrapper_and_bundle(Path('$root'))"

# 3) Arch-tagged output; keep the unsuffixed alias == HOST-native arch.
#    The merge above always writes libnmpc_bundle.a (target-arch content),
#    so a cross build must restore the alias afterwards.
solver_dir="$root/.artifacts/nmpc_solver"
bundle="$solver_dir/libnmpc_bundle.a"
target="$solver_dir/libnmpc_bundle_${arch}.a"
cp -f "$bundle" "$target"
echo "bundle: $target ($(stat -c%s "$target" 2>/dev/null || stat -f%z "$target") bytes, ${arch})"
host_arch="${NMPC_HOST_ARCH:-}"
if [[ "${NMPC_BUNDLE_ALIAS:-0}" == "1" ]]; then
  echo "alias: $bundle refreshed (= ${arch})"
elif [[ -n "$host_arch" && -f "$solver_dir/libnmpc_bundle_${host_arch}.a" ]]; then
  cp -f "$solver_dir/libnmpc_bundle_${host_arch}.a" "$bundle"
  echo "alias: $bundle restored to host arch ${host_arch}"
else
  echo "WARNING: $bundle currently holds ${arch} content; build the \
${host_arch:-host-native} bundle to restore the host-native alias" >&2
fi
