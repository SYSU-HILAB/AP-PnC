#!/usr/bin/env bash
# Runs only inside the pinned Linux builder; emits one architecture-tagged archive.
set -euo pipefail

root="${AP_PNC_DIR:-/ws}"
case "$root" in
  /*) ;;
  *) echo 'AP_PNC_DIR must be absolute' >&2; exit 2 ;;
esac
arch="$(uname -m)"
case "$arch" in
  aarch64 | x86_64) ;;
  *) echo "unsupported target arch: $arch" >&2; exit 2 ;;
esac

export AP_PNC_DIR="$root"
export ACADOS_SOURCE_DIR=/opt/acados
export ACADOS_PYTHON_INTERFACE_PATH=/opt/acados_src/interfaces/acados_template
export LD_LIBRARY_PATH="/opt/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# OCP codegen and compilation both use the image's toolchain, never the host's.
gen_dir="$root/.artifacts/c_generated_code"
mkdir -p "$gen_dir"
( cd "$gen_dir" && PYTHONPATH="$root" python3 "$root/tooling/nmpc_gen/create_ocp.py" )

PYTHONPATH="$root" python3 -c "\
import os
from pathlib import Path
from tooling.commands.gen_nmpc_lib import _build_wrapper_and_bundle
_build_wrapper_and_bundle(
    Path(os.environ['AP_PNC_DIR']),
    arch='$arch',
    staging_root=Path('/opt/acados'),
    static_dir=Path('/opt/acados_static'),
)"
