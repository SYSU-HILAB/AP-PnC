#!/usr/bin/env bash
set -euo pipefail
: "${AP_PNC_DIR:?absolute project root required}"
[[ "$AP_PNC_DIR" = /* && -d "$AP_PNC_DIR/.git" ]] || exit 2
case "$(uname -m)" in
  aarch64 | arm64) arch=arm64; solver_arch=aarch64 ;;
  x86_64 | amd64) arch=amd64; solver_arch=x86_64 ;;
  *) exit 2 ;;
esac
# Existing pinned acados build-preparation image supplies CasADi, not the runtime.
builder="ap-pnc-nmpc-builder:2b28dc320a-$solver_arch"
tool="ap-pnc-simple-toolchain:$arch"
docker image inspect "$builder" "$tool" >/dev/null
mkdir -p "$AP_PNC_DIR/.artifacts/flatness"
docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint /bin/bash \
  -e AP_PNC_DIR=/ws -e PYTHONPATH=/ws \
  -v "$AP_PNC_DIR:/ws:ro" -v "$AP_PNC_DIR/.artifacts/flatness:/ws/.artifacts/flatness:rw" \
  "$builder" -c '
set -euo pipefail
python3 "$AP_PNC_DIR/core/ros_packages/planner/core/tailsitter_df/codegen/generate.py"
dir="$AP_PNC_DIR/.artifacts/flatness/linux-$(uname -m)"
cc -O2 -fPIC -shared "$dir/flatness_generated.c" "$dir/flatness_wrapper.c" -lm -o "$dir/libflatness.so"
cc -O2 -c "$dir/flatness_generated.c" -o "$dir/flatness_generated.o"
cc -O2 -c "$dir/flatness_wrapper.c" -o "$dir/flatness_wrapper.o"
ar rcs "$dir/libflatness.a" "$dir/flatness_generated.o" "$dir/flatness_wrapper.o"
nm -u "$dir/libflatness.so" > "$dir/runtime-symbols.txt"
'
docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint /bin/bash \
  -e AP_PNC_DIR=/workspace -v "$AP_PNC_DIR:/workspace:ro" \
  -v "$AP_PNC_DIR/.artifacts/flatness:/workspace/.artifacts/flatness:rw" "$tool" -c '
set -euo pipefail
dir="$AP_PNC_DIR/core/ros_packages/planner/core/tailsitter_df"
c++ -O2 -std=c++17 -I/usr/include/eigen3 -I"$dir/include" "$dir/codegen/golden.cpp" \
  "$dir/src/tailsitter_df.cpp" -o "$AP_PNC_DIR/.artifacts/flatness/linux-$(uname -m)/flatness_golden"
c++ -O2 -std=c++17 -I/usr/include/eigen3 -I"$dir/include" "$dir/codegen/flu_golden.cpp" \
  -o "$AP_PNC_DIR/.artifacts/flatness/linux-$(uname -m)/flu_golden"
'
docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint /bin/bash \
  -e AP_PNC_DIR=/ws -e PYTHONPATH=/ws -v "$AP_PNC_DIR:/ws:ro" \
  -v "$AP_PNC_DIR/.artifacts/flatness:/ws/.artifacts/flatness:rw" "$builder" \
  -c 'python3 "$AP_PNC_DIR/core/ros_packages/planner/core/tailsitter_df/codegen/validate.py"; python3 "$AP_PNC_DIR/core/ros_packages/planner/core/tailsitter_df/codegen/validate_flu.py"'
