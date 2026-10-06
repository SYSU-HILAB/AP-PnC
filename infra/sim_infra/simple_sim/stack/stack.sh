#!/usr/bin/env bash
# One native-Linux container workflow; no host ROS/Rust/acados installation.
set -euo pipefail
: "${AP_PNC_DIR:?export an absolute project root}"
[[ "$AP_PNC_DIR" = /* && -d "$AP_PNC_DIR/.git" ]] || { echo 'invalid AP_PNC_DIR' >&2; exit 2; }
case "$(uname -m)" in
  arm64 | aarch64) arch=arm64 ;;
  x86_64 | amd64) arch=amd64 ;;
  *) echo 'unsupported native architecture' >&2; exit 2 ;;
esac
base="$AP_PNC_DIR/.artifacts/simple-stack/linux-$arch"
source_dir="$AP_PNC_DIR/infra/sim_infra/simple_sim/stack"
mkdir -p "$base" "$AP_PNC_DIR/.artifacts/simple-stack/cache"
tool="ap-pnc-simple-toolchain:$arch"
image="ap-pnc-simple-stack:$arch"
command="${1:-run}"

build() {
  docker image inspect "simple-sim:$arch" >/dev/null
  docker build --platform "linux/$arch" --target toolchain -t "$tool" -f "$source_dir/Dockerfile" "$AP_PNC_DIR"
  docker run --rm --platform "linux/$arch" --ipc=host --entrypoint /bin/bash \
    -e AP_PNC_DIR=/workspace -e CARGO_HOME=/workspace/.artifacts/simple-stack/cache/cargo \
    -v "$AP_PNC_DIR:/workspace:ro" -v "$source_dir:/workspace/infra/sim_infra/simple_sim/stack:rw" \
    -v "$base:/workspace/.artifacts/simple-stack/cache:rw" "$tool" -c '
      set -euo pipefail
      manifest="$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/Cargo.toml"
      target="$AP_PNC_DIR/.artifacts/simple-stack/cache/target"
      if [[ ! -f "${manifest%/*}/Cargo.lock" ]]; then cargo generate-lockfile --manifest-path "$manifest"; fi
      cargo fmt --manifest-path "$manifest" -- --check
      cargo test --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target"
      cargo build --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target"
      cargo clippy --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target" -- -D warnings
    '
  docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint /bin/bash \
    -e AP_PNC_DIR=/workspace -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0=/workspace \
    -v "$AP_PNC_DIR:/workspace:ro" -v "$base:/workspace/.artifacts/simple-stack/cache:rw" "$tool" -c '
      set -euo pipefail
      dir="$AP_PNC_DIR/.artifacts/simple-stack/cache/cpp"
      solver_arch="$(uname -m)"
      cmake -S "$AP_PNC_DIR/infra/sim_infra/simple_sim" -B "$dir/build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dir/install" \
        -DSIMPLE_SIM_WITH_ROS=OFF -DSIMPLE_SIM_WITH_NMPC=ON -DBUILD_TESTING=ON \
        -DNMPC_BUNDLE_PATH="$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_${solver_arch}.a"
      cmake --build "$dir/build" --parallel 2
      ctest --test-dir "$dir/build" --output-on-failure -E acceptance
      cmake --install "$dir/build"
    '
  docker build --platform "linux/$arch" --target runtime -t "$image" -f "$source_dir/Dockerfile" "$AP_PNC_DIR"
}

validate_job() {
  [[ "$1" = /* && -d "$1" ]] || { echo 'absolute existing job required' >&2; exit 2; }
  local canonical
  canonical="$(cd "$1" && pwd -P)"
  [[ "$canonical" = "$base/runs/"* && -d "$canonical/benchmark" && -d "$canonical/config" && ! -L "$canonical/benchmark" && ! -L "$canonical/config" ]] || { echo 'invalid job directory' >&2; exit 2; }
}

# These mounts are deliberately NOT the entire .artifacts tree: keep the image's
# compiled C++ install, and isolate every job's simulator output.
container() {
  local job="$1"; shift
  validate_job "$job"
  local sim_bind=()
  if [[ "${STACK_TEST_EXPORT_ONLY:-0}" != 1 ]]; then
    sim_bind=(-v "$base/cpp/install/lib/simple_sim/simple_sim_run:/workspace/.artifacts/colcon/install/lib/simple_sim/simple_sim_run:ro")
  fi
  local telemetry_args=(--network none)
  if [[ "${AP_PNC_TELEMETRY:-0}" = 1 ]]; then
    local endpoint="${DORA_OTLP_ENDPOINT:-http://otel-collector:4317}"
    [[ "$endpoint" = http://otel-collector:4317 || "$endpoint" = http://otel-collector:9 ]] || { echo 'only local Collector supported' >&2; exit 2; }
    docker network inspect ap-pnc-observability_default >/dev/null
    telemetry_args=(--network ap-pnc-observability_default -e "DORA_OTLP_ENDPOINT=$endpoint"
      -e OTEL_EXPORTER_OTLP_TIMEOUT=1000 -e OTEL_BSP_SCHEDULE_DELAY=200
      -e OTEL_METRIC_EXPORT_INTERVAL=500 -e OTEL_METRIC_EXPORT_TIMEOUT=1000)
  fi
  docker run --rm --platform "linux/$arch" --ipc=host "${telemetry_args[@]}" \
    -e AP_PNC_DIR=/workspace -e RERUN_ANALYTICS_ENABLED=false \
    -e HOME=/workspace/.artifacts/simple-stack/job \
    --workdir /workspace/.artifacts/simple-stack/job \
    -v "$base/target/release/ap-pnc-offline:/workspace/.artifacts/bin/ap-pnc-offline:ro" \
    "${sim_bind[@]}" \
    -v "$job:/workspace/.artifacts/simple-stack/job:rw" \
    -v "$job/benchmark:/workspace/.artifacts/benchmark:rw" \
    -v "$job/config:/workspace/core/bringup/config:ro" "$image" "$@"
}

case "$command" in
  build) build ;;
  import-matlab)
    mkdir -p "$AP_PNC_DIR/.artifacts/vtol-matlab"
    solver_arch=aarch64; [[ "$arch" = amd64 ]] && solver_arch=x86_64
    docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint python3 \
      -e AP_PNC_DIR=/ws -e PYTHONPATH=/ws -v "$AP_PNC_DIR:/ws:ro" \
      -v "$AP_PNC_DIR/.artifacts/vtol-matlab:/ws/.artifacts/vtol-matlab:rw" \
      "ap-pnc-nmpc-builder:2b28dc320a-$solver_arch" \
      /ws/infra/sim_infra/simple_sim/stack/import_matlab.py
    ;;
  run)
    [[ -x "$base/target/release/ap-pnc-offline" ]] || { echo "run build first: $source_dir/stack.sh build" >&2; exit 2; }
    [[ -x "$base/cpp/install/lib/simple_sim/simple_sim_run" ]] || { echo 'rebuild the new C++/solver ABI first' >&2; exit 2; }
    duration="${2:-0.0}"
    profile="${3:-ideal}"
    aero="${4:-configured}"
    vmax="${5:-configured}"
    hold="${6:-0}"
    mass="${7:-configured}"
    # Only models marked under_test (plus the null control) are accepted; see
    # infra/sim_infra/aerodynamics/include/aerodynamics/aero_models.hpp.
    [[ "$aero" =~ ^(configured|none|lyu|phi)$ ]] || exit 2
    [[ "$profile" = ideal || "$profile" = practical ]] || exit 2
    job="$base/runs/$(date -u +%Y%m%dT%H%M%SZ)-$$"
    mkdir -p "$job/benchmark" "$job/config"
    cp "$AP_PNC_DIR/core/bringup/config/"{simple_sim,planning,nmpc}.yaml "$job/config/"
    if [[ "$profile" = practical ]]; then
      cp "$AP_PNC_DIR/.artifacts/vtol-matlab/profile.yaml" "$job/profile.yaml"
      cp "$AP_PNC_DIR/.artifacts/vtol-matlab/provenance.json" "$job/upstream-provenance.json"
    fi
    docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint python3 \
      -e AP_PNC_DIR=/workspace -v "$job:/workspace/.artifacts/simple-stack/job:rw" "$image" -c '
import math, os, pathlib, sys, yaml
root = pathlib.Path(os.environ["AP_PNC_DIR"])
p = root / ".artifacts/simple-stack/job/config/simple_sim.yaml"
n = float(sys.argv[1])
assert math.isfinite(n) and n >= 0
cfg = yaml.safe_load(p.read_text())
cfg["simple_sim"]["duration_s"] = n
hold = float(sys.argv[5])
assert math.isfinite(hold) and hold >= 0
cfg["simple_sim"]["terminal_hold_s"] = hold
if sys.argv[2] == "practical":
    profile = yaml.safe_load((root / ".artifacts/simple-stack/job/profile.yaml").read_text())
    cfg["simple_sim"]["plant"].update(profile["plant"])
    cfg["simple_sim"]["actuator"] = profile["actuator"]
    # Only the planner owns a mass: the NMPC model is mass-normalized.
    owner = yaml.safe_load((p.parent / "planning.yaml").read_text())
    owner["problem_formulation"]["flatness"]["mass"] = profile["plant"]["mass"]
    (p.parent / "planning.yaml").write_text(yaml.safe_dump(owner, sort_keys=False))
if sys.argv[6] != "configured":
    mass = float(sys.argv[6])
    assert math.isfinite(mass) and mass > 0
    original_mass = cfg["simple_sim"]["plant"]["mass"]
    cfg["simple_sim"]["plant"]["mass"] = mass
    owner = yaml.safe_load((p.parent / "planning.yaml").read_text())
    owner["problem_formulation"]["flatness"]["mass"] = mass
    (p.parent / "planning.yaml").write_text(yaml.safe_dump(owner, sort_keys=False))
    import json
    (p.parent.parent / "experiment-overrides.json").write_text(json.dumps({
        "mass_kg": mass, "original_profile_mass_kg": original_mass,
        "unchanged": "inertia, motor calibration, PID, weights, bounds and timing"}, indent=2))
if sys.argv[3] != "configured":
    cfg["simple_sim"]["aero"]["model"] = sys.argv[3]
if sys.argv[4] != "configured":
    vmax = float(sys.argv[4])
    assert math.isfinite(vmax) and vmax > 0
    other = p.parent / "planning.yaml"
    planning = yaml.safe_load(other.read_text())
    planning["problem_formulation"]["cost"]["v_max"] = vmax
    other.write_text(yaml.safe_dump(planning, sort_keys=False))
p.write_text(yaml.safe_dump(cfg, sort_keys=False))
' "$duration" "$profile" "$aero" "$vmax" "$hold" "$mass"
    cp "$source_dir/dataflow.yml" "$job/dataflow.yml"
    docker image inspect "$image" "simple-sim:$arch" --format '{{json .}}' > "$job/images.jsonl"
    cp "$source_dir/Cargo.lock" "$job/Cargo.lock"
    solver_arch=aarch64; [[ "$arch" = amd64 ]] && solver_arch=x86_64
    cp "$AP_PNC_DIR/.artifacts/nmpc_build/$solver_arch/c_generated_code/acados_ocp_tailsitter_flu.json" "$job/generated-ocp.json"
    git -C "$AP_PNC_DIR" rev-parse HEAD > "$job/source-revision.txt"
    git -C "$AP_PNC_DIR" status --porcelain > "$job/source-status.txt"
    shasum -a 256 "$base/target/release/ap-pnc-offline" > "$job/node-binary.sha256"
    shasum -a 256 "$base/cpp/install/lib/simple_sim/simple_sim_run" > "$job/simulator-binary.sha256"
    echo "JOB=$job"
    container "$job" dora run /workspace/.artifacts/simple-stack/job/dataflow.yml 2>&1 | tee "$job/stack.log"
    # Graph exit alone is insufficient; require the decoder-verified receipt.
    [[ $(find "$job/benchmark" -name verified.json | wc -l) -eq 1 ]] || { echo 'missing verification receipt' >&2; exit 1; }
    echo "JOB=$job"
    ;;
  export | verify | check-run)
    job="${2:?absolute job directory}"; run="${3:?simulator run directory name}"
    [[ "$run" =~ ^simple_sim_[0-9]+_[0-9]+$ ]] || exit 2
    role=export; path="/workspace/.artifacts/benchmark/$run"
    if [[ "$command" = verify ]]; then role=check-replay; path="$path/replay.json"; fi
    if [[ "$command" = check-run ]]; then role=check-run; fi
    container "$job" /workspace/.artifacts/bin/ap-pnc-offline "$role" "$path"
    ;;
  test-ffi)
    docker run --rm --platform "linux/$arch" --ipc=host --network none --entrypoint /bin/bash \
      -e AP_PNC_DIR=/workspace -e CARGO_HOME=/workspace/.artifacts/simple-stack/cache/cargo \
      -v "$AP_PNC_DIR:/workspace:ro" -v "$base:/workspace/.artifacts/simple-stack/cache:rw" "$tool" -c '
        set -euo pipefail
        manifest="$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/Cargo.toml"
        target="$AP_PNC_DIR/.artifacts/simple-stack/cache/target"
        cargo test --features flatness-ffi --release --locked --offline --jobs 2 --manifest-path "$manifest" --target-dir "$target"
        cargo clippy --features flatness-ffi --release --locked --offline --jobs 2 --manifest-path "$manifest" --target-dir "$target" -- -D warnings
      '
    ;;
  test)
    job="${2:?absolute completed job directory}"
    validate_job "$job"
    # Source is copied into the job so the image stays independent of host cwd.
    cp "$source_dir/acceptance.py" "$job/acceptance.py"
    STACK_TEST_EXPORT_ONLY=1 container "$job" python3 /workspace/.artifacts/simple-stack/job/acceptance.py
    ;;
  view)
    job="${2:?absolute job directory}"; run="${3:?simulator run directory name}"
    validate_job "$job"
    [[ "$run" =~ ^simple_sim_[0-9]+_[0-9]+$ && -f "$job/benchmark/$run/verified.json" ]] || exit 2
    # Replay a completed file only. No SDK connection to the simulator exists.
    docker run --rm --platform "linux/$arch" --ipc=host \
      -e RERUN_ANALYTICS_ENABLED=false -e AP_PNC_DIR=/workspace \
      -e HOME=/workspace/.artifacts/simple-stack/job \
      -p 127.0.0.1:9090:9090 -p 127.0.0.1:9876:9876 \
      -v "$job:/workspace/.artifacts/simple-stack/job:rw" \
      -v "$job/benchmark:/workspace/.artifacts/benchmark:ro" "$image" \
      rerun --serve-web --bind 0.0.0.0 --web-viewer-port 9090 --port 9876 \
      "/workspace/.artifacts/benchmark/$run/replay.rrd"
    ;;
  *) echo 'build | import-matlab | run [duration_s] [ideal|practical] [aero|configured] [vmax|configured] [terminal_hold_s] [mass_kg|configured] | test-ffi | test JOB | export/verify/check-run/view JOB RUN_NAME' >&2; exit 2 ;;
esac
