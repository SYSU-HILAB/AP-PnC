# Offline simulation and independently verified Rerun replay

The pipeline executes sequentially: C++ simulation → Rust artifact export →
independent Rerun CLI verification. Only the simulator plans and advances the
plant. The exporter reads committed CSV and the saved reference polynomial;
it never reruns a planner or plant. No ROS graph, PX4 or Gazebo is involved.

Interactive perception development belongs to the separate
[ROS2 pillar/LiDAR simulator](DEV_SIM.md), not this benchmark path.

## Build and run

Use an absolute `AP_PNC_DIR`. On Apple Silicon all numerical builds execute in
native Linux ARM64 Docker; no silent AMD64 emulation. The matching solver bundle
is statically linked, so regenerate **and rebuild** after model/ABI changes.

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
uv run --project "$AP_PNC_DIR" ap-pnc benchmark --profile ideal --aero lyu --vmax 8
```

Rust 1.96.0 and Rerun 0.38.1 are pinned. CLI downloads are SHA256 checked and
Cargo dependencies are locked. Runtime containers use host IPC with networking
 disabled. Job configuration/output/binaries are mounted individually; never
mount the complete host artifacts tree over an image's installed binaries.

`tooling.simple_stack.run_job(JobRequest(...))` owns staging, configuration,
execution and receipt validation. It invokes `ap-pnc-offline run CONFIG`, which
stops on the first failed stage. The default CLI benchmark compares controller
variants and saves original simulation data under
`.artifacts/simple-stack/linux-<arch>/runs/<job>/benchmark/simple_sim_<id>/`.

The optional `practical` profile requires local authorized numeric extraction:

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" import-matlab
uv run --project "$AP_PNC_DIR" ap-pnc benchmark --profile practical --aero lyu --vmax 8
```

Profiles/provenance stay under `.artifacts/vtol-matlab/`. Missing practical data
fails explicitly; there is no fallback to ideal actuation. Raw private MATLAB
source is never uploaded or committed.

## Replay, negative checks and terminal policy

Use the printed absolute JOB and actual simulator directory NAME:

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" export "$JOB" "$NAME"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" verify "$JOB" "$NAME"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test "$JOB"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" view "$JOB" "$NAME"
```

The completed-file viewer binds host loopback ports 9090 and 9876. It is not a
live simulator. Original CSV/YAML/manifest/metrics remain authoritative.
`replay.json` binds source and recording hashes; `verified.json` is written only
after the independent decoder accepts the RRD. Failed retries invalidate stale
success markers. Export rejects unfinished, empty, truncated, discontinuous,
nonfinite and inconsistent scientific data. Acceptance uses copied fixtures and
disables the simulator executable to test independent export.

World coordinates are ENU, body coordinates FLU. Recorded final states use
`next_time_ns`. Reference samples beyond the trajectory repeat the complete
last point when `terminal_policy: hold_last_point` is serialized. Older data
without that policy keeps strict coverage checks. Optional terminal hold is
part of the requested window; partial RMSE is never full-run acceptance.

## Model and validation boundary

The current NMPC is the ABI3 command integrator, with 15 states and 4 command
 derivatives. It integrates the optimized derivative over the 20ms control
period, not the 100ms prediction knot. Its OCP has no motor/mixer/inertia model;
the practical plant separately contains four motors, mixer, PID, wing aero and
rigid-body RK4. Independent export checks committed command integration.

Planner dynamic limits are soft penalties, so report observed peaks rather than
claiming strict feasibility. Successful recording proves scientific consistency,
not good tracking, collision avoidance, or hardware/flight acceptance.
Historical speed/tracking campaigns and their failures remain in `docs/` and
saved artifact directories; this lifecycle change does not retest them.

Build-time flatness kernels remain separate:

```bash
bash "$AP_PNC_DIR/core/ros_packages/planner/src/tailsitter_df/codegen/build.sh"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test-ffi
```

They are not a runtime replacement for the active planner. Python CLI telemetry
is opt-in; native solver/RK4 internal span coverage is not claimed. No exporters
or force-flush calls are added to real-time numerical paths.
