# simple_sim offline stack (first Rust-first acceptance stage)

`dora simulate -> record -> verify`, all in native Linux Docker. The Rust
source node invokes the existing ROS-free C++ experiment executable and its
real, statically linked acados NMPC bundle. It does **not** run ROS, PX4 or
Gazebo. The other Rust nodes only consume completed experiment artifacts.
This is not yet a Rust rewrite of the simulator/planner/controller.

## Entry point

Export an absolute `AP_PNC_DIR`. The script chooses the host's native CPU
architecture; it does not silently emulate another target. A matching
`simple-sim:<arm64|amd64>` baseline image must already exist. To prepare it:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc sim build --target simple
```

Regenerate the matching NMPC bundle after prediction-model/ABI changes, then
use the Docker-only stack entry point (no host Rust/ROS/acados install):

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
```

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" build
uv run --project "$AP_PNC_DIR" ap-pnc benchmark stack --aero lyu --vmax 8
# Omit duration (or use 0) for the complete reference-supported NMPC horizon.
```

Rust **1.96.0**, dora **1.0.1**, Rerun **0.38.1** are pinned. Rerun 0.38.1
requires Rust 1.96, not the 1.95 minimum of dora. Container release downloads
are SHA256-checked; crate dependencies are frozen in `Cargo.lock`.

Each run prints an absolute `JOB`. Its `benchmark/simple_sim_<id>_<suffix>`
subdirectory is the simulator's original output. Caches are isolated under
`<root>/.artifacts/simple-stack/linux-<arch>/`; jobs have distinct directories.
Only the benchmark/config/job/binary are mounted into the runtime image, never
the host's entire artifacts tree over the C++ install tree. All containers use
host IPC. The experiment graph runs with Docker networking disabled by default.

## Native dora OpenTelemetry (opt-in)

The node explicitly enables dora's `tracing`/`metrics` features and public tracing
lifecycle. No second Rust subscriber/provider or new message envelope is installed.
The original path payload is unchanged; reserved dora metadata carries W3C context.

```bash
uv run --project "$AP_PNC_DIR" --extra telemetry python -c "from tooling import observability as o; o.collector(['up', '-d'])"
uv run --project "$AP_PNC_DIR" ap-pnc benchmark stack --aero lyu --vmax 8  # telemetry: put a Collector on OTEL_EXPORTER_OTLP_ENDPOINT
# Compare actual Collector spans AND scientific data with a telemetry-off job:
uv run --project "$AP_PNC_DIR" python "$AP_PNC_DIR/infra/observability/verify_dora.py" "$ON_JOB" --baseline "$OFF_JOB"
```

Opt-in runtime uses only the local Collector network/gRPC endpoint. Root-level
`otel-*.json` sidecars map actors/artifacts to span IDs, without changing scientific
files. Real spans cover simulate/export/verify, C++ process and decoder boundaries.
Logged solver status/tick/reference digest and solve-duration histograms are derived
from actual scientific output; this is **not** C++ solver/RK4 function instrumentation.
Default remains off; SDK flush/shutdown never runs within a control period.

Native ARM64 full off/on/outage comparisons preserve 874 ticks × 74 scientific
columns and actual solver-reference CSV (only solve wall time excluded). A real
status4 failure produces ERROR spans and no successful replay marker. See the full
the project observability contract for scope/gates.

## Terminal reference and speed sweeps

Reference samples beyond the planned end repeat the complete last point, matching
ROS ReferenceBuffer. No polynomial extrapolation; `sample_time_ns` is clamped,
while `reference_time_ns` retains the requested prediction time. The serialized
`terminal_policy: hold_last_point` enables the decoder rule explicitly; old
artifacts without it retain strict coverage checks.

Automatic duration now includes the whole trajectory, rounded up to a complete
control period. Optional `terminal_hold_s` adds dwell; explicit-duration short
probes retain their complete-step floor. Example (full trajectory + 2 s dwell):

```bash
uv run --project "$AP_PNC_DIR" ap-pnc benchmark stack --aero lyu --vmax 8
uv run --project "$AP_PNC_DIR" python "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" run \
  --v-max 8 --hold-seconds 2 --output "$AP_PNC_DIR/.artifacts/benchmark/NEW_V8"
```

New matched sweep completion: 6 m/s 5/5, 8 m/s 4/5, 10/12 m/s 0/5. See the
speed/hold report. Historical
17.48 s v6 figures below exclude the last prediction tail and are not the new
full-trajectory+dwell window. Do not compare failed partial RMSE with full runs.

## Independent export, verification, replay

Substitute the printed absolute JOB and the actual simulator directory NAME:

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" export "$JOB" "$NAME"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" verify "$JOB" "$NAME"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test "$JOB"
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" view "$JOB" "$NAME"
```

The viewer is only started **after** completion. Open `http://127.0.0.1:9090`
and use the `sim_time` timeline. Ports are host-loopback-only; 9876 serves this
completed file to the viewer, not live simulator data. Ctrl-C stops the viewer.
It is not necessary to leave the viewer running to preserve the experiment.

Original CSV/YAML/metrics/manifest remain authoritative. `replay.rrd` contains
trajectory/reference, pose and body axes, velocity, tracking error, state,
command, IMU specific force, aero wrench, solver status and solve duration.
World is ENU, body is FLU; quaternion input is wxyz and SDK input is xyzw.
Pre-step data uses `time_ns`; the final next-state uses `next_time_ns`.
The final reference is evaluated from the **saved** polynomial coefficients,
not from a new planner or plant run. Complete RMSE/max/solver counts are checked.

The exporter rejects unfinished/failed/interrupted, empty, truncated,
discontinuous, nonfinite, nonunit-quaternion or inconsistent runs. Source
hashes are checked before/after conversion. Partial recordings have no success
receipt. `replay.json` records source/recording hashes; `verified.json` is only
written after the independent Rerun CLI decodes the file and verifies footers.
Failed retries invalidate old markers. The acceptance script operates on copied
fixtures and disables the simulator executable to prove independent export.
It also checks a corrupt RRD even when its outer SHA256 is updated to match.

## Current ABI3 speed campaign: four motors, 2 kg

Run only none/lyu/ma, with an explicit mass override after the private profile
is loaded. Original profile/provenance remains untouched; inertia, motor calibration,
PID, solver weights/bounds and timing are unchanged. Full reference plus 2 s hold:

```bash
uv run --project "$AP_PNC_DIR" python "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" run \
  --profile practical --v-max 8 --hold-seconds 2 --mass 2 --output ABSOLUTE_NEW_ARTIFACT_DIRECTORY
uv run --project "$AP_PNC_DIR" python "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/plot_aero_history.py" \
  --output ABSOLUTE_SAVED_GROUP_DIRECTORY
```

The second command only plots saved CSV. Model-returned wing-FRD alpha/beta and
FLU aerodynamic force/moment are recorded at before/after states; curves include
the final committed state. Low-speed angle guards remain model-owned.

Actual 6/8/10/12 results:
11 status4 failures; ma/6 completes but position RMSE is 18.98 m and max error 69.36 m.
Completion/verified RRD is scientific-record validation, **not tracking acceptance**.
Do not test phi/advanced in this campaign.

## Command-integrator NMPC prediction and separate physical plant

The OCP contains **no rotor/PWM/mixer/inertia/torque allocation model**.
Its 15 states are `[p(3), v(3), rate_command(3), q_wxyz(4), thrust_command, k_aero]`.
The four inputs are derivatives of `[collective specific thrust, body rates]`:

```
d(thrust_command)/dt = u[0]       # N/kg/s
d(rate_command)/dt   = u[1:4]     # rad/s^2
```

This assumes ideal inner-loop tracking, NOT first-order actuator response.
The objective is only `[p, v, body_y]` tracking plus zero-reference command
**derivative** regularization, including stage 0. Absolute command bounds are
state constraints at future and terminal nodes. YAML owns horizon, objective and command-bound tuning;
`config.py`, lag constants, zero-weight rate targets and hover cost targets are removed.
The existing lumped body-X coefficient `cx` (IMU-calibrated) is retained; it is
also the wing-normal axis because `+X_B = +Z_L`. Its d(alpha)/dt term now
includes the missing `-omega x v` rotating-frame contribution.

The controller integrates the optimized derivative over the actual **20 ms**
control period: `c_next = c_previous + dt_control * u0`. It publishes neither
`u0` itself nor `x1` at the 100 ms prediction knot. Command memory is not
replaced by measured motor/inner-loop response; reset can supply a known initial
command. Solver ABI **3** is **15/4/1** (x/u/p), with **13/9** stage/terminal
outputs. The ABI symbol and generated-header static assertions reject old archives
at link or startup. The CSV persists actual u0 derivatives, integrated commands
and acceptance, and the independent decoder checks every committed integration. **Rebuilding the bundle alone does not update a statically linked
simulator/flight binary.** `stack.sh build` now also rebuilds/tests/installs C++
and mounts that executable explicitly into the offline graph.

The optional physical plant retains four motors, mixer, inner PID, propeller
inflow/reaction/gyro, wing aero and rigid-body RK4. There are **no control
surfaces**. Authorized private MATLAB numeric data is extracted only locally:

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" import-matlab
uv run --project "$AP_PNC_DIR" ap-pnc benchmark stack --aero lyu --vmax 8
# Default `run` is the ideal-rate-loop plant, available as the control baseline.
```

The importer expects the authorized checkout under
`<root>/.artifacts/upstream/VTOL-SIM-MATLAB/`. It writes numeric parameters and
provenance under `.artifacts/vtol-matlab/`; no raw source is uploaded or pushed.
It uses the actual loaded MAT mass/inertia and transforms wing to FLU with a
proper rotation, synchronizing planner/plant/controller runtime mass. The
mass-normalized command-integrator OCP has **no fixed mass/inertia/rotor geometry** to
retune. RPM is a continuous RK4 state, unlike upstream's 4 ms Euler state; this
is not a claim of whole-simulator tick equivalence. MATLAB/Octave execution
golden remains unverified. Scientific CSV now includes RPM/next-RPM, held
RPM targets/duty and applied wrench, with offline Rerun actuator channels.

**Historical ABI2** ARM64 acceptance used the direct FLU public reference: all five matched
practical plant aero models (none/lyu/phi/ma/advanced) finish 17.48 s / 874 ticks
at v_max setting 6 and pass independent replay verification. This is a soft speed
constraint: reference peak is about 6.00201 m/s, not strictly ≤6.0000. Actual NMPC
references are audited separately from planned rates/thrust; see
the tracking evaluation notes. Native dora OTel
off/on/outage also preserve all 74 non-wall-time scientific columns on the full
Lyu run. No hardware/flight acceptance is implied.

**Historical pre-FLU-reference failures are preserved:** ideal Lyu failed at
5.26 s / 263 ticks and motor Lyu at 4.68 s / 234 ticks with status4 and no replay
success markers. High actual pitch rate was observed, but its mechanism was not
independently resolved. Historical v6 success does not explain every historical
failure or establish high-speed robustness; a separate v12 case still fails.

## Build-time CasADi C kernel

This separate prototype generates the four `tailsitter_df` optimization
outputs, a dense column-major 4x9 Jacobian, and `J^T seed`. A separate production
VJP entry computes the backward gradient **without constructing the Jacobian**.
Python/CasADi is only
used inside the existing pinned NMPC builder image; the generated runtime
library depends on C/libm, with per-call stack workspace. It does not touch
acados codegen, the NMPC bundle, or the active planner implementation.

```bash
bash "$AP_PNC_DIR/core/ros_packages/planner/core/tailsitter_df/codegen/build.sh"
```

Outputs: `<root>/.artifacts/flatness/linux-<aarch64|x86_64>/`. This workflow needs
`ap-pnc-nmpc-builder:2b28dc320a-<solver_arch>` and the stack toolchain image.
`validation.json` reports C++/autodiff golden, finite-difference, VJP and parallel
checks. The first output is **tangent acceleration**, not thrust magnitude;
legacy C++ variable names are misleading. Low-speed/projected-acceleration
policies are branchwise; no smooth derivative is claimed at switching surfaces.

An optional Rust safe wrapper is in `src/flatness.rs` (`flatness-ffi` feature).
After codegen, run `bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test-ffi`
for the six safe-wrapper tests and strict clippy, in the toolchain container.
The wrapper validates finite inputs/outputs, fixed buffer dimensions and layout;
it retains no foreign pointers. It is **not** yet wired into planner Data's
attitude/force side effects or trajectory optimization.

## Roadmap / acceptance boundaries

1. Freeze C++ numerical baseline and migrate Runner/RK4/Plant/rate loop, then
   controllers, reference generation and planner module by module into Rust.
2. Integrate the validated generated derivatives with planner backward and all
   required forward outputs/branch/frame contracts. Keep C++ golden tests.
3. Expand Rerun exports to the remaining aerodynamics/ML/sweep experiments;
   decide separately whether exact PDF/PGF publication layout can be retired.
4. Rust FAST-LIO adapter, real-data golden/time sync/latency/license checks.
5. Rust MAVLink gateway and flight safety state machine, then fault injection
   and native-Linux flight/hardware acceptance.

## Frozen legacy-model native ARM64 results (2026-10-05)

These results predate the current command-integrator model; they are not its acceptance.

- dora graph: 1 s / 50 ticks and full supported horizon / 568 ticks / 11.36 s.
- Independent Rerun decoder/footer validation and actual browser replay passed;
  screenshots include 3D trajectories and plots on the `sim_time` timeline.
- 13 negative/independent-export checks passed. A real mass-mismatch startup
  failure made the graph exit 1 and produced no recording/success receipt.
- Direct C++ baseline: all 52 scientific CSV columns match every tick exactly;
  only wall-clock `solve_time_ms` is excluded. Metrics are identical.
- **Tracking performance is NOT accepted:** position RMSE 7.1203 m, maximum
  18.7164 m on this configuration. Zero solver-status-2 events do not establish
  good tracking. This deviation is already present in the direct C++ baseline.
- Generated C kernel: 511 C++ golden cases, 100 finite-difference cases, 2044
  parallel calls; Jacobian max absolute error 1.315e-13. Five Rust FFI tests,
  two Rust node utility tests, strict clippy and 50 CLI tests passed.

Evidence is under `<root>/.artifacts/feasibility/simple-stack/`, full run under
`<root>/.artifacts/simple-stack/linux-arm64/runs/20261005T035929Z-5400/`.
The baseline image's manifest has an empty build revision; image IDs and bundle
SHA256, not a guessed source revision, establish its identity. New jobs also
record checkout revision/dirty status and Rust node binary hash.

Native ARM64 execution does not imply AMD64 acceptance. A successful file
replay is not PX4 flight/LIO/hardware validation. The existing Mac PX4 rebuild
deferral stays in effect. No unrelated Docker resources are cleaned up here.
