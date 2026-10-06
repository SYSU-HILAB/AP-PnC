# Simple sim — lockstep trajectory tracking

A lightweight, ROS-free C++ simulation kernel with optional ROS observation.
**No collision, ground contact, LiDAR, scene generation, PX4 or MAVROS.**
It verifies trajectory tracking, not the SITL/offboard flight procedure.

## Offline dora / Rerun stack

The new [Docker-only offline stack](stack/README.md) runs the existing real-NMPC
experiment through Rust dora nodes, then exports and independently verifies a
Rerun recording. Rerun is opened after completion, not connected to a live
simulator. The ROS adapter below remains a baseline/legacy entry point.
The stack documentation includes native ARM64 evidence, the build-only
CasADi-to-C backward-gradient prototype, and the Rust migration roadmap.

## Ownership

- `core/bringup/` owns **all runtime YAML, launch and RViz resources**.
- `planner::core::ReferenceTrajectory` owns the reference and exact time evaluation.
  The simulator neither defines another reference representation nor interpolates a sample table.
- `core/ros_packages/nmpc/controller/` owns ROS-free NMPC tracking logic, shared
  by `nmpc_node` and the simulator. The existing solver bundle remains the solver.
- `simple_sim/core/` owns simulation time, lifecycle, rate-loop actuation and physics.
- `io/` composes an experiment and records it; `app/` and `ros/` use the same Runner.

```text
simple_sim/
  core/{include/simple_sim,src}/   # Eigen + standard library only
  adapters/{include,src}/          # controller/aerodynamics owner interfaces
  io/{include,src}/                # config snapshots, trajectory export, metrics/logs
  app/main.cpp                    # headless executable
  ros/src/sim_node.cpp             # observation + pause/resume/step/reset
  test/                           # standalone contracts + integration tests
  docker/                         # image/Compose deployment

core/bringup/
  config/{planning,nmpc,simple_sim}.yaml
  launch/simple_sim.launch.py
  rviz/simple_sim.rviz
```

## Lockstep contract

One `Runner::step()` is one complete control period:

1. Read committed `x_k`, feedback and integer `t_k`.
2. Evaluate planner references at the controller's requested times; compute `u_k` synchronously.
3. Hold the outer-loop command and advance a fixed number of RK4 physics substeps.
4. Commit `x_{k+1}`, feedback and time together. A failed solve/physics step commits nothing.

`control_dt` comes from `nmpc.ctrl_frq`; it must be representable in nanoseconds
and divisible by `simple_sim.physics_dt_ns`. NMPC prediction spacing comes from
`nmpc.traj_res_s` and is **not** the control period or the physics step.
The bundle's actual horizon and each compiled time step are checked at startup.
No extrapolation is allowed: a run ends while its full prediction horizon still
fits inside the planned trajectory. A positive requested duration is rounded
down to complete control periods; `duration_s: 0` uses all available coverage.

Wall time measures solver cost and optionally paces ROS playback. It never
supplies integration `dt`, skips steps, or advances the reference while solving.
ROS timers only schedule complete Runner transactions. Publishing `/clock` is
an observation of those commits, not the synchronization mechanism.

Physics uses world **ENU**, body **FLU**, a Hamilton quaternion rotating body
vectors into world, diagonal rigid-body inertia, and optional Zhang-Lyu wing
FRD aerodynamics. The aerodynamic adapter converts `(x,y,z)_wing = (z,-y,x)_FLU`
and applies world-frame wind before conversion. IMU output is ideal body-frame
specific force, excluding gravity.

NMPC rotor states are already **N/kg**. Their sum is the specific-force command;
vehicle mass is applied exactly once by the actuator to obtain newtons. The
predicted first-node rates/thrust form a command, **never a replacement plant state**.
A stateless P rate loop with gyroscopic compensation and explicit torque/rate/thrust
limits drives the rigid body throughout each physics step. It is an idealized
inner loop, not an identified PX4 actuator model; gains require experiment-specific
validation. This simulator does not promise flight-calibrated tracking accuracy.

Non-finite inputs, invalid configuration, unsupported aero models and solver
failures terminate the experiment. Existing accepted solver status `2` is retained
and counted explicitly. There is no silent force-zeroing or last-command fallback.
Without ground contact, negative altitude is possible and is not corrected.

## Build and run (Linux)

```bash
export AP_PNC_DIR="$(git rev-parse --show-toplevel)"
# Required again after the solver wrapper API changes in this refactor.
uv run ap-pnc gen-nmpc-lib
uv run ap-pnc build simple-sim

# Headless: no ROS graph, RViz or K3s required.
uv run ap-pnc sim track
uv run ap-pnc sim track --controller se3

# ROS visualization/interactive operation, owned by bringup.
source "$AP_PNC_DIR/.artifacts/colcon/install/setup.bash"
ros2 launch bringup simple_sim.launch.py rviz:=true
ros2 service call /simple_sim/resume std_srvs/srv/Trigger '{}'
```

Both entrances load the same canonical bringup configurations. In repository
layout they are under `<AP_PNC_DIR>/core/bringup/config/`; flattened container
layout `<AP_PNC_DIR>/bringup/config/` is also supported. Only `AP_PNC_DIR` is a
runtime path environment variable. `--config`/launch `config:=` can specify an
**absolute** simple_sim override; `controller:=se3` selects the baseline.
The ROS simulator starts **paused**, so observers can attach before the first step.
Do not launch asynchronous flight planner/NMPC nodes alongside it.

Services: `/simple_sim/{pause,resume,step,reset}` (`std_srvs/Trigger`). Single-step
requires pause and advances one control period. Reset starts a fresh recording,
reuses the exact planned trajectory and frozen configuration, clears solver and
simulation state, and returns to paused time zero. Reset does not reload/replan;
restart to apply configuration changes.

Observation topics: `/clock`, `/simple_sim/odom`, `/simple_sim/imu`,
`/simple_sim/aero_wrench`, `/simple_sim/path`, `/simple_sim/reference`.
Odometry pose is world-frame; twist and wrench are body-frame. RViz uses
`use_sim_time`. Visualization is decimated and its path history is bounded.
The default ROS middleware is used; no custom RMW/FastDDS profiles are supplied.

## Tests

```bash
colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" test \
  --build-base "$AP_PNC_DIR/.artifacts/colcon/build" --packages-select simple_sim
colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" test-result \
  --test-result-base "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim"
```

For a standalone core-only check (Eigen3 installed):

```bash
cmake -S "$AP_PNC_DIR/infra/sim_infra/simple_sim" \
  -B "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim_core_tests" \
  -DSIMPLE_SIM_BUILD_APP=OFF -DSIMPLE_SIM_WITH_ROS=OFF -DSIMPLE_SIM_WITH_NMPC=OFF
cmake --build "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim_core_tests"
ctest --test-dir "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim_core_tests" --output-on-failure
```

The core suite covers analytic dynamics, frames/IMU, RK4 convergence, saturation,
slow-controller wall-time invariance, pause/reset, rollback and NaN rejection.
NMPC contract tests explicitly use a fake solver to check layout, units, status,
reset and stale-grid rejection; they are **not** acados integration claims.
App builds additionally check planner-reference SE3 tracking. NMPC builds run the
ordered **9 m/s acceptance** below; baseline-only builds check full-run exact
repeatability, excluding diagnostic wall-time fields. ROS builds test `/clock`
and pause/resume/step/reset in an isolated localhost ROS domain. None are
flight-performance acceptance tests. Repeated builds/platforms are not promised
bitwise identical floating-point behavior.

### Acceptance: no aero, then Lyu, planning v_max = 9 m/s

```bash
ctest --test-dir "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim" \
  -R simple_sim_acceptance_v9 --output-on-failure
```

This test executes the **real NMPC bundle**, not the fake contract solver:

1. Write run-local copies of the bringup configs and set planner `cost.v_max: 9.0`.
2. Plan once; run the full available horizon-covered interval with `aero.model: none`.
3. Reuse the exact same planner trajectory and all other parameters with `aero.model: lyu`.
   (`aero.model` accepts the canonical set `lyu | phi | ma | advanced | none`; the
   acceptance regression intentionally exercises `none` and `lyu`.)
4. Repeat each case; require completion, finite state/control, no rejected solver
   result, and exact state/reference/control replay (wall time excluded).

The canonical flight `planning.yaml` is not edited. `acceptance.json` under
`.artifacts/benchmark/simple_sim_acceptance_<id>/` links the four individual runs
and reports sampled reference/actual speed peaks and position errors. Planner
speed limits are soft optimization constraints, so the measured peak is reported
rather than silently clipping the trajectory. Completion/replay is the current
acceptance criterion; tracking-error thresholds are not silently assumed.

## Artifacts and deployment

Every run creates a fresh `<AP_PNC_DIR>/.artifacts/benchmark/simple_sim_<id>/`:

- `simple_sim.yaml`, `planning.yaml`, `nmpc.yaml`: frozen inputs actually executed;
- `reference.yaml`: exact planner-owned polynomial coefficients, durations and frame;
- `steps.csv`: aligned before/after state, reference, command, feedback and solver diagnostics;
- `manifest.json`: completion/failure, last committed tick, revision/dirty status,
  compiler/platform and bundle SHA256;
- `metrics.json`: full-run position/velocity RMSE, maximum position error and status-2 count.

JSON summaries are replaced atomically; committed CSV rows are flushed. A reset
creates a new run directory rather than overwriting an earlier run. No stochastic
model is currently present. Headless SIGINT/SIGTERM records interruption.

Build/install/log output stays under `.artifacts/colcon/`. No generated files
belong in the source tree. NMPC is Linux-only; `SIMPLE_SIM_WITH_NMPC=OFF` permits
non-acados baseline/core checks but does not make macOS a supported flight host.

Docker build selects the native Linux CPU target (Apple Silicon: ARM64),
creates a missing arch-tagged bundle in Docker and verifies its ELF members:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc sim build --target simple
# Explicit alternate target:
uv run --project "$AP_PNC_DIR" ap-pnc sim build --target simple --arch amd64
```

Raw Compose works as-is (`--profile build` is what the tooling adds); the
architecture comes from the generated override, while
building the shared ROS base first, then the simple-sim service. The image is `simple-sim:<docker_arch>`.
All services use host IPC; simple-sim uses host networking/FastDDS. The default
ROS launch is headless (`rviz:=false`), with no host X11 mounts.
K3s image tags must be updated independently before switching that backend.

Compose mounts only benchmark data and canonical bringup configs, not all artifacts
(which would hide the image's install tree). K3s selects the image via `SIM_TYPE`;
it no longer embeds a second simulation YAML in its ConfigMap.
