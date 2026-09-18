# AP-PnC

Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

---

## Install

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git
uv sync --group dev --group extra
uv run ap-pnc install
uv run ap-pnc gen-nmpc-lib   # acados codegen → self-contained libnmpc_bundle.a
```

No recursive clone needed: `ap-pnc install` fetches acados automatically
(commit-locked into `.artifacts/acados_src`). The vendored dependencies
(`ap_pnc/sciplots`, `autodiff`, `ROS2-Utils`, `tomlplusplus`, `yamlcpp`,
`BSplinebasis`, `px4ctrl`, `quadrotor_msgs`, `livox_ros_driver2`, `fast_lio`,
`ekf_quat_pose`) are git subtrees and live directly in the repo.

---

## Architecture

Research code is separated from base services:

```
┌─ ap-pnc (core) ────────────────────────────────────────────────────┐
│ planner (tailsitter_df phi-theory + trajectory_server + traj_buffer)│
│ c_nmpc  — acados NMPC tracker → /mpc_controls (FLU rates + thrust)  │
│ px4ctrl — arming/offboard/takeoff/hover + PASS_THROUGH forwarding   │
└──────────────┬─────────────────────────────────────────────────────┘
               │ /cur_state ↑   /mpc_controls + toggle service ↓
┌─ nav_infra (sidecar) ──────────────────────────────────────────────┐
│ nmpc_bridge — RLS throttle model + INDI thrust, odom world-frame   │
│ mavros2 + (real profile) livox + fast_lio + ekf_quat               │
└──────────────┬─────────────────────────────────────────────────────┘
               │ MAVLink UDP 14580↔14540
   stock PX4 v1.17.0 + Gazebo Harmonic (gz_x500) — no firmware forks
```

- **Handover**: `ros2 service call /px4ctrl/toggle_pass_through
  std_srvs/srv/SetBool "{data: true}"` (synchronous) switches px4ctrl from
  hover to verbatim forwarding of the NMPC rates stream; stale streams fall
  back to hover automatically. CLI shortcut: `uv run ap-pnc stack trigger
  go|back`.
- **Profiles**: same nav_infra image for sim and real —
  `profile:=sim|real` selects mavros-only vs livox+fast_lio+ekf_quat and the
  px4ctrl parameter set (`px4ctrl.yaml` hover 0.9 vs `px4ctrl_real.yaml`
  hover 0.5).

---

## CLI Usage

All project commands are accessed through the unified CLI — no Makefile, no
shell wrappers:

```bash
uv run ap-pnc --help
```

| Group          | Commands | Description |
|----------------|----------|-------------|
| `install`      | (default) | Install everything (acados + deps) |
| `gen-nmpc-lib` | (default) | acados NMPC codegen + self-contained `libnmpc_bundle.a` |
| `traj`         | `viz`, `viz-tracking`, `viz-elmpc`, `odom` | Trajectory visualization |
| `aerodyn`      | `gen`, `viz`, `ml train\|pipeline`, `show`, `direct-show`, `csv-vs-model` | Aerodynamics |
| `benchmark`    | `run`    | Planning experiments (`--vmax --type --radius` → sync config → start stack) |
| `test`         | `all`, `nmpc`, `lmpc`, ..., `timing-jax/torch/compare`, `colcon`, `colcon-result` | Test runners |
| `build`        | `all`, `cnmpc` (c_nmpc + px4ctrl), `aero`, `simple-sim`, `traj-server`, `bindings` | C++ colcon builds |
| `sim`          | `simple`, `gazebo`, `stop`, `status`, `logs`, `pcd` | K3s ConfigMap-driven simulation |
| `stack`        | `start`, `trigger go\|back` | Local ROS2 stack lifecycle (go/back = PASS_THROUGH service) |
| `debug`        | `echo`, `pcd` | Interactive debugging (fzf topic picker, open3d viewer) |
| `nn-trim`      | `run`, `plot` | Neural network trim analysis |
| `docker`       | `build`, `up`, `down` | Docker compose commands |
| `lint`         | (default), `fmt` | Ruff linter and formatter |

### Examples

```bash
uv run ap-pnc gen-nmpc-lib         # acados NMPC codegen + bundle
uv run ap-pnc traj viz             # Launch reference trajectory + RViz2
uv run ap-pnc benchmark run --vmax 5 --type 1 --radius 4   # Planning experiment
uv run ap-pnc test nmpc            # Run proposed NMPC test
uv run ap-pnc build all            # Build the whole workspace (core + nav_infra)
uv run ap-pnc lint                 # Lint
```

---

## Repository Layout

```
ros_packages/        # Research code
  planner/           #   trajectory generation: tailsitter_df (DF phi-theory
                     #   kernel), basic_trajectories, trajectory_server, traj_buffer
  c_nmpc/            #   NMPC tracking node + nmpc_wrapper
  px4ctrl/           #   vendored L2 control (arming/offboard/takeoff/PASS_THROUGH)
  quadrotor_msgs/    #   vendored (PositionCommand, TakeoffLand, Px4ctrlDebug)
  utils/msgs/        #   interface package (shared ROS msg contract, no px4_msgs)
  utils/             #   ROS2-Utils + vendored libraries
nav_infra/           # Base-services sidecar (not research code)
  ros_packages/      #   px4_interface (nmpc_bridge RLS+INDI), livox, fast_lio, ekf_quat_pose
  docker/Dockerfile  #   nav_infra:latest — mavros2 (source) + gateway + lio, sim/real profiles
bringup/             # Single config/launch source: planning, c_nmpc, px4ctrl (sim/real),
                     # mavros_bridge, nav profiles; bringup/nav_infra.launch.py
ap_pnc/              # Python package: CLI, nmpc/ codegen toolchain, lu_mpc, aerodynamics
simulations/         # aerodynamics (ground truth), gazebo, simple_sim, k3s
benchmark/           # Prior-based experiment configs (Lyu force-balance)
docker/              # amd64/arm64 ros2-base images
.agents/skills/      # bootstrap / docker / gazebo-e2e (flight runbook + pitfalls)
```

## Runtime Configuration

No ROS parameters for core nodes — they read sectioned yaml directly from
the cwd-relative `bringup/config/` (matches the container `/workspace`
mount):

- `planning.yaml` — `problem_formulation` (box, basic_traj, nested cost,
  MINCO, prior-free phi-theory flatness) + `io` (topics, dump dirs)
- `c_nmpc.yaml` — `nmpc: {mass, cz_rate}`
- `mavros_bridge.yaml` — `nmpc_bridge` (RLS throttle model + INDI thrust
  law, odom source switch `/mavros/local_position/odom` sim vs
  `/ekf_quat/ekf_odom` real)
- `px4ctrl.yaml` / `px4ctrl_real.yaml` — sim/real control parameters
  (px4ctrl is the one ROS-parameter exception, vendored style)
- `nav/` — mavros sim/real endpoints, livox, fast_lio, ekf_quat profiles

---

## Simulation Layers

| Layer | Directory | What it runs |
|-------|-----------|--------------|
| Gazebo | `simulations/gazebo/` | Generic-interface SITL: stock PX4 v1.17.0 under Gazebo Harmonic (`gz_x500`), bridged by mavros2 (`px4-simulator` + `nav_infra` + `qground_control` + `ap-pnc`) |
| Simple sim | `simulations/simple_sim/` | Lightweight C++ simulator (MARSIM-style) with synthetic LiDAR sensing |

```bash
uv run ap-pnc sim gazebo     # Gazebo layer via K3s ConfigMap
uv run ap-pnc sim simple     # simple_sim layer
uv run ap-pnc sim stop       # scale everything down
```

For the full flight procedure (takeoff → hover → NMPC handover) and the
SITL pitfalls reference, see the `ap-pnc-gazebo-e2e` skill in
`.agents/skills/`. Details of each layer: `nav_infra/README.md`,
`simulations/gazebo/README.md`, `simulations/simple_sim/README.md`.

---

## Generated Artifacts and Git Worktrees

Everything generated lands in a single untracked `.artifacts/` root (acados
staging, codegen output, `libnmpc_bundle.a`, pybind11 bindings, colcon
build/install/log). Python `.venv` is shared across worktrees; `.artifacts/`
(including colcon output) is worktree-local.

Zero environment variables are required — the CLI sets up every path it
needs internally.
