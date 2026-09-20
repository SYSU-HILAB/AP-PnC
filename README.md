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
└──────────────┬─────────────────────────────────────────────────────┘
               │ /cur_state ↑   /mpc_controls ↓
┌─ nav_infra (sidecar) ──────────────────────────────────────────────┐
│ px4ctrl — FCU interface: arming/offboard/takeoff/hover, single      │
│   mavros setpoint publisher, state adapter (/cur_state), controller │
│   modes: tracking (ctrl_mode=0) | NMPC rates + RLS/INDI (ctrl_mode=1)│
│ mavros2 + (real profile) livox + fast_lio + ekf_quat               │
└──────────────┬─────────────────────────────────────────────────────┘
               │ MAVLink UDP 14580↔14540
   stock PX4 v1.17.0 + Gazebo Harmonic (gz_x500) — no firmware forks
```

- **Handover**: `ros2 service call /px4ctrl/toggle_pass_through
  std_srvs/srv/SetBool "{data: true}"` (synchronous) switches px4ctrl from
  tracking hover to its remote-rates source: the internal NMPC controller
  (`ctrl_mode=1`, `/mpc_controls` → RLS eta + INDI thrust) or an external
  rates stream (`ctrl_mode=0`); stale streams fall back to hover
  automatically. CLI shortcut: `uv run ap-pnc stack trigger go|back`.
- **Throttle estimation**: `throttle_estimator=0` keeps the legacy
  `thr2acc` mapping, `throttle_estimator=1` uses the RLS eta + INDI
  incremental law in the tracking controller too. Both are startup
  parameters in `px4ctrl.yaml`/`px4ctrl_real.yaml`.
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
core/                # Research code — owns only itself
  ros_packages/      #   planner (tailsitter_df + trajectory_server + traj_buffer),
                     #   c_nmpc, utils (+msgs interface contract)
  bringup/           #   core config/launch: planning, c_nmpc, fastdds
  docker/            #   amd64/arm64 ros2-base dev images
infra/               # Infrastructure — each unit self-contained
  nav_infra/         #   ros_packages/ (px4ctrl tracking+NMPC/INDI, quadrotor_msgs,
                     #   livox, fast_lio, ekf_quat_pose)
                     #   + own bringup/ (nav profiles, px4ctrl{,_real}) + docker/Dockerfile
  sim_infra/         #   gazebo/ (stock PX4 v1.17 + Harmonic stack), simple_sim/, aerodynamics/
  sim_k3s/           #   ConfigMap-driven sim orchestration manifests
ap_pnc/              # Python CLI — top-level orchestration (stack start, build, sim)
benchmark/           # Prior-based experiment configs (Lyu force-balance)
.agents/skills/      # bootstrap / docker / gazebo-e2e (flight runbook + pitfalls)
```

## Runtime Configuration

Core nodes read sectioned yaml directly from the cwd-relative
`core/bringup/config/` (matches the container `/workspace` mount):

- `planning.yaml` — `problem_formulation` (box, basic_traj, nested cost,
  MINCO, prior-free phi-theory flatness) + `io` (topics, dump dirs)
- `c_nmpc.yaml` — `nmpc: {mass, cz_rate}`

nav_infra nodes read from `infra/nav_infra/bringup/config/`:

- `px4ctrl.yaml` / `px4ctrl_real.yaml` — sim/real control parameters:
  `ctrl_mode` (0 tracking / 1 NMPC), `throttle_estimator` (0 mapping /
  1 RLS eta + INDI), `rls.*`, `indi.*`, gains, takeoff/land, timeouts
  (px4ctrl is the one ROS-parameter exception, vendored style)
- `nav/` — mavros sim/real endpoints, livox, fast_lio, ekf_quat profiles
  (mavros plugin sub-node params are set by `nav_infra.launch.py`, not by
  these files)

---

## Simulation Layers

| Layer | Directory | What it runs |
|-------|-----------|--------------|
| Gazebo | `infra/sim_infra/gazebo/` | Generic-interface SITL: stock PX4 v1.17.0 under Gazebo Harmonic (`gz_x500`), bridged by mavros2 (`px4-simulator` + `nav_infra` + `qground_control` + `ap-pnc`) |
| Simple sim | `infra/sim_infra/simple_sim/` | Lightweight C++ simulator (MARSIM-style) with synthetic LiDAR sensing |

```bash
uv run ap-pnc sim gazebo     # Gazebo layer via K3s ConfigMap
uv run ap-pnc sim simple     # simple_sim layer
uv run ap-pnc sim stop       # scale everything down
```

For the full flight procedure (takeoff → hover → NMPC handover) and the
SITL pitfalls reference, see the `ap-pnc-gazebo-e2e` skill in
`.agents/skills/`. Details of each layer: `infra/nav_infra/README.md`,
`infra/sim_infra/gazebo/README.md`, `infra/sim_infra/simple_sim/README.md`.

---

## Generated Artifacts and Git Worktrees

Everything generated lands in a single untracked `.artifacts/` root (acados
staging, codegen output, `libnmpc_bundle.a`, pybind11 bindings, colcon
build/install/log). Python `.venv` is shared across worktrees; `.artifacts/`
(including colcon output) is worktree-local.

Zero environment variables are required — the CLI sets up every path it
needs internally.
