# AP-PnC

Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

---

## Quick Start

### 1. Install uv and git

The only host prerequisites are [uv](https://docs.astral.sh/uv/) (Python
package/environment manager) and git. Both are standard on Linux:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # install uv
sudo apt install git                                # or your distro's git package
```

Verify: `uv --version` and `git --version`.

### 2. Clone the repository

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git
cd AP-PnC
```

No recursive clone needed: `ap-pnc install` fetches acados automatically
(commit-locked into `.artifacts/acados_src`). The vendored dependencies
(`autodiff`, `px4ctrl`, `quadrotor_msgs`, `livox_ros_driver2`, `fast_lio`,
`ekf_quat_pose`) are git subtrees and live directly in the repo.
Aerodynamics ships its own B-spline evaluator (`aero_utils` + a self-contained
`bspline_basis.hpp`), so no third-party B-spline library is vendored.

### 3. Full-stack install

The one-shot bootstrap (venv + acados + OCP bundle + Docker images), with a
step-by-step log:

```bash
uv run ap-pnc stack full-install          # everything, ap-pnc + nav_infra images
uv run ap-pnc stack full-install --docker-all   # + px4-simulator + qground_control (slow)
```

Or step-by-step, for a C++/ROS2-only pipeline:

```bash
uv sync --group dev --group extra   # Python env: dev + ML/compute extras
uv run ap-pnc install               # acados C libs (shared+static), t_renderer, acados_template
uv run ap-pnc gen-nmpc-lib          # acados codegen → self-contained libnmpc_bundle.a
```

- `uv sync` creates the project `.venv`; `--group extra` pulls the ML/compute
  stack (JAX, Torch, CasADi, matplotlib, ...) used by the aerodynamics
  commands — omit it if you only run the C++/ROS2 pipeline.
- `ap-pnc install` builds acados from the locked source, downloads
  `t_renderer`, and installs `acados_template` into the venv.
- `ap-pnc gen-nmpc-lib` generates the OCP solver code and merges it with
  acados/hpipm/blasfeo into the self-contained static bundle
  `.artifacts/nmpc_solver/libnmpc_bundle.a` for ROS2 linking.

Sanity check:

```bash
uv run ap-pnc --help
ls -lh .artifacts/nmpc_solver/libnmpc_bundle.a
```

For a fresh Linux host the full dependency checklist is in the
`ap-pnc-bootstrap` skill (`.agents/skills/`); the Docker/ROS2 stack is
covered by the `ap-pnc-docker` skill.

---

## Tuning the NMPC Parameters

The NMPC tracking controller has two parameter surfaces: the
horizon/model convention read at runtime, and the OCP cost weights baked in
at codegen time.

### Runtime parameters: `bringup/config/nmpc.yaml`

| Key          | Default | Meaning |
|--------------|---------|---------|
| `horizon_s`  | 1.0     | Prediction horizon [s] (acados tf) |
| `traj_res_s` | 0.1     | Horizon node spacing [s]; grid = `N + 1` reference points, `N = round(horizon_s/traj_res_s)` |
| `ctrl_frq`   | 50.0    | Solve / reference-request frequency [Hz] |
| `u_hover`    | 2.4525  | Actuation reference at hover [N/kg per rotor] |
| `mass`       | 2.0     | Vehicle mass [kg] |
| `cz_rate`    | 0.10    | Vertical drag (cz) rate parameter |

The planner builds the reference on exactly this grid (no downsampling):
point `k` is evaluated at `t0 + k·traj_res_s`, the playhead advances by
`1/ctrl_frq` per request. Changing the grid (`horizon_s` / `traj_res_s`)
also changes the codegen discretization — see below.

### Cost weights: `ap_pnc/nmpc_gen/config.py` (`MpcConfig`)

The convex-over-nonlinear least-squares cost lives in
`ap_pnc/nmpc_gen/config.py`:

| Field    | Meaning |
|----------|---------|
| `qPos`   | Position tracking weight `[x, y, z]` |
| `qVel`   | Velocity tracking weight |
| `qAngV`  | Body-rate tracking weight |
| `qYB`    | Body-y (lateral) tracking weight, derived from `qh`/`qv` |
| `rU`     | Control-effort weight (per rotor `u`) |
| `lb_r`/`ub_r` | Body-rate bounds |
| `lb_u`/`ub_u` | Actuation bounds |

The cost layout is `[pos, vel, rate, yb, u]`; the rate and actuation
references are controller-side (rate weight is zero, `u` holds at hover).

Changing any weight, bound, or the horizon grid changes the OCP itself, so
regenerate the solver and rebuild the node:

```bash
uv run ap-pnc gen-nmpc-lib    # regenerate solver + bundle
uv run ap-pnc build nmpc      # rebuild the nmpc node against the new bundle
```

---

## Architecture

Research code is separated from base services:

```
┌─ ap-pnc (core) ────────────────────────────────────────────────────┐
│ planner — GCOPTER/MINCO → buffered RefTrajectory → /nmpc/reference│
│ nmpc    — acados NMPC → /nmpc/control (FLU rates + specific force)  │
└──────────────┬─────────────────────────────────────────────────────┘
               │ /px4ctrl/state ↑   /nmpc/control ↓
┌─ nav_infra (sidecar) ──────────────────────────────────────────────┐
│ px4ctrl — FCU interface: arming/offboard/takeoff/hover, single      │
│   mavros setpoint publisher, state adapter (/px4ctrl/state), controller │
│   modes: tracking (ctrl_mode=0) | NMPC rates + RLS/INDI (ctrl_mode=1)│
│ mavros2 + (real profile) livox + fast_lio + ekf_quat               │
└──────────────┬─────────────────────────────────────────────────────┘
               │ MAVLink UDP 14580↔14540
   stock PX4 v1.17.0 + Gazebo Harmonic (gz_x500) — no firmware forks
```

- **Handover**: `ros2 service call /px4ctrl/toggle_pass_through
  std_srvs/srv/SetBool "{data: true}"` (synchronous) switches px4ctrl from
  tracking hover to its remote-rates source: the internal NMPC controller
  (`ctrl_mode=1`, `/nmpc/control` → RLS eta + INDI thrust) or an external
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
| `start`        | `--target simple\|gazebo\|stop\|stack` | Start the project runtime |
| `install`      | (default) | Install everything (acados + deps) |
| `gen-nmpc-lib` | (default) | acados NMPC codegen + self-contained `libnmpc_bundle.a` |
| `benchmark`    | `run`, `flight` | `run` = offline pipeline (plan → SE3 → data); `flight` = sync config → start stack |
| `test`         | `all`, `nmpc`, `lmpc`, ..., `colcon`, `colcon-result` | Test runners |
| `build`        | `all`, `nmpc` (nmpc + px4ctrl), `planner`, `planner-bindings`, `aero`, `simple-sim`, `bindings` | C++ colcon builds + pybind |
| `sim`          | `simple`, `gazebo`, `stop`, `status`, `logs` | K3s ConfigMap-driven simulation |
| `stack`        | `full-install`, `start`, `trigger go\|back` | Full bootstrap (venv+acados+bundle+Docker) / local ROS2 stack lifecycle (go/back = PASS_THROUGH service) |
| `docker`       | `build`, `up`, `down` | Docker compose commands |
| `lint`         | (default), `fmt` | Ruff linter and formatter |

### Examples

```bash
uv run ap-pnc gen-nmpc-lib         # acados NMPC codegen + bundle
uv run ap-pnc build planner-bindings  # pybind planning core for the benchmark
uv run ap-pnc benchmark run --vmax 8 --type 2 --radius 12   # Offline SE3 benchmark
uv run ap-pnc benchmark flight --vmax 5 --type 1 --radius 4 # Sync config + start stack
uv run ap-pnc build all            # Build the whole workspace (core + nav_infra)
uv run ap-pnc lint                 # Lint
```

### Offline benchmark pipeline

`ap-pnc benchmark run` orchestrates the whole offline loop in one process:

```
planning.yaml ─► planner_core.plan() ─► Reference ─► SE3 ─► rigid-body plant ─► data.npz + metrics
                  (pybind, pure C++)     (numpy)    controller
```

Artifacts land in `.artifacts/benchmark/<name>/` (`planning.yaml`, `data.npz`,
`metrics.json`). No ROS and no CSV are involved; the real C++ planner is
reused in-process via `planner_bindings`.

---

## Repository Layout

```
core/                # C++/ROS research code
  ros_packages/      #   planner (core/ pure C++ + node/ ROS adapter + python/ pybind),
                     #   nmpc, interface (the message contract)
  bringup/           #   core config/launch: planning, nmpc
  docker/            #   amd64/arm64 ros2-base dev images
infra/               # Infrastructure — each unit self-contained
  nav_infra/         #   ros_packages/ (px4ctrl tracking+NMPC/INDI, quadrotor_msgs,
                     #   livox, fast_lio, ekf_quat_pose)
                     #   + own bringup/ (nav profiles, px4ctrl{,_real}) + docker/Dockerfile
  sim_infra/         #   gazebo/ (stock PX4 v1.17 + Harmonic), simple_sim/,
                     #   aerodynamics/ (C++ ROS pkg + python/ ML service + bindings)
  sim_k3s/           #   ConfigMap-driven sim orchestration manifests
ap_pnc/              # Python
  cli.py, env.py, commands/          # the ap-pnc CLI
  nmpc_gen/                          # acados OCP codegen (core algorithm)
  paper/{benchmark,figure,methods/lu}/  # paper layer
test/                # pytest + manual scripts
.agents/skills/      # bootstrap / docker / gazebo-e2e (flight runbook + pitfalls)
```

## Runtime Configuration

Nodes read sectioned yaml from `<AP_PNC_DIR>/bringup/config/`:

- `planning.yaml` — `problem_formulation` (box, basic_traj, nested cost,
  MINCO, prior-free phi-theory flatness)
- `nmpc.yaml` — the horizon convention + model:
  `nmpc: {horizon_s, traj_res_s, ctrl_frq, mass, cz_rate, u_hover}`

The aerodynamics alpha-solver reads
`infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml`.

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

## Generated Artifacts

Everything generated lands in a single untracked `.artifacts/` root (acados
staging, codegen output, `libnmpc_bundle.a`, pybind11 bindings, colcon
build/install/log). Python `.venv` is shared across worktrees; `.artifacts/`
(including colcon output) is worktree-local.

Zero environment variables are required — the CLI sets up every path it
needs internally.
