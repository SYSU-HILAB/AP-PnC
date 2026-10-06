# AP-PnC

Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

## Paper

This repository is the reference implementation of

> **Aerodynamic Prior-Free Coordinated Trajectory Generation and Tracking Control
> for a Tail-Sitter UAV**
> Erchao Rong, Zihao Liu, Junning Liang, Jianguo Wang, Xiao Jie, Haoran Fu,
> Ziliang Chen, Ximin Lyu
> arXiv:2609.11698 — <https://arxiv.org/abs/2609.11698>

```bibtex
@article{rong2026apPnc,
  title  = {Aerodynamic Prior-Free Coordinated Trajectory Generation and
            Tracking Control for a Tail-Sitter UAV},
  author = {Rong, Erchao and Liu, Zihao and Liang, Junning and Wang, Jianguo and
            Jie, Xiao and Fu, Haoran and Chen, Ziliang and Lyu, Ximin},
  journal = {arXiv preprint arXiv:2609.11698},
  year   = {2026},
  url    = {https://arxiv.org/abs/2609.11698}
}
```

The paper's two phase-specific choices are the contract this code follows:

| Phase | Paper | Repository |
|---|---|---|
| Planning | full phi-theory flatness under coordinated flight, parameterised by one flat-plate drag coefficient `C_D,flat = 1.28` | `tailsitter_df/flu_reference.hpp` + `flu_flatness.hpp` feed the MINCO cost |
| Tracking | simplified model with **one** non-linear longitudinal aerodynamic parameter `c_z` estimated online | NMPC state `cx` (same physical axis: the wing-normal direction) |

The paper also states two modelling decisions we rely on: the lateral coefficient
vanishes under the coordinated-flight assumption, and the aerodynamic moment is
neglected because the **inclined motor layout** provides sufficient control
authority. Axis labels differ from the paper: the paper puts the thrust axis on
body `x`, this repository puts it on body `+Z` (FLU), so the paper's `c_z` is our
`cx`.

## Numerical specifications

- [Main ENU/FLU frames and simulation-only legacy wing boundary](docs/coordinate_frames.md)
- [v_max=6 tracking acceptance and actual NMPC reference plots](docs/tracking_evaluation.md)
- [6 m/s 历史窗口测试报告](docs/tracking_v6_report.md)
- [6/8/10/12 m/s + 末点悬停测试报告](docs/tracking_speed_report.md)

These define the migration/acceptance contract. The direct FLU public reference
and a five-model matched-reference simulation sweep are now implemented; the
legacy optimization cost kernel and broader dynamic/frame acceptance remain pending.

## Observability

[Project-wide OpenTelemetry contract and rollout](docs/observability.md) covers
traces, metrics, correlated events, real-time budgets, clocks and privacy.
The local Collector, Python CLI and real dora simulate→record→verify trace chain
are implemented. C++ subprocess/decoder boundaries and logged solver diagnostics
are correlated; internal solver/plant function spans and ROS/SITL remain pending.

---

## Quick Start

### 1. Install uv and git

Host prerequisites are [uv](https://docs.astral.sh/uv/), git and Docker
with Compose/BuildKit. ROS2, PX4, Gazebo, QGC and the solver run in Linux
containers. Apple Silicon defaults to native Linux ARM64, not AMD64 emulation.
For macOS host-network requirements, see the [Gazebo guide](infra/sim_infra/gazebo/README.md).

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # install uv
sudo apt install git                                # or your distro's git package
```

Verify: `uv --version` and `git --version`.

### 2. Clone the repository

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
git clone https://github.com/WarriorHanamy/AP-PnC.git "$AP_PNC_DIR"
```

No recursive clone needed: `ap-pnc install` fetches acados automatically
(commit-locked into `.artifacts/acados_src`). The vendored dependencies
(`autodiff`, `px4ctrl`, `quadrotor_msgs`, `livox_ros_driver2`, `fast_lio`,
`ekf_quat_pose`) are git subtrees and live directly in the repo.
Aerodynamics ships its own B-spline evaluator (`aero_utils` + a self-contained
`bspline_basis.hpp`), so no third-party B-spline library is vendored.

### 3. Full-stack install

Build/start the host-native all-Docker simulation, headless by default:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc docker build
uv run --project "$AP_PNC_DIR" ap-pnc docker up
# Optional QGC + Gazebo container desktop (http://127.0.0.1:6088/vnc.html):
uv run --project "$AP_PNC_DIR" ap-pnc docker build --gui
uv run --project "$AP_PNC_DIR" ap-pnc docker up --gui
uv run --project "$AP_PNC_DIR" ap-pnc docker down
```

Build/up accept `--arch arm64|amd64`; default is the host CPU. A missing
arch-tagged bundle is generated in the pinned Docker builder with isolated
caches. Every service uses host IPC; ROS2 and SITL also use host networking.
Startup automatically launches the processes, but does not request takeoff.
Current exception: the new production PX4 rebuild is **temporarily deferred on
Mac**; complete it on native Linux runners. ARM64 core/nav, solver, SHM and QGC
results are recorded in the [validation table](infra/sim_infra/gazebo/README.md#validation-status-2026-10-05).
Those results are not full-flight or native AMD64 acceptance.
`stack full-install` is now Docker-only (core/nav by default; `--docker-all`
includes SITL and GUI), with no host ROS/acados installation.

Optional **native Linux** toolchain/research setup, not the Mac runtime path:

```bash
uv sync --project "$AP_PNC_DIR" --group dev --group extra
uv run --project "$AP_PNC_DIR" ap-pnc install
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
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
uv run --project "$AP_PNC_DIR" ap-pnc --help
ls -lh "$AP_PNC_DIR/.artifacts/nmpc_solver/"libnmpc_bundle_*.a
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
| `initial_specific_thrust` | 9.81 | Initial collective command [N/kg], NOT a cost target |
| `cx_alpha_slope` | 0.10 | Lumped body-X coefficient evolution parameter |

`nmpc.yaml` carries **no mass, area or inertia**: the NMPC model is
mass-normalized (specific thrust in N/kg, `cx` in 1/m). Mass belongs to the
planner/plant, which keep their own `flatness.mass` / `plant.mass` check.

The planner builds the reference on exactly this grid (no downsampling):
point `k` is evaluated at `t0 + k·traj_res_s`, the playhead advances by
`1/ctrl_frq` per request. Changing the grid (`horizon_s` / `traj_res_s`)
also changes the codegen discretization — see below.

### Cost weights and bounds: the same `nmpc.yaml`

`config.py` and its duplicate `MpcConfig` are removed. YAML `objective` owns
`position`, `velocity`, `body_y` and `command_derivative` weights;
`command_bounds.lower/upper` bound the four absolute commands, not their derivatives.

The objective output is `[pos(3), vel(3), yb(3), dc/dt(4)]`, with reference
`[pos_ref, vel_ref, yb_ref, zeros(4)]`; terminal output is `[pos, vel, yb]`.
No absolute hover/body-rate command target exists. Command states obey `dc/dt=u`;
there is no first-order actuator lag in prediction. Stage 0 includes the derivative
penalty. See [NMPC model and cost](docs/nmpc_model.md).

Changing any weight, bound, or the horizon grid changes the OCP itself, so
regenerate the solver and rebuild the node:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib # regenerate solver + bundle
uv run --project "$AP_PNC_DIR" ap-pnc build nmpc   # statically relink the node
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
| `docker`       | `build`, `up`, `down`, `config`, `dev` | Native-target Docker simulation; optional `--gui` |

### Examples

The shortest path from a fresh clone to a running simulation:

```bash
# 1. Generate the acados NMPC solver (Linux)
uv run ap-pnc gen-nmpc-lib

# 2a. Simple tracking sim — headless, no ROS graph or K3s required
uv run ap-pnc build simple-sim
uv run ap-pnc sim track --controller nmpc

# 2b. Or the full PX4 + Gazebo stack via K3s
uv run ap-pnc sim gazebo
uv run ap-pnc sim logs -f          # stream the running sim pod's logs
uv run ap-pnc sim stop

# 3. Inspect the results
#    metrics are printed at the end of each run; with ROS the simple sim
#    can also be visualized live in RViz:
source "$AP_PNC_DIR/.artifacts/colcon/install/setup.bash"
ros2 launch bringup simple_sim.launch.py rviz:=true
```

Web visualization of trajectories and tracking results in the browser is on
the near-term plan — see the [Roadmap](#roadmap).

---

## Benchmark

Benchmarks are a first-class workflow, kept separate from the quick-start
examples above. Every run lands in `.artifacts/benchmark/<name>/`.

### Offline tracking benchmark (implemented)

`ap-pnc benchmark run` orchestrates the whole offline loop in one process:

```
planning.yaml ─► planner_core.plan() ─► Reference ─► SE3 ─► rigid-body plant ─► data.npz + metrics
                  (pybind, pure C++)     (numpy)    controller
```

```bash
uv run ap-pnc build planner-bindings                        # pybind planner core
uv run ap-pnc benchmark run --vmax 8 --type 2 --radius 12   # plan → track → data
```

Artifacts: `planning.yaml` (exact config snapshot), `data.npz`, `metrics.json`.
No ROS and no CSV are involved; the real C++ planner is reused in-process via
`planner_bindings`.

### Flight benchmark (SITL)

`ap-pnc benchmark flight` syncs `planning.yaml` with the requested trajectory
and starts the full stack; the flight itself (takeoff → hover → NMPC handover)
follows the `ap-pnc-gazebo-e2e` skill runbook in `.agents/skills/`.

```bash
uv run ap-pnc benchmark flight --vmax 5 --type 1 --radius 4
```

### Planned extensions

- NMPC vs SE3 controller comparison in one sweep (simple_sim already accepts
  `--controller nmpc|se3`)
- Metric aggregation across sweeps into a single comparison table
- Web dashboard for browsing runs (see Roadmap)

---

## Roadmap

**Done**

- Single-path install: `ap-pnc install` (auto acados fetch) + `gen-nmpc-lib`
- Planner / NMPC nodes with PASS_THROUGH handover (`ap-pnc stack trigger go`)
- K3s ConfigMap simulation switching: `ap-pnc sim simple|gazebo|stop`
- Headless lockstep tracking sim: `ap-pnc sim track`
- Offline benchmark pipeline: `ap-pnc benchmark run`
- RViz visualization for the simple sim

**In progress**

- Web visualization: browse trajectories, tracking error and metrics in the
  browser
- Benchmark sweeps & aggregation: NMPC vs SE3 comparison table from multiple
  `benchmark run` invocations
- Flight benchmark automation: one-command takeoff → handover → log capture
  in SITL

---

## Repository Layout

```
core/                # C++/ROS research code
  ros_packages/      #   planner (core/ pure C++ + node/ ROS adapter + python/ pybind),
                     #   nmpc, interface (the message contract)
  bringup/           #   core config/launch: planning, nmpc
  docker/            #   one multiarch ROS2 base; isolated native-target dev caches
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
  `nmpc: {horizon_s, traj_res_s, ctrl_frq, initial_specific_thrust,
  cx_alpha_slope, objective, command_bounds}`

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
| Simple sim | `infra/sim_infra/simple_sim/` | Collision-free C++ lockstep tracking; planner/NMPC called synchronously, optional ROS observation |

```bash
uv run ap-pnc sim gazebo     # Gazebo layer via K3s ConfigMap
uv run ap-pnc sim simple     # simple_sim via K3s (initially paused)
uv run ap-pnc sim track      # installed simple_sim headless run, no K3s/ROS graph
uv run ap-pnc sim stop       # scale everything down
```

For the full flight procedure (takeoff → hover → NMPC handover) and the
SITL pitfalls reference, see the `ap-pnc-gazebo-e2e` skill in
`.agents/skills/`. Simple sim is a separate tracking-only pipeline, not that flight
procedure; its YAML, launch and RViz resources live in `core/bringup/`.
Details of each layer: `infra/nav_infra/README.md`,
`infra/sim_infra/gazebo/README.md`, `infra/sim_infra/simple_sim/README.md`.

---

## Generated Artifacts

Everything generated lands in a single untracked `.artifacts/` root (acados
staging, codegen output, `libnmpc_bundle.a`, pybind11 bindings, colcon
build/install/log). Python `.venv` is shared across worktrees; `.artifacts/`
(including colcon output) is worktree-local.

Zero environment variables are required — the CLI sets up every path it
needs internally.
