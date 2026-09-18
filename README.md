# AP-PnC

Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

---

## Install

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git
uv sync --group dev --group extra
uv run ap-pnc install
```

No recursive clone needed: `ap-pnc install` fetches acados automatically.

`uv run ap-pnc install` handles:

- acados source fetch into `.artifacts/acados_src` (commit-locked)
- acados C library build, staged into `.artifacts/acados` (shared + static)
- t_renderer download
- acados_template Python interface (editable install)

For C++ message builds, init the only remaining submodule when needed:
The vendored dependencies (`ap_pnc/sciplots`, `autodiff`, `ROS2-Utils`,
`tomlplusplus`, `yamlcpp`, `BSplinebasis`) are git subtrees and live directly
in the repo. `uv run ap-pnc gen-nmpc-lib` runs all of the above on demand
when prerequisites are missing.

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
| `build`        | `all`, `cnmpc`, `aero`, `simple-sim`, `traj-server`, `bindings` | C++ colcon builds |
| `sim`          | `simple`, `gazebo`, `stop`, `status`, `logs`, `pcd` | K3s ConfigMap-driven simulation |
| `stack`        | `start`, `trigger pos_est\|go` | Local ROS2 stack lifecycle |
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
uv run ap-pnc build all            # Build the whole workspace
uv run ap-pnc lint                 # Lint
```

---

## Repository Layout

```
ros_packages/        # Core ROS2 C++ stack
  planner/           #   trajectory generation: tailsitter_df (DF phi-theory
                     #   kernel), basic_trajectories, trajectory_server, traj_buffer
  c_nmpc/            #   NMPC tracking node + nmpc_wrapper
  px4_interface/     #   PX4 bridge
  utils/msgs/        #   interface package (shared ROS msg contract, no px4_msgs)
  utils/             #   ROS2-Utils + vendored libraries
bringup/             # Top-layer package: config/ + launch/ (single entry)
ap_pnc/              # Python package: CLI, nmpc/ codegen toolchain, lu_mpc, aerodynamics
simulations/         # aerodynamics (ground truth), gazebo, simple_sim, k3s
benchmark/           # Prior-based experiment configs (Lyu force-balance)
docker/              # amd64/arm64 ros2-base images
```

## Runtime Configuration

No ROS parameters. Nodes read sectioned yaml directly from `bringup/config/`:

- `planning.yaml` — `problem_formulation` (box, basic_traj, nested cost,
  MINCO, prior-free phi-theory flatness) + `io` (topics, dump dirs)
- `c_nmpc.yaml` — `nmpc: {mass, cz_rate}`

Nodes read the conventional cwd-relative `bringup/config/*.yaml` (matches
the container `/workspace` mount). See
[docs/build-and-launch.md](docs/build-and-launch.md).

---

## Simulation Layers

Two stacked simulation setups live under `simulations/`:

| Layer | Directory | What it runs |
|-------|-----------|--------------|
| Gazebo | `simulations/gazebo/` | Generic-interface SITL: stock PX4 v1.17.0 under Gazebo Harmonic (`gz_x500`), bridged by mavros2 to the AP-PnC NMPC stack (`px4-simulator` + `nav_infra` + `qground_control` + `ap-pnc`) |
| Simple sim | `simulations/simple_sim/` | Lightweight C++ simulator (MARSIM-style) with synthetic LiDAR sensing, for obstacle-avoidance principle verification |

```bash
uv run ap-pnc sim gazebo     # Gazebo layer (stock PX4 v1.17.0 + Gazebo Harmonic + mavros2)
uv run ap-pnc sim simple     # simple_sim layer (lightweight LiDAR sim)
uv run ap-pnc sim stop       # scale everything down
```

The gazebo layer runs stock upstream PX4 (no fork), pinned to a release tag
via `ARG PX4_VERSION` in the image. The single `nav_infra` container provides
the base services (mavros2 + NMPC-mavros bridge + setpoint mux; livox +
fast_lio + ekf_quat join on the real machine — same image, different bringup
profile), while the `ap-pnc` core container carries only research code
(planner + c_nmpc + px4ctrl). See `nav_infra/README.md`,
`simulations/gazebo/README.md` and `simulations/simple_sim/README.md`.

---

## Generated Artifacts and Git Worktrees

Everything generated lands in a single untracked `.artifacts/` root (acados
staging, codegen output, `libnmpc_bundle.a`, pybind11 bindings, colcon
build/install/log). Python `.venv` is shared across worktrees; `.artifacts/`
(including colcon output) is worktree-local.

Zero environment variables are required — the CLI sets up every path it
needs internally.
