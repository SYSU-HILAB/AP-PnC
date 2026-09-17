# Build and Launch Commands

This document explains how to build and launch ROS2 components in AP-PnC.
All commands go through the single `uv run ap-pnc` CLI entry point — the legacy
`scripts/tools/*` wrappers and `.env` are gone.

## Build

```bash
uv run ap-pnc build all          # whole workspace (all discovered packages)
uv run ap-pnc build cnmpc        # c_nmpc + px4_interface only
uv run ap-pnc build aero         # aerodynamics only
uv run ap-pnc build simple-sim   # simple_sim only
uv run ap-pnc build traj-server  # benchmark trajectory_server
uv run ap-pnc build bindings     # aerodynamics pybind11 bindings
uv run ap-pnc build all --clean  # wipe .artifacts/colcon first
```

Colcon output lives under `.artifacts/colcon/{build,install,log}` (merge-install
layout). The build environment is prepared automatically: ROS2 is sourced,
virtual-env/conda variables are cleared so colcon uses the system Python, and
`CMAKE_PREFIX_PATH` points at the workspace install so in-workspace packages
find each other.

## Launch

### Simulation (K3s-managed, containers)

```bash
uv run ap-pnc sim simple        # lightweight LiDAR + aero simulator
uv run ap-pnc sim gazebo        # PX4-VTOL SITL + Gazebo
uv run ap-pnc sim stop          # scale everything to zero
uv run ap-pnc sim status        # ConfigMap selection + pod state
uv run ap-pnc sim logs          # follow the active simulator
```

### Local ROS2 stack (bare metal)

```bash
uv run ap-pnc stack start             # trajectory_server + px4_interface + c_nmpc, core-pinned
uv run ap-pnc stack trigger pos_est   # publish /est_cmd
uv run ap-pnc stack trigger go        # publish /mode
```

`stack start` sources `/opt/ros/humble` and `.artifacts/colcon/install`
automatically, pins the three nodes to the last three CPU cores (mirroring the
legacy `runall`), and cleans everything up on Ctrl-C.

### Planning benchmark experiment

The one-shot experiment system: pick a velocity, sync the planner config, start
the full stack.

```bash
uv run ap-pnc benchmark run --vmax 5 --type 1 --radius 4        # config + stack start
uv run ap-pnc benchmark run --vmax 6.5 --type 2 --no-launch     # config only
```

Trajectory types: 1=Lemniscate, 2=Circle, 3=Sinusoidal, 4=Line.

## Workspace Strategy: Python vs C++/ROS2

- **Python** (`uv run`): the `.venv` is shared across worktrees
  (`UV_PROJECT_ENVIRONMENT` or a symlinked `.venv`), because Python deps are
  large and branch-independent.
- **C++/ROS2**: `.artifacts/colcon` is worktree-local. Each worktree gets its
  own build/install/log under its own `.artifacts/`.

## Debugging

```bash
uv run ap-pnc debug echo                  # fzf-pick a topic, ros2 topic echo
uv run ap-pnc debug pcd frame.pcd         # open3d viewer for simple_sim dumps
uv run ap-pnc debug pcd frame.pcd --info  # stats only, no window
```

## Troubleshooting

- **Package not found during build** → `uv run ap-pnc build all --clean`.
- **Node not found at launch** → build first; `stack start` needs
  `.artifacts/colcon/install/setup.bash` to exist.
- **acados library errors** → `LD_LIBRARY_PATH` is set by the CLI to
  `.artifacts/acados/lib`; check `uv run ap-pnc install`.
