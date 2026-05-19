# AP-PnC

Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

---

## Install

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git --recurse-submodules
```

```bash
uv sync --group dev
uv run ap-pnc install
```

`uv run ap-pnc install` handles:

- acados C library build (cmake + make)
- t_renderer download
- acados_template Python interface (editable install)

---

## CLI Usage

All project commands are accessed through the unified CLI:

```bash
uv run ap-pnc --help
```

| Group      | Commands                         | Description                          |
|------------|----------------------------------|--------------------------------------|
| `install`  | (default)                        | Install everything (acados + deps)   |
| `cocp`     | (default)                        | Generate acados OCP C code           |
| `traj`     | `viz`, `viz-tracking`, `viz-elmpc`, `odom` | Trajectory visualization    |
| `aerodyn`  | `gen`, `viz`, `ml train\|pipeline`, `show`, `direct-show`, `csv-vs-model` | Aerodynamics |
| `benchmark`| `jax`, `torch`, `compare`        | Compute timing benchmarks            |
| `test`     | `all`, `vmap`, `jax-trim`, `lmpc`, `nmpc`, `sim-model`, `sim-gd`, `casadi`, `frame-conversion`, `ros-viz` | Test runners |
| `build`    | `aero`, `traj-server`, `all`     | C++ colcon builds                    |
| `nn-trim`  | `run`, `plot`                    | Neural network trim analysis         |
| `docker`   | `build`, `up`, `down`            | Docker compose commands              |
| `lint`     | (default), `fmt`                 | Ruff linter and formatter            |

### Examples

```bash
uv run cocp                    # Generate acados OCP solver C code
uv run ap-pnc traj viz             # Launch reference trajectory + RViz2
uv run ap-pnc aerodyn gen          # Generate coordinated aero data (JAX)
uv run ap-pnc aerodyn viz          # Visualize aero data
uv run ap-pnc benchmark compare    # Compare JAX/Torch/CasADi timing
uv run ap-pnc test nmpc            # Run proposed NMPC test
uv run ap-pnc build all            # Build all C++ packages
uv run ap-pnc lint                 # Lint test files
```

---

## Building and Launching ROS2 Components

Always use the wrapper scripts in `scripts/tools/` for building and launching ROS2 components. Do not use `colcon build` or `ros2 launch` directly.

See [docs/build-and-launch.md](docs/build-and-launch.md) for details.

### Quick Reference

```bash
scripts/tools/build              # Regular build
scripts/tools/build --clean      # Clean build
scripts/tools/plan               # Launch trajectory server
```

---

## Workspace Behavior with Git Worktrees

### Python Projects (via `uv run`)

- **Virtual Environment**: `.venv` at the main repository root, shared across worktrees
- **Location**: Main repository root (e.g., `/home/rec/AP-PnC/.venv`)
- **Rationale**: Python packages are large; sharing prevents duplication

### C++/ROS2 Projects (via `scripts/tools/`)

- **Workspace**: Current worktree (where you run the command)
- **Build Artifacts**: Worktree-specific (`build/`, `install/`, `log/`)
- **Rationale**: ROS2/colcon builds produce compiled artifacts that must be worktree-isolated

---

## Environment Variables

| Variable           | Purpose                    | Default                      |
|--------------------|----------------------------|------------------------------|
| `AP_PNC_DIR`         | Override git root detection | auto-detected via `git rev-parse` |
| `ACADOS_SOURCE_DIR`  | acados C source tree       | `$AP_PNC_DIR/mpc_framework`    |
| `LD_LIBRARY_PATH`    | acados runtime libraries   | `$AP_PNC_DIR/mpc_framework/lib` |
