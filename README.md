# AP-PnC

**Documentation**: [https://warriorhanamy.github.io/AP-PnC/](https://warriorhanamy.github.io/AP-PnC/)

---

# Installations
## Python Wrapper (`pyp`)

The `pyp` script provides a native Python wrapper that automatically locates and executes the project's Python interpreter from `.acados_venv`. It works both in the main repository and in git worktrees.

### Usage

```shell
# Run Python interactively
./pyp

# Run a Python script
./pyp script.py

# Run a module
./pyp -m module_name

# Pass arguments to Python
./pyp -c "print('hello')"
```

### How It Works

1. **Git-aware**: Automatically detects the git repository root, handling both:
   - Normal repositories (where `.git` is a directory)
   - Git worktrees (where `.git` is a file pointing to the worktree metadata)

2. **Virtual environment detection**: Searches for Python in this order:
   - `.acados_venv/bin/python` (primary)
   - `.venv/bin/python`
   - `venv/bin/python`
   - `env/bin/python`

3. **Environment override**: You can override the git root detection:
   ```shell
   AP_PNC_DIR=/custom/path ./pyp
   ```

### When to Use `pyp`

Use `pyp` when you need to run Python commands outside of the `justfile` recipes:

- In CI/CD pipelines
- From other scripts
- When running Python interactively from the terminal

---

# Building and Launching ROS2 Components

**IMPORTANT**: Always use the wrapper scripts in `scripts/tools/` for building and launching ROS2 components. **DO NOT** use `colcon build` or `ros2 launch` directly.

See [docs/build-and-launch.md](docs/build-and-launch.md) for detailed information.

## Quick Reference

```bash
# Build the workspace (instead of colcon build)
scripts/tools/build              # Regular build
scripts/tools/build --clean      # Clean build

# Launch ROS2 nodes (instead of ros2 launch)
scripts/tools/pxinterface        # Launch px4_interface node
scripts/tools/plan               # Launch trajectory_server
scripts/tools/sim_run_mpc        # Run MPC simulation
```

## Workspace Behavior with Git Worktrees

This project uses different workspace strategies for Python vs C++/ROS2 components:

### Python Projects (via `./pyp`)
- **Virtual Environment**: Shared across all worktrees
- **Location**: Main repository root (e.g., `/home/rec/AP-PnC/.acados_venv`)
- **Rationale**: Python packages are large; sharing prevents duplication across worktrees

### C++/ROS2 Projects (via `scripts/tools/`)
- **Workspace**: Current worktree (where you run the command)
- **Build Artifacts**: Worktree-specific (`build/`, `install/`, `log/`)
- **Rationale**: ROS2/colcon builds produce compiled artifacts that must be worktree-isolated

**Example:**
```bash
# When in /home/rec/AP-PnC.worktree/proposed:
./pyp script.py           # Uses /home/rec/AP-PnC/.acados_venv (shared)
scripts/tools/build        # Builds in /home/rec/AP-PnC.worktree/proposed (worktree-specific)
```

## Why Use Wrappers?

The wrapper scripts automatically:
- Source the ROS2 environment (`/opt/ros/humble/setup.bash`)
- Set `AP_PNC_DIR` to the correct git repository root
- Configure `LD_LIBRARY_PATH` for ACADOS libraries
- Handle git worktrees correctly
- **Unset Python-related environments** (VIRTUAL_ENV, CONDA_PREFIX, PYTHONPATH) to ensure `colcon` uses the **system Python**

**Important**: Using the system Python for colcon build prevents issues where colcon might accidentally use a Python from another project's virtual environment.