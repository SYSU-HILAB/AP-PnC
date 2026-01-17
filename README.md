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

### Building and Running

After modifying `global.toml`:

```bash
# Build the workspace
scripts/tools/build

# Launch trajectory server with new solver configuration
scripts/tools/plan
```

## Quick Reference

```bash
# Build the workspace (instead of colcon build)
scripts/tools/build              # Regular build
scripts/tools/build --clean      # Clean build

```

### User-Facing vs. Developer Scripts

Only production-ready entry points belong under `scripts/tools`. Each file in that tree is treated as an operational wrapper that prepares the ROS2/colcon environment, so users expect to call it directly (without `./pyp`) on any workstation. Dropping ad-hoc experiments there—such as `scripts/tools/compare_solver_methods.py`, which is purely a solver comparison harness—avoids the `./pyp` bootstrap, mutates `sys.path` manually, and ships an undocumented interface next to our supported tooling. Keep developer utilities in `test/` (with pytest) or alongside the package they exercise inside `feature/`, and reserve `scripts/tools` for user-level launch/build wrappers only.

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
