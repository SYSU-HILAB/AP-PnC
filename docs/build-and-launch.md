# Build and Launch Commands

This document explains the correct way to build and launch ROS2 components in AP-PnC.

## Build Commands

Use `uv run ap-pnc build` for Python-managed builds, or `scripts/tools/build` directly for manual colcon invocation.

## Why Wrappers Are Needed

The wrapper scripts source `.env_setup.sh` which:
1. Sets up the ROS2 environment (`source /opt/ros/humble/setup.bash`)
2. Sets `AP_PNC_DIR` to the correct git repository root
3. Sets workspace to the current worktree root for proper build isolation
4. Configures the `LD_LIBRARY_PATH` for ACADOS libraries
5. **Unsets Python-related environments** (VIRTUAL_ENV, CONDA_PREFIX, PYTHONPATH, etc.) to ensure `colcon` uses the **system Python**

**Important**: The build wrapper ensures colcon uses the system Python by clearing virtual environment and conda environment variables. This prevents issues where colcon might accidentally use a Python from a different project's virtual environment.

## Workspace Strategy: Python vs C++/ROS2

This project uses different workspace strategies for Python and C++/ROS2 development:

### Python Development (`uv run`)
- **Virtual Environment**: Shared across worktrees (via `UV_PROJECT_ENVIRONMENT` or symlinked `.venv`)
- **Why**: Python packages are large (GBs) and branch-independent; sharing prevents duplication

### C++/ROS2 Development (`scripts/tools/`)
- **Workspace**: Current worktree (where you run the command)
- **Build Artifacts**: Worktree-specific (`build/`, `install/`, `log/`)
- **Why**: Colcon builds produce compiled artifacts that must be isolated per worktree

**Example workflow in a worktree:**
```bash
# Create and enter a worktree
git worktree add ../AP-PnC.worktree/proposed proposed
cd ../AP-PnC.worktree/proposed

# Python: Uses project's virtual environment (shared)
uv run pytest tests/

# C++/ROS2: Uses current worktree as workspace (isolated)
scripts/tools/build              # Builds in the worktree
scripts/tools/test --packages-select aerodynamics
```

### Key Principle

When working with git worktrees:
- **Python code** runs from the shared main repository virtual environment
- **ROS2/C++ builds** run in the current worktree with isolated build artifacts

This design ensures efficient resource usage while maintaining proper build isolation.

## Building

### Via CLI (recommended)
```bash
uv run ap-pnc build              # Build all packages
uv run ap-pnc build aero         # Build aerodynamics only
uv run ap-pnc build traj-server  # Build trajectory_server only
uv run ap-pnc build --clean      # Clean all build artifacts first
```

### Using Shell Scripts Directly
```bash
scripts/tools/build
scripts/tools/build --clean
scripts/tools/build --packages-select px4_interface
```

## Launching ROS2 Nodes

### Correct Way
Use the wrapper scripts in `scripts/tools/`:

| Wrapper | Launches | Description |
|---------|----------|-------------|
| `scripts/tools/plan` | `trajectory_server` | Trajectory planning server |

### Example
```bash
# Launch trajectory server
scripts/tools/plan

# For PX4 interface, use runall or launch directly:
scripts/tools/runall  # Launches full system (px4_interface + trajectory_server + MPC)
# Or:
ros2 launch px4_interface px4_interface.launch.py
```

## Available Wrapper Scripts

```bash
# Build
scripts/tools/build [--clean]

# Launch trajectory server
scripts/tools/plan

# Launch full system (px4_interface + trajectory_server + MPC)
scripts/tools/runall
```

## Environment Setup

All wrapper scripts automatically source `.env_setup.sh` which configures:
- `AP_PNC_DIR`: Points to the git repository root
- `LD_LIBRARY_PATH`: Includes ACADOS libraries
- `PYTHONPATH`: Includes the project root
- ROS2 environment from `/opt/ros/humble/setup.bash`

## What `.env_setup.sh` Does

The `.env_setup.sh` script:
1. Detects the git repository root (handles both normal repos and worktrees)
2. Sources the ROS2 environment
3. Sets `AP_PNC_DIR` environment variable
4. Configures library paths for ACADOS
5. Provides colored output functions for scripts

## Troubleshooting

### Build Fails with "package not found"
```bash
# Try clean build via CLI or scripts
uv run ap-pnc build --clean
# or
./scripts/tools/build --clean
```

### Launch Fails with "node not found"
```bash
# First build the workspace
uv run ap-pnc build --clean
# or
./scripts/tools/build

# Use runall for the full system
./scripts/tools/runall
```

### Library Not Found Errors
The wrapper scripts automatically configure `LD_LIBRARY_PATH`. If you still have issues:
```bash
echo $LD_LIBRARY_PATH | grep mpc_framework
# Should show the ACADOS library path
```

## Visualization Nodes

The project provides two visualization nodes for trajectory tracking in RViz2:

### Simulation Visualizer

Runs the full NMPC controller and CasADi simulator for closed-loop visualization:

```bash
# Using Python directly
uv run python -m feature.lu_mpc.ros.simulation_visualizer

# Using the helper
uv run python -m feature.utils.ros_visualizer

# Using the user-facing launcher
uv run python tests/lu_mpc/ros/user_test_traj_tracking_viz.py
```

**Topics Published:**
- `/nmpc/odometry`: Vehicle state from simulator
- `/nmpc/reference_path`: Reference trajectory from CSV
- `/nmpc/tracking_path`: Actual tracking path
- `/nmpc/motor_thrusts`: Motor commands
- `/nmpc/aero_forces`: Aerodynamic forces

**Use case:** Closed-loop simulation with controller performance evaluation

### Reference Visualizer

Lightweight CSV replay without controller/simulator:

```bash
# Using Python directly
uv run python -m feature.lu_mpc.ros.reference_visualizer

# Using the user-facing launcher
uv run python tests/lu_mpc/ros/user_test_ref_traj_viz.py
```

**Topics Published:**
- `/nmpc/reference_path`: Full reference trajectory from CSV (static)
- `/nmpc/odometry`: Reference state replayed at playback_dt
- `/nmpc/tracking_path`: Rolling buffer of replayed reference states

**Use case:** Quick visualization of planned trajectories without simulation overhead

### Quaternion Format

Both visualizers use the **WXYZ** quaternion format `[w, x, y, z]` where w is the scalar part. This matches:
- The CSV trajectory loader format
- The mathematical convention in `ap_pnc/utils/math_utils.py`
- ROS2 standard Quaternion message ordering

### Architecture Notes

The visualization nodes use shared utilities from `ap_pnc/utils/ros_visualizer` to avoid code duplication:
- ROS message conversion helpers (NumPy → ROS messages)
- Path creation and management utilities
- QoS profile factory functions
- Topic naming utilities

This follows the **DRY principle** and ensures consistent behavior across all visualization components.
