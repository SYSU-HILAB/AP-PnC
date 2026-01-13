# Build and Launch Commands

This document explains the correct way to build and launch ROS2 components in AP-PnC.

## Important: Use Wrapped Commands

**DO NOT** use `colcon build` or `ros2 launch` directly. Always use the wrapper scripts in `scripts/tools/`.

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

### Python Development (`./pyp`)
- **Virtual Environment**: Shared across all worktrees
- **Location**: Main repository root (e.g., `/home/rec/AP-PnC/.acados_venv`)
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

# Python: Uses main repo's virtual environment (shared)
./pyp -m pytest tests/

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

### Correct Way
```bash
scripts/tools/build
```

### With Clean Build
```bash
scripts/tools/build --clean
```

### Build Specific Packages
```bash
scripts/tools/build --packages-select px4_interface
```

The wrapper builds only the necessary packages by default:
- `trajectory_server`
- `px4_interface`
- `c_nmpc`
- `nmpc`
- `tailsitter_df`
- `utils`

## Launching ROS2 Nodes

### Correct Way
Use the wrapper scripts in `scripts/tools/`:

| Wrapper | Launches | Description |
|---------|----------|-------------|
| `scripts/tools/pxinterface` | `px4_interface` | PX4 interface node |
| `scripts/tools/plan` | `trajectory_server` | Trajectory planning server |

### Example
```bash
# Instead of:
# ros2 launch px4_interface px4_interface.launch.py

# Use:
scripts/tools/pxinterface
```

## Available Wrapper Scripts

```bash
# Build
scripts/tools/build [--clean]

# Launch PX4 interface
scripts/tools/pxinterface

# Launch trajectory server
scripts/tools/plan

# Run MPC simulation
scripts/tools/sim_run_mpc

# Run MPC on real hardware
scripts/tools/real_run_mpc
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
# Make sure you're using the wrapper, not colcon directly
scripts/tools/build --clean
```

### Launch Fails with "node not found"
```bash
# First build the workspace
scripts/tools/build

# Then source the workspace
source install/setup.bash

# Then use the wrapper to launch
scripts/tools/pxinterface
```

### Library Not Found Errors
The wrapper scripts automatically configure `LD_LIBRARY_PATH`. If you still have issues:
```bash
echo $LD_LIBRARY_PATH | grep mpc_framework
# Should show the ACADOS library path
```
