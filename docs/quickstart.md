# Quick Start Guide

**AP-PnC**: Aerodynamic Prior-Free Trajectory Planning and Tracking Control for a Tail-sitter UAV

This guide will get you from zero to running the AP-PnC trajectory tracking visualization in minutes.

## Prerequisites

Before starting, ensure you have:

- **Operating System**: Ubuntu 22.04 (or compatible Linux distribution)
- **ROS2**: Humble distribution installed and sourced
- **Hardware**: 4, 8, or 16 CPU cores (for `srunall` core-aware process distribution)
- **Development Tools**:
  - `colcon` (`sudo apt install python3-colcon-common-extensions`)
  - `git`
  - Python 3.10+

## Installing `uv` (Fast Python Package and Project Manager)

This project uses `uv` for managing dependencies, Python environments, and running CLI commands.

### Install `uv`

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Or via package manager:

```bash
# macOS / Linux (Homebrew)
brew install uv

# Arch Linux
sudo pacman -S uv
```

Verify installation:

```bash
uv --version
```

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git
cd AP-PnC
```

### 2. Install Dependencies

The project uses ACADOS (a C++ optimization framework) and Python packages. Installation is handled via `uv`:

```bash
uv run ap-pnc install
```

This will:
- Sync the virtual environment at `.venv/`
- Build ACADOS from source in `mpc_framework/`
- Install Python dependencies (numpy, typer, rich, acados_template)

### 3. Verify Installation

Check that everything is set up correctly:

```bash
uv run ap-pnc install  # (re-runs if any deps are missing)
```

## Build the ROS2 Workspace

The C++ and ROS2 components need to be compiled. Use the provided wrapper script:

```bash
./scripts/tools/build
```

This builds all required ROS2 packages for trajectory tracking and control.

**Expected output**: Colcon build summary showing all packages built successfully.

### Clean Build (if needed)

If you encounter build errors, try a clean build:

```bash
./scripts/tools/build --clean
```

## Run the System

The `srunall` script launches the complete system with components distributed across CPU cores:

```bash
./scripts/tools/srunall
```

**Expected output**: Process IDs (PIDs) displayed, indicating all components are running.

### Stopping the System

Press `CTRL-C` to gracefully shut down all components.

## Visualize Trajectory Tracking

Now open a new terminal (keep `srunall` running) and launch the visualization:

```bash
# Source ROS2 first
source /opt/ros/humble/setup.bash

# Launch trajectory tracking visualization
uv run ap-pnc traj viz-tracking
```

### What You'll See

- **RViz2 window** with 3D trajectory visualization
- **Green path**: Reference trajectory (planned path)
- **Red path**: Actual tracking path (vehicle position over time)
- **Coordinate frame**: Vehicle orientation
- **Motor thrusts**: Real-time motor commands

### Keyboard Controls in RViz2

- Mouse drag: Rotate view
- Scroll wheel: Zoom in/out
- Shift + drag: Pan view

## Complete Workflow Summary

```bash
# 1. Install (one-time setup)
uv sync --group dev
uv run ap-pnc install
uv run ap-pnc cocp

# 2. Build (after code changes)
./scripts/tools/build

# 3. Run system
./scripts/tools/srunall

# 4. Visualize (in new terminal)
source /opt/ros/humble/setup.bash
uv run ap-pnc traj viz-tracking
```

## Troubleshooting

### `colcon: command not found`

Install ROS2 build tools:
```bash
sudo apt install python3-colcon-common-extensions
```

### `uv: command not found`

Install `uv`:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Build fails with "package not found"

Ensure you're using the wrapper script (not `colcon` directly):
```bash
./scripts/tools/build --clean
```

### Library not found errors

The wrapper scripts automatically configure `LD_LIBRARY_PATH`. If issues persist:
```bash
echo $LD_LIBRARY_PATH | grep mpc_framework
# Should display the ACADOS library path
```

### RViz2 window doesn't appear

1. Ensure `srunall` is still running
2. Check that ROS2 is sourced: `source /opt/ros/humble/setup.bash`
3. Verify the workspace is built: `./scripts/tools/build`

### Wrong number of CPU cores for srunall

The `srunall` script requires 4, 8, or 16 CPU cores. Check your core count:
```bash
nproc
```

If you have a different number of cores, you can edit the script or launch components manually using the individual wrapper scripts in `scripts/tools/`.

## Next Steps

- **[Control Stack & Module Reference](control-stack.md)** - Detailed architecture and ROS topics
- **[Why We Use Wrapped Python](why-wrapped-python.md)** - Understanding our development philosophy
- **[Build and Launch Commands](build-and-launch.md)** - Deep dive into wrapper scripts

## Common Commands Reference

```bash
# Installation
uv run ap-pnc install     # Install everything (venv, acados, deps)
uv sync --group dev        # Sync dev dependencies

# Building
./scripts/tools/build      # Build ROS2 workspace
./scripts/tools/build --clean    # Clean rebuild

# Running
./scripts/tools/srunall     # Launch complete system

# Visualization
uv run python -m feature.lu_mpc.ros.simulation_visualizer    # Direct Python launch

# Python execution
uv run python script.py            # Run Python script with correct env
uv run python -m module_name       # Run Python module
```
