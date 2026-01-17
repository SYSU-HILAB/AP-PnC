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

## Installing Rust (for `just` command runner)

`just` is built with Rust and requires Cargo to install. If you already have Rust and Cargo installed, skip to the next section.

### Install Rust via rustup (Recommended)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

This will:
- Install Rust toolchain (rustc, cargo)
- Configure your PATH environment variable
- Add Rust to your shell profile

After installation, restart your terminal or run:
```bash
source $HOME/.cargo/env
```

### Verify Installation

```bash
cargo --version
rustc --version
```

Expected output: version numbers for both tools.

## Installing `just` (Command Runner)

The `just` command runner is used throughout this project for task automation. It's optional but highly recommended.

### Option 1: Install via Cargo (Recommended)

If you have Rust and Cargo installed:

```bash
cargo install just
```

### Option 2: Install from Pre-built Binary

Download and install the latest release:

```bash
# Download latest binary (adjust version if needed)
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin
```

### Option 3: Install via Package Manager

**Ubuntu/Debian:**
```bash
# Add just's PPA (if available) or use cargo
sudo apt install cargo  # if not already installed
cargo install just
```

**Fedora:**
```bash
sudo dnf install just
```

**Arch Linux:**
```bash
sudo pacman -S just
```

### Verify Installation

```bash
just --version
```

**Alternative**: If you prefer not to install `just`, all commands can be run directly (e.g., `./installations/install-all.sh` instead of `just install-all`).

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/WarriorHanamy/AP-PnC.git
cd AP-PnC
```

### 2. Install Dependencies

The project uses ACADOS (a C++ optimization framework) and Python packages. The `just install-all` command handles everything:

```bash
just install-all
```

This will:
- Create a virtual environment at `.acados_venv/`
- Build ACADOS from source in `mpc_framework/`
- Install Python dependencies (numpy, pandas, acados_template)

**Note**: If you don't have `just` installed, run the installation script directly:
```bash
./installations/install-all.sh
```

### 3. Verify Installation

Check that everything is set up correctly:

```bash
just status
```

Expected output:
```
Git repository root: /path/to/AP-PnC
Python environment: .acados_venv
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
just viz-traj-tracking
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
just install-all

# 2. Build (after code changes)
./scripts/tools/build

# 3. Run system
./scripts/tools/srunall

# 4. Visualize (in new terminal)
source /opt/ros/humble/setup.bash
just viz-traj-tracking
```

## Troubleshooting

### `colcon: command not found`

Install ROS2 build tools:
```bash
sudo apt install python3-colcon-common-extensions
```

### `just: command not found`

Either install `just` or run scripts directly:
```bash
# Install just (recommended)
cargo install just

# Or run directly
./installations/install-all.sh
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
# Prerequisites Installation
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh   # Install Rust (for just)
source $HOME/.cargo/env                                        # Load Rust environment
cargo install just                                              # Install just command runner

# Installation
just install-all          # Install everything
just install-venv          # Create virtual environment only
just status                # Check environment status

# Building
./scripts/tools/build      # Build ROS2 workspace
./scripts/tools/build --clean    # Clean rebuild

# Running
./scripts/tools/srunall     # Launch complete system

# Visualization
just viz-traj-tracking      # Visualize the sim-and-ctrl playground
./pyp -m feature.lu_mpc.ros.simulation_visualizer    # Direct Python launch

# Python execution
./pyp script.py            # Run Python script with correct env
./pyp -m module_name       # Run Python module
```
