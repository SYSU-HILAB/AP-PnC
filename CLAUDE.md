# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Vibe Coding Rules & Project Structure

You are an expert AI assistant helping with a fast-paced "Vibe Coding" project.
Your goal is to maintain momentum while keeping the codebase disposable and modular.

### 1. Branching Strategy & MVP
- Focus on **Minimum Viable Product (MVP)** functionality first. Do not over-engineer.
- Assume the user is working on a dedicated **Feature Branch**.
- If the current task involves a new distinct capability, treat it as a temporary experiment.

### 2. Directory Structure & Isolation
- **Root Feature Folder:** All new, non-trivial features MUST be located in `<project_root>/feature/`.
- **Self-Contained:** When implementing a new feature (e.g., "drone-control"), create a directory `feature/drone-control/`.
    - Place ALL related files (Logic, UI, Styles, Tests, Assets) inside this specific folder.
    - Do NOT scatter code across global `src/components` or `src/utils` unless it is strictly a generic utility used by 3+ features.
- **Colocation:** Keep things together. If a file is only used by this feature, it belongs in the feature folder.

###  3. Coding Style
- Prefer **Self-contained** logic over DRY (Don't Repeat Yourself) if it reduces coupling.
- It is better to duplicate a small utility function inside `feature/<name>/utils.cpp` than to modify a global shared file and risk breaking other features.
- Use English for all variable names, comments, and technical terminology.

### 4. Deletability
- Write code knowing it might be deleted tomorrow. Avoid tight coupling with the core system.

## Development
Always using ${AP_PNC_DIR}/.acados.venv/bin/python to develop python code, and system development tools to develop cpp project. The docker is for user, not for development.

## Project Overview

AP-PnC (Adaptive Planning and Control) is a ROS2-based framework for autonomous control of tailsitter UAVs. The system implements differential flatness-based trajectory generation combined with real-time model predictive control using the ACADOS framework.

## System Architecture

### Core Components

1. **Trajectory Server** (`src/trajectory_server`):
   - GCOPTER-based trajectory optimization using differential flatness
   - Generates smooth, collision-free trajectories
   - Publishes reference trajectories at ~50Hz

2. **NMPC Controller** (`src/c_nmpc`):
   - Real-time nonlinear model predictive control using ACADOS
   - High-frequency control loop (100Hz+)
   - Tracks generated trajectories with optimal control inputs

3. **Differential Flatness** (`src/tailsitter_df`):
   - Mathematical framework for tailsitter dynamics
   - Forward/backward transformations between flat outputs and full states
   - Handles singularities in low-velocity regimes

4. **Aerodynamics** (`src/aerodynamics`):
   - Multiple aerodynamic models (Phi-based, B-spline, Zhang-Lyu)
   - MATLAB/Simulink generated code integration
   - Factory pattern for model selection

5. **PX4 Interface** (`src/px4_interface`):
   - Communication layer between ROS2 and PX4 autopilot
   - MAVLink protocol handling and offboard control

### Control Flow
```
High-Level Planning → Trajectory Generation → MPC Control → PX4 → UAV
        ↑                                              ↓
Environmental Sensing ← State Estimation ← Sensor Data
```

## Package Dependencies

### ROS2 Packages with Custom Messages:
- **interface**: Custom message definitions (`State`, `RefTrajectory`, `TrajectoryPoint`, `Controls`)
- **px4_msgs**: PX4-specific message definitions

### Core Libraries:
- **ACADOS**: High-performance nonlinear MPC solver
- **Eigen**: Linear algebra operations
- **AutoDiff**: Automatic differentiation
- **GCOPTER**: Trajectory optimization framework (heavily modified)
- **TOML++**: Configuration file parsing
- **YAML-CPP**: YAML configuration handling

## Configuration Management

### Global Configuration
- `global.toml`: System-wide parameters and aerodynamic coefficients
- Node-specific YAML configs in each package directory
- ROS2 parameter system for runtime configuration

### Key Configuration Areas:
- Aerodynamic model selection and parameters
- MPC weights and constraints
- Trajectory generation parameters
- Safety limits and bounds

## Development Patterns

### Code Organization
- **Header-Source Separation**: Clear interface definitions in include/ folders
- **Template-based Design**: Used for trajectory classes and generic algorithms
- **Factory Pattern**: For aerodynamic model selection in `aerodynamics/`
- **Singleton Pattern**: For global configuration and state management

### External Libraries
All external libraries are managed as git submodules in `src/external_libraries/`:
- **BSplinebasis**: B-spline basis functions (test module removed)
- **ScopeProfiler**: Performance profiling utilities
- **CSVWriter**: Data export utilities

## Build System

### ROS2 Workspace Structure
```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select trajectory_server

# Build withsymlinks for development
colcon build --symlink-install --packages-select c_nmpc
```

### Container Development
When developing in containers, source volume mounts provide live code updates:
- `src/tailsitter_df:/ap-pnc/src/tailsitter_df`
- `src/trajectory_server:/ap-pnc/src/trajectory_server`
- `scripts/tools:/ap-pnc/scripts/tools`
- `temp_data:/ap-pnc/temp_data`

## Testing and Debugging

### Common Development Tasks
1. **Trajectory Generation Testing**: Modify `src/trajectory_server/config/` and restart container
2. **MPC Parameter Tuning**: Adjust weights in `src/c_nmpc/config/`
3. **Aerodynamic Model Development**: Add new models to `src/aerodynamics/src/models/`
4. **Visualization**: Use RViz2 to inspect trajectories and system state

### Debugging Tools
- **ROS2 CLI**: `ros2 topic echo`, `ros2 topic hz`, `ros2 param list`
- **RViz2**: Visualize trajectories, UAV state, and control inputs
- **QGroundControl**: Monitor UAV hardware and telemetry
- **PX4 SITL**: Software-in-the-loop simulation

## Important Notes

### X11 Forwarding Setup
Add this line to your `~/.bashrc` or `~/.profile`:
```bash
echo "xhost +local: >/dev/null" >> ~/.bashrc
```

### Multi-rate Control Architecture
The system uses different control frequencies:
- **Trajectory Generation**: ~50Hz
- **MPC Control**: 100Hz+
- **Hardware Interface**: As fast as hardware allows

This architecture ensures smooth trajectory execution while maintaining computational efficiency.
