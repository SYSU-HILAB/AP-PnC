# AP-PnC Documentation

Welcome to the AP-PnC (Aerodynamic Prior-Free Trajectory Planning and Tracking Control for a Tail-sitter UAV) project documentation.

**New to AP-PnC?** [Start with the Quick Start Guide](quickstart.md) to get up and running in minutes.

## Quick Links

### User Guide
- [Why We Use Wrapped Python](why-wrapped-python.md) - Deep dive into our `pyp` wrapper (legacy) and modern software design

### Developer Resources
- [Quick Start Guide](quickstart.md) - Get AP-PnC running in minutes
- [Control Stack & Module Reference](control-stack.md) - Calling sequence, modules, and ROS topics
- [Build and Launch Commands](build-and-launch.md) - Correct way to build and launch ROS2 components
- [Test the Lu et al.(TIE '23, IJRR '24, TRO '25)](viz-traj-tracking-elmpc) - An acados implementation of this so-called on-manifold mpc.

## Project Overview

AP-PnC is a modern autonomous parking framework featuring:

- 🎯 **Model Predictive Control (MPC)** with acados integration
- 🐍 **Python-first** development with modern tooling
- 🚀 **Git-aware** tooling that works in worktrees
- 🤖 **ROS2** integration for robotics applications
- 📦 **Containerized** development with Docker

## Getting Started

For complete installation and setup instructions, see the **[Quick Start Guide](quickstart.md)**.

Quick reference:

```bash
# Clone and install
git clone https://github.com/WarriorHanamy/AP-PnC.git
cd AP-PnC
just install-all

# Build ROS2 workspace
./scripts/tools/build

# Run the system
./scripts/tools/srunall

# Visualize (in new terminal)
source /opt/ros/humble/setup.bash
just viz-traj-tracking
```

## Key Features

### Unified Python Execution via `uv run`

The project uses `uv` as the unified Python runner, replacing the legacy `pyp` wrapper:

- ✅ Automatic virtual environment detection (`.venv`)
- ✅ Git worktree support
- ✅ ACADOS library path setup via `ap_pnc.env.setup_env()`
- ✅ Consistent environment across local, CI/CD, and worktrees

**Read more:** [Why We Use Wrapped Python](why-wrapped-python.md)

### Modern Development Workflow

```bash
# uv-based recipes
uv run ap-pnc <group> <command>   # Run CLI commands
uv run python script.py           # Run Python scripts
uv run pytest tests/              # Run tests
uv run ruff check                 # Lint
```

## Documentation Index

| Document | Description |
|----------|-------------|
| [Quick Start Guide](quickstart.md) | Get AP-PnC running in minutes |
| [Build and Launch Commands](build-and-launch.md) | Correct way to build and launch ROS2 components |
| [Control Stack & Module Reference](control-stack.md) | Architecture, calling procedures, ROS topics |
| [Why Wrapped Python](why-wrapped-python.md) | Software design philosophy (historical perspective) |

## Contributing

We welcome contributions! Please see our [contributor guidelines](https://github.com/WarriorHanamy/AP-PnC/blob/main/CONTRIBUTING.md) (coming soon).

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/WarriorHanamy/AP-PnC/blob/main/LICENSE) file for details.

---

**Questions?** Open an issue on [GitHub](https://github.com/WarriorHanamy/AP-PnC/issues)
