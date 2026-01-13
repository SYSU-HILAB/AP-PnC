# AP-PnC Documentation

Welcome to the AP-PnC (Autonomous Parking with Predictive control) project documentation.

## Quick Links

### User Guide
- [Why We Use Wrapped Python](why-wrapped-python.md) - Deep dive into our `pyp` wrapper and modern software design

### Developer Resources
- [Control Stack & Module Reference](control-stack.md) - Calling sequence, modules, and ROS topics
- [Python Wrapper (`pyp`)](https://github.com/WarriorHanamy/AP-PnC#python-wrapper-pyp) - Quick start guide
- [Installation Guide](https://github.com/WarriorHanamy/AP-PnC) - Setup instructions

## Project Overview

AP-PnC is a modern autonomous parking framework featuring:

- 🎯 **Model Predictive Control (MPC)** with acados integration
- 🐍 **Python-first** development with modern tooling
- 🚀 **Git-aware** tooling that works in worktrees
- 🤖 **ROS2** integration for robotics applications
- 📦 **Containerized** development with Docker

## Getting Started

```bash
# Clone the repository
git clone https://github.com/WarriorHanamy/AP-PnC.git
cd AP-PnC

# Install everything (acados + Python dependencies)
just install-all

# Run Python scripts with our wrapper
./pyp your_script.py
```

## Key Features

### The `pyp` Python Wrapper

Our custom Python wrapper provides:

- ✅ Automatic virtual environment detection
- ✅ Git worktree support
- ✅ ROS2 environment setup
- ✅ Colored, helpful error messages
- ✅ Works everywhere (local, CI/CD, worktrees)

**Read more:** [Why We Use Wrapped Python](why-wrapped-python.md)

### Modern Development Workflow

```bash
# Justfile recipes
just run-simulation    # Run simulation
just test              # Run tests
just type-check        # Type checking with mypy

# Everything uses ./pyp under the hood
```

## Documentation Index

| Document | Description |
|----------|-------------|
| [Control Stack & Module Reference](control-stack.md) | Architecture, calling procedures, ROS topics |
| [Why Wrapped Python](why-wrapped-python.md) | Software design philosophy behind our approach |
| [Main README](https://github.com/WarriorHanamy/AP-PnC) | Project overview and quick start |

## Contributing

We welcome contributions! Please see our [contributor guidelines](https://github.com/WarriorHanamy/AP-PnC/blob/main/CONTRIBUTING.md) (coming soon).

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/WarriorHanamy/AP-PnC/blob/main/LICENSE) file for details.

---

**Questions?** Open an issue on [GitHub](https://github.com/WarriorHanamy/AP-PnC/issues)
