# AP-PnC Architecture Overview

**Last Updated**: 2025-01-25
**Project**: Autonomous Parking with Predictive Control (AP-PnC)

## System Overview

AP-PnC is a hybrid C++/Python framework for autonomous parking and trajectory tracking of tailsitter UAVs. The system combines:

- **Non-linear MPC (NMPC)**: Real-time optimization via ACADOS
- **Multi-backend Python**: CasADi, JAX, PyTorch support
- **ROS2 Integration**: Humble distribution for robotics middleware
- **Aerodynamic Modeling**: Multiple solver methods with adaptive optimization

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Layer                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Trajectory   │  │   PX4        │  │    MPC       │         │
│  │   Server     │──│  Interface   │──│  Controller  │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Control Layer                              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                  ACADOS NMPC                             │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐        │  │
│  │  │   QP       │  │   HPIPM    │  │  qpOASES   │        │  │
│  │  │  Solvers   │  │   Solver   │  │  Solver    │        │  │
│  │  └────────────┘  └────────────┘  └────────────┘        │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Model Layer                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Python Multi-Backend                        │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐                  │  │
│  │  │ CasADi  │  │   JAX   │  │ PyTorch │                  │  │
│  │  └─────────┘  └─────────┘  └─────────┘                  │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Aerodynamic Models                          │  │
│  │  Zhang-Lyu | BSpline | Phi | Advanced Lift/Drag         │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Hardware Layer                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  PX4 Flight  │  │  Motors/     │  │   Sensors    │         │
│  │ Controller   │  │  Actuators   │  │              │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
AP-PnC/
├── src/                        # C++ backend implementation
│   ├── aerodynamics/           # Aerodynamic solvers (lookup, Newton-Raphson)
│   ├── trajectory_server/      # Trajectory planning and generation
│   ├── px4_interface/          # PX4 SITL ROS2 node
│   ├── c_nmpc/                 # C++ NMPC ACADOS implementation
│   ├── tailsitter_df/          # Tailsitter dynamics model
│   ├── basic_trajectories/     # Trajectory primitives
│   ├── utils/                  # Utilities (config, CSV, profiling)
│   └── external_libraries/     # Third-party dependencies
│
├── ap_pnc/                    # Main Python package (CLI + library)
│   ├── aerodynamics/          # MLP models, data, training, inference
│   ├── lu_mpc/                # MPC controller, model, plotting
│   ├── utils/                 # utilities (color_print, math_utils, etc.)
│   ├── sciplots/              # vendored scientific plotting library
│   ├── paper_plots/           # paper figure generation scripts
│   └── recipes/               # CLI command implementations
│
├── mpc_framework/             # ACADOS framework
├── scripts/tools/             # Shell wrappers for colcon build, launch, env setup
├── docs/                      # Documentation
├── pyproject.toml             # uv project config
└── .venv/                     # uv-managed virtual environment
```

## Key Architectural Patterns

### 1. Multi-Backend Pattern
Same aerodynamic models can be evaluated with different backends:
- **CasADi**: Symbolic optimization, primary for ACADOS
- **JAX**: JIT compilation, automatic vectorization (`vmap`)
- **PyTorch**: Neural network integration, GPU acceleration

### 2. Layered Control Architecture
```
Reference Trajectory → Global Planner → Local MPC → Motor Commands
                      (Trajectory       (NMPC)         (PX4)
                       Server)
```

### 3. Factory Pattern
- `get_model()` in `ap_pnc/lu_mpc/model/sim/`: Backend-agnostic model creation
- `CzCalculate` in `src/aerodynamics/`: Adaptive solver selection

### 4. Cache Strategy
- Aerodynamic data: `ap_pnc/lu_mpc/.cache/mesh_aero_data*.npy`
- Lookup tables: `artifacts/simulations/aero_lookup_table.csv`

## State Space (18D)

```
State Vector: [x, y, z, vx, vy, vz, phi, theta, psi,
               qw, qx, qy, qz, m1, m2, m3, m4, thrust_rate]
               └─────────────────┘ └─────────────────┘
                 Position/Rotation      Motor Commands
```

## Key Flows

### Trajectory Planning Flow
```
1. global.toml → Trajectory Server configuration
2. Trajectory Server → MINCO/LBFGS trajectory generation
3. CzCalculate → Angle of attack from lift coefficient
4. Output → Reference trajectory for MPC
```

### Control Flow
```
1. Reference Trajectory → Error-State Computation
2. Error-State + Model → ACADOS NMPC
3. ACADOS Solution → Control Inputs (u = [α, φ, θ, total_thrust])
4. Control Inputs → Motor Mixing → PX4 Motor Commands
```

### Aerodynamic Computation Flow
```
1. Request (cz) → CzCalculate
2. Solver Method Selection:
   - lookup_table: Binary search + interpolation
   - newton_raphson: Iterative with temporal coherence
   - auto: Adaptive selection
3. Output → α (angle of attack)
```

## Build System

### Primary: uv CLI
```bash
uv run ap-pnc install     # Complete setup
uv run ap-pnc traj        # Trajectory tools
uv run ap-pnc aerodyn     # Aerodynamics
uv run ap-pnc benchmark   # Timing benchmarks
uv run ap-pnc test        # Testing
```

### Python Execution
```bash
uv run python             # uv-managed Python execution
uv run python script.py   # Auto-detects .venv and project root
uv run ap-pnc <cmd>       # Run project CLI commands
```

### ROS2 Build
```bash
scripts/tools/build       # Colcon wrapper
scripts/tools/plan        # Trajectory planning
scripts/tools/srunall     # Full system launch
```

## Configuration Files

| File | Purpose |
|------|---------|
| `global.toml` | Global system parameters, solver method selection |
| `ap_pnc/lu_mpc/.cache/ocp_*.json` | ACADOS solver configurations |
| `.clang-format` | C++ formatting rules |
| `justfile` | All development tasks |

## Integration Points

| Component | Interface |
|-----------|-----------|
| Python → C++ | ACADOS Python bindings |
| ROS2 → PX4 | MAVSDK/Mavlink |
| Planning → Control | Reference trajectory messages |
| Simulation → Hardware | PX4 SITL interface |

## Documentation

- **Full Documentation**: https://warriorhanamy.github.io/AP-PnC/
- **Source**: `docs/` directory (Jekyll)
