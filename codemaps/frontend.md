# Frontend Architecture (Python)

**Last Updated**: 2025-01-25

## Overview

The Python frontend provides a modern, flexible interface for model simulation, control algorithms, and visualization. It supports multiple symbolic computation backends (CasADi, JAX, PyTorch) through a unified API.

## Directory Structure

```
feature/lu_mpc/
├── __init__.py
│
├── model/                      # Model implementations
│   ├── sim/                   # Simulation backends
│   │   ├── sim_gd_model.py           # Unified interface
│   │   ├── gd_model_casadi.py        # CasADi backend
│   │   ├── gd_model_jax.py           # JAX backend
│   │   └── gd_model_torch.py         # PyTorch backend
│   │
│   └── ctrl/                  # Control models
│       ├── proposed_model.py         # Proposed tailsitter model
│       ├── err_state_model.py        # Error-state formulation
│       └── aero_evaluator.py         # Aerodynamic evaluation
│
├── controller/                # MPC controllers
│   ├── ctrl_proposed_nmpc.py         # Non-linear MPC
│   ├── ctrl_err_state_lmpc.py        # Linear MPC (error-state)
│   └── __init__.py
│
├── plotting/                  # Visualization tools
│   ├── plot_aero_coefficients.py     # Aerodynamic plots
│   ├── plot_trajectory.py            # 3D trajectory visualization
│   └── plot_mesh_aero.py             # Mesh-based aero visualization
│
├── ros/                       # ROS2 integration
│   ├── nodes/                          # ROS2 node implementations
│   ├── launch/                         # Launch files
│   └── config/                         # ROS2 configuration
│
├── test/                      # Test suite
│   ├── test_sim_model.py              # Model tests
│   ├── test_vmap.py                   # JAX vectorization tests
│   └── test_acados_nmpc.py            # NMPC tests
│
└── .cache/                    # Cached data
    ├── mesh_aero_data_jax.npy
    ├── mesh_aero_data_casadi.npy
    └── mesh_aero_data_torch.npy
```

## Core Components

### 1. Simulation Models (`feature/lu_mpc/model/sim/`)

#### Unified Interface
**File**: `sim_gd_model.py`

Factory function for backend-agnostic model creation.

```python
from feature.lu_mpc.model.sim import get_model

# Backend-agnostic model creation
model = get_model(backend="jax")  # or "casadi", "torch"

# Unified interface
x_next = model.dynamics(state, control)
aero = model.aerodynamics(alpha, velocity)
```

**Factory Function Signature**:
```python
def get_model(
    backend: Literal["casadi", "jax", "torch"] = "jax",
    config: dict | None = None
) -> GDModel:
    """Get aerodynamic model with specified backend.

    Args:
        backend: Symbolic computation backend
        config: Optional configuration dictionary

    Returns:
        GDModel instance with unified interface
    """
```

**Exports**:
- `get_model()`: Factory function
- `GDModel`: Base model class (protocol)

#### CasADi Backend
**File**: `gd_model_casadi.py`

Primary backend for ACADOS integration and symbolic optimization.

```python
import casadi as ca
from feature.lu_mpc.model.sim.gd_model_casadi import GDModelCasadi

model = GDModelCasadi()

# Symbolic variables
alpha = ca.SX.sym('alpha')
velocity = ca.SX.sym('velocity', 3)

# Symbolic computation
cz, cd = model.aerodynamics(alpha, velocity)

# Generate C code for ACADOS
model.generate_c_code("aerodynamics.c")
```

**Key Classes**:
```python
class GDModelCasadi:
    """CasADi-based aerodynamic model.

    Features:
        - Symbolic differentiation
        - C code generation for ACADOS
        - Analytic Jacobians
    """

    def aerodynamics(
        self, alpha: ca.SX, velocity: ca.SX
    ) -> tuple[ca.SX, ca.SX]:
        """Compute lift and drag coefficients.

        Returns:
            (cz, cd) symbolic expressions
        """

    def dynamics(
        self, state: ca.SX, control: ca.SX
    ) -> ca.SX:
        """Continuous-time dynamics."""

    def export_to_acados(self) -> str:
        """Export model for ACADOS solver."""
```

**Dependencies**:
- `casadi >= 3.6.0`
- `numpy`

**Used By**:
- `feature/lu_mpc/model/ctrl/proposed_model.py` (ACADOS integration)
- ACADOS code generation

#### JAX Backend
**File**: `gd_model_jax.py`

High-performance backend with JIT compilation and automatic vectorization.

```python
import jax
import jax.numpy as jnp
from feature.lu_mpc.model.sim.gd_model_jax import GDModelJax

model = GDModelJax()

# JIT-compiled computation
compute_aero = jax.jit(model.aerodynamics)
cz, cd = compute_aero(alpha, velocity)

# Vectorized computation (batch processing)
batch_alpha = jnp.array([0.1, 0.2, 0.3])
batch_cz = jax.vmap(model.aerodynamics)(batch_alpha, velocity)
```

**Key Classes**:
```python
class GDModelJax:
    """JAX-based aerodynamic model.

    Features:
        - JIT compilation (jax.jit)
        - Automatic vectorization (jax.vmap)
        - Automatic differentiation (jax.grad)
        - GPU/TPU support
    """

    @jax.jit
    def aerodynamics(
        self, alpha: jnp.ndarray, velocity: jnp.ndarray
    ) -> tuple[jnp.ndarray, jnp.ndarray]:
        """Compute lift and drag coefficients.

        Returns:
            (cz, cd) JIT-compiled function
        """

    def dynamics_batch(
        self, states: jnp.ndarray, controls: jnp.ndarray
    ) -> jnp.ndarray:
        """Vectorized dynamics computation."""
```

**Performance**:
- **JIT speedup**: 10-100x vs pure NumPy
- **vmap speedup**: Near-linear scaling for batch operations
- **Memory**: Efficient XLA compilation

**Dependencies**:
- `jax >= 0.4.0`
- `jaxlib >= 0.4.0`

**Used By**:
- Batch trajectory optimization
- Monte Carlo simulation
- Sensitivity analysis

#### PyTorch Backend
**File**: `gd_model_torch.py`

Neural network integration with GPU acceleration.

```python
import torch
from feature.lu_mpc.model.sim.gd_model_torch import GDModelTorch

model = GDModelTorch()

# GPU computation
device = torch.device("cuda")
model.to(device)

alpha = torch.tensor([0.1], device=device)
cz, cd = model.aerodynamics(alpha, velocity)

# Neural network integration
class NeuralAeroModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.base_model = GDModelTorch()
        self.nn_correction = torch.nn.Linear(3, 2)

    def forward(self, alpha, vel):
        cz_base, cd_base = self.base_model(alpha, vel)
        correction = self.nn_correction(torch.cat([alpha, vel]))
        return cz_base + correction[:, 0], cd_base + correction[:, 1]
```

**Key Classes**:
```python
class GDModelTorch:
    """PyTorch-based aerodynamic model.

    Features:
        - GPU acceleration
        - Neural network integration
        - PyTorch ecosystem compatibility
    """

    def aerodynamics(
        self, alpha: torch.Tensor, velocity: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Compute lift and drag coefficients.

        Returns:
            (cz, cd) as PyTorch tensors
        """
```

**Dependencies**:
- `torch >= 2.0.0`

**Used By**:
- Neural-augmented aerodynamics
- Deep reinforcement learning
- GPU-accelerated batch processing

### 2. Control Models (`feature/lu_mpc/model/ctrl/`)

#### Proposed Model
**File**: `proposed_model.py`

Full tailsitter dynamics model for NMPC formulation.

```python
from feature.lu_mpc.model.ctrl.proposed_model import ProposedModel

model = ProposedModel(
    backend="casadi",  # Required for ACADOS
    config="global.toml"
)

# Define OCP constraints
model.set_constraints(
    state_constraints={...},
    control_constraints={...},
    terminal_constraints={...}
)

# Export to ACADOS
model.export_acados_template("feature/lu_mpc/acados_ocp_nlp.json")
```

**State Space** (18D):
```python
# State vector
x = [
    x, y, z,           # Position (3)
    vx, vy, vz,        # Velocity (3)
    phi, theta, psi,   # Euler angles (3)
    qw, qx, qy, qz,    # Quaternion (4)
    m1, m2, m3, m4,    # Motor commands (4)
    thrust_rate        # Thrust rate (1)
]  # Total: 18
```

**Control Input** (4D):
```python
u = [alpha, phi, theta, total_thrust]
#   angle of attack, roll, pitch, thrust
```

**Key Methods**:
```python
class ProposedModel:
    """Full tailsitter dynamics for NMPC.

    State: 18D (position, velocity, attitude, quaternion, motors, thrust)
    Control: 4D (alpha, phi, theta, total_thrust)
    """

    def dynamics(self, x, u):
        """Continuous-time dynamics: dx/dt = f(x, u)"""

    def residual_dynamics(self, x, u, x_ref):
        """Error-state dynamics for tracking"""

    def stage_cost(self, x, u, x_ref, u_ref):
        """Stage cost: l(x, u)"""

    def terminal_cost(self, x, x_ref):
        """Terminal cost: V(x_N)"""
```

**Dependencies**:
- `feature/lu_mpc/model/sim/gd_model_casadi.py`
- `casadi`

#### Error-State Model
**File**: `err_state_model.py`

Linearized error-state model for lMPC formulation.

```python
from feature.lu_mpc.model.ctrl.err_state_model import ErrStateModel

model = ErrStateModel(operating_point="hover")

# Linearized dynamics: delta_x_dot = A * delta_x + B * delta_u
A, B = model.linearize(x_op, u_op)

# LQR formulation
Q = model.get_Q_weight()
R = model.get_R_weight()

# Solve discrete-time LQR
K = model.solve_lqr(A, B, Q, R)
```

**Error-State Formulation**:
```python
# Error state
delta_x = x - x_ref

# Error dynamics (linearized)
delta_x_dot = A * delta_x + B * delta_u
```

**Key Methods**:
```python
class ErrStateModel:
    """Error-state model for linear MPC.

    Linearizes dynamics around operating point.
    """

    def linearize(self, x_op, u_op) -> tuple[npt.NDArray, npt.NDArray]:
        """Compute Jacobians: A, B"""

    def discretize(self, A, B, dt) -> tuple[npt.NDArray, npt.NDArray]:
        """Discretize: Ad, Bd"""

    def get_constraints(self) -> dict:
        """Get state/control constraints"""
```

**Dependencies**:
- `feature/lu_mpc/model/sim/gd_model_jax.py` (or CasADi)
- `scipy` (for LQR)

#### Aerodynamic Evaluator
**File**: `aero_evaluator.py`

Evaluates aerodynamic coefficients from pre-computed mesh data.

```python
from feature.lu_mpc.model.ctrl.aero_evaluator import AeroEvaluator

evaluator = AeroEvaluator(backend="jax")

# Load cached mesh data
evaluator.load_cache("feature/lu_mpc/.cache/mesh_aero_data_jax.npy")

# Evaluate at query point
cz, cd, cy = evaluator.evaluate(alpha=0.1, beta=0.0, velocity=10.0)
```

**Key Classes**:
```python
class AeroEvaluator:
    """Interpolates aerodynamic coefficients from mesh data.

    Mesh grid: (alpha, beta, velocity) → (cz, cd, cy)
    """

    def evaluate(
        self, alpha: float, beta: float, velocity: float
    ) -> tuple[float, float, float]:
        """Trilinear interpolation on mesh data."""
```

### 3. Controllers (`feature/lu_mpc/controller/`)

#### Proposed NMPC Controller
**File**: `ctrl_proposed_nmpc.py`

Non-linear MPC controller using ACADOS solver.

```python
from feature.lu_mpc.controller.ctrl_proposed_nmpc import CtrlProposedNMPC

controller = CtrlProposedNMPC(
    config="feature/lu_mpc/acados_ocp_nlp.json",
    model=ProposedModel(backend="casadi")
)

# Solve MPC problem
u_opt, x_opt = controller.solve(
    x0=current_state,
    x_ref=reference_trajectory
)

# Get first control input
u_star = u_opt[:, 0]  # [alpha, phi, theta, total_thrust]
```

**Key Classes**:
```python
class CtrlProposedNMPC:
    """Non-linear MPC with ACADOS.

    Horizon: N (typically 10-20)
    Solve time: < 10ms per iteration
    """

    def solve(self, x0, x_ref):
        """Solve OCP: min ∑(l(x, u) + V(x_N))

        Returns:
            u_opt: Optimal control sequence [N, 4]
            x_opt: Optimal state sequence [N+1, 18]
        """

    def get_solver_stats(self) -> dict:
        """Get ACADOS solver statistics.

        Returns:
            {'solve_time': ..., 'iterations': ..., 'status': ...}
        """
```

**Configuration**: `feature/lu_mpc/acados_ocp_nlp.json`

**Dependencies**:
- `acados` (Python bindings)
- `feature/lu_mpc/model/ctrl/proposed_model.py`

#### Error-State lMPC Controller
**File**: `ctrl_err_state_lmpc.py`

Linear MPC controller based on error-state linearization.

```python
from feature.lu_mpc.controller.ctrl_err_state_lmpc import CtrlErrStateLMPC

controller = CtrlErrStateLMPC(
    model=ErrStateModel(operating_point="hover"),
    horizon=20,
    Q=state_weights,
    R=control_weights
)

# Solve linear MPC (quadratic program)
delta_u = controller.solve(delta_x, delta_x_ref)

# Apply to operating point
u = u_op + delta_u
```

**Key Classes**:
```python
class CtrlErrStateLMPC:
    """Linear MPC based on error-state model.

    Formulates QP: min ½δx'Qδx + ½δu'Rδu
    s.t. δx_dot = A δx + B δu
    """

    def solve(self, delta_x, delta_x_ref):
        """Solve QP using OSQP/qpOASES."""
```

**Dependencies**:
- `cvxpy` or `osqp`
- `feature/lu_mpc/model/ctrl/err_state_model.py`

### 4. Plotting (`feature/lu_mpc/plotting/`)

#### Aerodynamic Coefficient Plotter
**File**: `plot_aero_coefficients.py`

```python
from feature.lu_mpc.plotting.plot_aero_coefficients import plot_aero

plot_aero(
    backend="jax",
    save_path="aero_plots.png",
    show=True
)
```

**Supported Plots**:
- Cz vs α (lift coefficient)
- Cd vs α (drag coefficient)
- Cy vs β (side force)
- 3D surface plots

#### 3D Trajectory Visualizer
**File**: `plot_trajectory.py`

```python
from feature.lu_mpc.plotting.plot_trajectory import plot_trajectory_3d

plot_trajectory_3d(
    reference_traj,
    actual_traj,
    save_path="trajectory.png"
)
```

#### Mesh Aerodynamic Visualizer
**File**: `plot_mesh_aero.py`

```python
from feature.lu_mpc.plotting.plot_mesh_aero import plot_mesh_aero

plot_mesh_aero(
    backend="torch",
    velocity_range=(5, 25),
    alpha_range=(-10, 25)
)
```

### 5. ROS2 Integration (`feature/lu_mpc/ros/`)

#### NMPC Visualization Node
**File**: `feature/lu_mpc/ros/nodes/test_proposed_nmpc_viz.py`

```python
from feature.lu_mpc.ros.nodes.test_proposed_nmpc_viz import main

main(
    config="global.toml",
    acados_config="acados_ocp_nlp.json"
)
```

**ROS2 Topics**:
- Subscribes: `/reference_trajectory`
- Publishes: `/mpc_trajectory`, `/tracking_error`

#### Launch Files
**Directory**: `feature/lu_mpc/ros/launch/`

- `test_proposed_nmpc_viz.launch.py`: Launch NMPC with visualization
- `test_ref_trajectory_viz.launch.py`: Playback reference trajectory

## Execution

### Python Wrapper
**File**: `pyp` (project root)

Git-aware Python wrapper for correct environment detection.

```bash
# Run Python with auto-detected environment
./pyp script.py
./pyp -m module_name
./pyp -c "print('hello')"
```

**Features**:
- Detects git root (including worktrees)
- Uses `.acados_venv` virtual environment
- Sets `LD_LIBRARY_PATH` for ACADOS
- No manual environment sourcing needed

### Justfile Commands

```bash
# Data generation
just generate-mesh-data-jax 361       # JAX backend
just generate-mesh-data-casadi        # CasADi backend
just generate-mesh-data-torch 361     # PyTorch backend

# Plotting
just plot-aero                        # Aerodynamic coefficients
just plot-mesh-aero-jax               # JAX mesh visualization
just plot-mesh-aero-casadi            # CasADi mesh
just plot-mesh-aero-torch             # PyTorch mesh

# Testing
just test-vmap                        # JAX vectorization
just test-sim-model                   # Simulation model
just test-proposed-nmpc               # ACADOS NMPC
just test-proposed-nmpc-viz           # NMPC with ROS2

# Benchmarks
just compare-timing                   # CasADi vs JAX vs Torch
just benchmark-jax                    # JAX performance
just benchmark-torch                  # PyTorch performance
```

## Configuration

### Global Configuration
**File**: `global.toml`

```toml
[backend]
default = "jax"  # casadi, jax, torch

[cz_calculate]
solver_method = "auto"

[cache]
enabled = true
path = "feature/lu_mpc/.cache"
```

### ACADOS Configuration
**File**: `feature/lu_mpc/acados_ocp_nlp.json`

ACADOS solver configuration (NLP, QP solver, integrator).

## Dependencies

### Core Dependencies
| Package | Version | Purpose |
|---------|---------|---------|
| CasADi | >= 3.6.0 | Symbolic optimization |
| JAX | >= 0.4.0 | JIT compilation |
| PyTorch | >= 2.0.0 | Neural networks |
| NumPy | >= 1.24 | Numerical computing |
| SciPy | >= 1.10 | Scientific computing |

### Optional Dependencies
| Package | Purpose |
|---------|---------|
| cvxpy | QP formulation (lMPC) |
| osqp | QP solver |
| matplotlib | Plotting |
| ROS2 | Real-time visualization |

## Performance

### Backend Comparison

| Backend | JIT Compile | Vectorization | GPU | Best For |
|---------|-------------|---------------|-----|----------|
| CasADi | No | No | No | ACADOS, symbolic optimization |
| JAX | Yes | Yes (vmap) | Yes | Batch processing, simulation |
| PyTorch | Yes | Manual | Yes | Neural networks, GPU |

### Typical Timings

| Operation | CasADi | JAX | PyTorch |
|-----------|--------|-----|---------|
| Dynamics (single) | 10 μs | 1 μs | 5 μs |
| Dynamics (batch 100) | 1 ms | 10 μs | 100 μs |
| Gradient | Auto | Auto | Auto |
| Code generation | Yes | No | No |

## Type Hints

Project uses modern Python type hints:

```python
from typing import Literal

def get_model(
    backend: Literal["casadi", "jax", "torch"] = "jax",
    config: dict | None = None
) -> GDModel:
    ...
```

No `Optional` or `Union` - use `| None` instead.
