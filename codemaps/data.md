# Data Models and Schemas

**Last Updated**: 2025-01-25

## Overview

This document describes the data structures, schemas, and interfaces used throughout the AP-PnC system, including state representations, control inputs, trajectories, and configuration formats.

## State Space Definition

### Full State (18D)

The 18-dimensional state vector represents the complete system state of the tailsitter UAV.

```cpp
// C++ representation
using State = Eigen::Matrix<double, 18, 1>;

// Index mapping
enum StateIndex {
    // Position (3)
    POS_X = 0, POS_Y, POS_Z,

    // Velocity (3)
    VEL_X, VEL_Y, VEL_Z,

    // Euler angles (3)
    PHI, THETA, PSI,

    // Quaternion (4)
    QW, QX, QY, QZ,

    // Motor commands (4)
    M1, M2, M3, M4,

    // Thrust rate (1)
    THRUST_RATE
};
```

```python
# Python representation
import numpy as np
from typing import NamedTuple

class State(NamedTuple):
    """18D state vector."""

    # Position (3)
    x: float
    y: float
    z: float

    # Velocity (3)
    vx: float
    vy: float
    vz: float

    # Euler angles (3)
    phi: float    # Roll
    theta: float  # Pitch
    psi: float    # Yaw

    # Quaternion (4)
    qw: float
    qx: float
    qy: float
    qz: float

    # Motor commands (4)
    m1: float
    m2: float
    m3: float
    m4: float

    # Thrust rate (1)
    thrust_rate: float

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'State':
        """Create from 18-element array."""
        return cls(**{f.name: arr[i] for i, f in enumerate(cls._fields)})

    def to_array(self) -> np.ndarray:
        """Convert to 18-element array."""
        return np.array(self)
```

### State Segments

#### Position (3D)
```python
Position = tuple[float, float, float]  # (x, y, z) in meters (NED frame)
```

#### Velocity (3D)
```python
Velocity = tuple[float, float, float]  # (vx, vy, vz) in m/s (body frame)
```

#### Attitude
```python
# Euler angles (ZYX convention)
EulerAngles = tuple[float, float, float]  # (roll, pitch, yaw) in radians

# Quaternion
Quaternion = tuple[float, float, float, float]  # (w, x, y, z), normalized
```

#### Motor Commands (4D)
```python
MotorCommands = tuple[float, float, float, float]  # (m1, m2, m3, m4) in [0, 1]
```

## Control Input

### Control Vector (4D)

```cpp
// C++
using ControlInput = Eigen::Matrix<double, 4, 1>;

// Elements
// [0]: alpha (angle of attack) in radians
// [1]: phi (roll angle) in radians
// [2]: theta (pitch angle) in radians
// [3]: total_thrust in Newtons
```

```python
# Python
class ControlInput(NamedTuple):
    """4D control input vector."""

    alpha: float         # Angle of attack [rad]
    phi: float           # Roll angle [rad]
    theta: float         # Pitch angle [rad]
    total_thrust: float  # Total thrust [N]

    # Constraints
    ALPHA_MIN, ALPHA_MAX = -np.deg2rad(10), np.deg2rad(25)
    PHI_MIN, PHI_MAX = -np.deg2rad(30), np.deg2rad(30)
    THETA_MIN, THETA_MAX = -np.deg2rad(30), np.deg2rad(30)
    THRUST_MIN, THRUST_MAX = 0.0, 40.0  # Newtons

    def validate(self) -> bool:
        """Check if control input is within bounds."""
        return (
            self.ALPHA_MIN <= self.alpha <= self.ALPHA_MAX and
            self.PHI_MIN <= self.phi <= self.PHI_MAX and
            self.THETA_MIN <= self.theta <= self.THETA_MAX and
            self.THRUST_MIN <= self.total_thrust <= self.THRUST_MAX
        )
```

## Trajectory Data

### Reference Trajectory

```python
class ReferenceTrajectory:
    """Reference trajectory for MPC tracking."""

    def __init__(
        self,
        t: np.ndarray,          # Time stamps [N]
        pos: np.ndarray,        # Position [N, 3]
        vel: np.ndarray,        # Velocity [N, 3]
        acc: np.ndarray | None = None,  # Acceleration [N, 3]
        jerk: np.ndarray | None = None  # Jerk [N, 3]
    ):
        self.t = t
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.jerk = jerk

    def interpolate(self, t_query: float) -> 'ReferencePoint':
        """Interpolate trajectory at query time."""
        # Cubic spline interpolation
        ...

class ReferencePoint(NamedTuple):
    """Single point on reference trajectory."""

    t: float
    pos: np.ndarray    # [3]
    vel: np.ndarray    # [3]
    acc: np.ndarray    # [3] (optional)
```

### Trajectory Types

```python
from enum import Enum

class TrajectoryType(Enum):
    """Supported trajectory types."""

    LINE = "line"
    CIRCLE = "circle"
    LEMNISCATE = "lemniscate"  # Figure-8
    VERTICAL_LOOP = "vertical_loop"
    CUSTOM = "custom"
```

## Aerodynamic Data

### Aerodynamic Coefficients

```python
class AeroCoefficients(NamedTuple):
    """Aerodynamic force coefficients."""

    cx: float  # Drag coefficient (x-body axis)
    cy: float  # Side force coefficient (y-body axis)
    cz: float  # Lift coefficient (z-body axis)

    @classmethod
    def from_lookup(cls, alpha: float, mach: float) -> 'AeroCoefficients':
        """Lookup from pre-computed table."""
        ...
```

### Mesh Data Format

```python
class AeroMeshData:
    """3D mesh grid of aerodynamic coefficients.

    Grid dimensions: (n_alpha, n_beta, n_velocity)
    """

    def __init__(
        self,
        alpha_grid: np.ndarray,     # Angle of attack grid [n_alpha]
        beta_grid: np.ndarray,      # Sideslip angle grid [n_beta]
        velocity_grid: np.ndarray,  # Velocity grid [n_velocity]
        cx: np.ndarray,             # Drag coeff [n_alpha, n_beta, n_velocity]
        cy: np.ndarray              # Side force [n_alpha, n_beta, n_velocity]
        cz: np.ndarray,             # Lift coeff [n_alpha, n_beta, n_velocity]
    ):
        self.alpha_grid = alpha_grid
        self.beta_grid = beta_grid
        self.velocity_grid = velocity_grid
        self.cz = cz
        self.cd = cd
        self.cy = cy

    def evaluate(
        self,
        alpha: float,
        beta: float,
        velocity: float
    ) -> AeroCoefficients:
        """Trilinear interpolation."""
        ...

    @classmethod
    def load(cls, path: str) -> 'AeroMeshData':
        """Load from .npy file."""
        data = np.load(path, allow_pickle=True).item()
        return cls(**data)

    def save(self, path: str) -> None:
        """Save to .npy file."""
        np.save(path, self.__dict__)
```

### Lookup Table Format

```csv
# aero_lookup_table.csv
alpha_deg,cz,cd,cy
-10.0,-0.15,0.02,0.001
-9.5,-0.12,0.018,0.001
...
25.0,1.2,0.15,0.002
```

## Configuration Schemas

### Global Configuration (TOML)

```toml
# global.toml

[backend]
# Default symbolic computation backend
default = "jax"  # Options: casadi, jax, torch

[cz_calculate]
# Solver method for inverse aerodynamic lookup
solver_method = "auto"  # Options: lookup_table, newton_raphson, auto

[cz_calculate.lookup_table]
path = "artifacts/simulations/aero_lookup_table.csv"
interpolation_method = "linear"

[cz_calculate.newton_raphson]
max_iterations = 50
tolerance = 1e-8
damping_factor = 1.0
derivative_epsilon = 1e-6
enable_fallback = true

[cache]
# Cached aerodynamic data
enabled = true
path = "ap_pnc/lu_mpc/.cache"
mesh_size_alpha = 361    # Number of grid points in alpha
mesh_size_beta = 361     # Number of grid points in beta
mesh_size_velocity = 100 # Number of grid points in velocity

[trajectory]
# Trajectory planning parameters
planning_horizon = 10.0  # seconds
sampling_time = 0.01     # seconds
max_velocity = 25.0      # m/s
max_acceleration = 10.0  # m/s^2

[mpc]
# MPC parameters
horizon = 20             # Prediction horizon
dt = 0.05                # Time step
q_state = 1.0            # State weight
r_control = 0.1          # Control weight
```

### ACADOS Configuration (JSON)

```json
{
  "acados_ocp_nlp": {
    "N": 20,
    "time_steps": [0.05, 0.05, ...],
    "constraints": {
      "x0": {
        "idx": [0, 1, 2, ...],
        "values": [0.0, 0.0, 0.0, ...]
      },
      "lbx": {
        "idx": [3],
        "values": [-10.0]
      },
      "ubx": {
        "idx": [3],
        "values": [25.0]
      }
    },
    "cost": {
      "type": "external_cost",
      "cost_type": "linear_ls"
    },
    "solver": {
      "qp_solver": "full_condensing_hpipm",
      "hpipm": {
        "max_iter": 50,
        "tol": 1e-6
      },
      "regularization": "convexify"
    }
  }
}
```

## ROS2 Message Schemas

### Trajectory Message

```python
# ReferenceTrajectory.msg
# std_msgs/Header header
# float64[] t             # Time stamps [N]
# float64[] x             # Position x [N]
# float64[] y             # Position y [N]
# float64[] z             # Position z [N]
# float64[] vx            # Velocity x [N]
# float64[] vy            # Velocity y [N]
# float64[] vz            # Velocity z [N]
```

### State Message

```python
# StateMsg.msg
# std_msgs/Header header
# float64 x
# float64 y
# float64 z
# float64 vx
# float64 vy
# float64 vz
# float64 phi
# float64 theta
# float64 psi
# float64 qw
# float64 qx
# float64 qy
# float64 qz
# float64 m1
# float64 m2
# float64 m3
# float64 m4
# float64 thrust_rate
```

### Control Message

```python
# ControlMsg.msg
# std_msgs/Header header
# float64 alpha
# float64 phi
# float64 theta
# float64 total_thrust
```

## Data Flow Diagrams

### Planning Pipeline

```
Waypoints → MINCO Trajectory → Aerodynamic Constraints
                             → Reference Trajectory
                             → MPC Reference
```

### Control Pipeline

```
Current State + Reference Trajectory
         ↓
    Error-State Computation
         ↓
        NMPC Solver (ACADOS)
         ↓
    Optimal Control Sequence
         ↓
    First Control Input
         ↓
    Motor Mixing
         ↓
    PX4 Motor Commands
```

## File Formats

### Data Artifacts

| File | Format | Description |
|------|--------|-------------|
| `aero_lookup_table.csv` | CSV | Aerodynamic lookup table |
| `mesh_aero_data_*.npy` | NumPy | Mesh aerodynamic data |
| `trajectory_*.csv` | CSV | Trajectory logs |
| `simulation_*.bag` | ROS2 bag | Simulation data |

### Configuration Files

| File | Format | Description |
|------|--------|-------------|
| `global.toml` | TOML | Global configuration |
| `acados_ocp_nlp.json` | JSON | ACADOS solver config |
| `launch_*.py` | Python | ROS2 launch files |

## Validation

### State Validation

```python
def validate_state(x: State) -> tuple[bool, list[str]]:
    """Validate state vector.

    Returns:
        (is_valid, list_of_errors)
    """
    errors = []

    # Check position bounds
    if not (-100 <= x.z <= 100):
        errors.append(f"Altitude {x.z} out of bounds")

    # Check velocity bounds
    v = np.array([x.vx, x.vy, x.vz])
    if np.linalg.norm(v) > 30.0:
        errors.append(f"Velocity {np.linalg.norm(v)} exceeds max")

    # Check quaternion normalization
    q = np.array([x.qw, x.qx, x.qy, x.qz])
    if abs(np.linalg.norm(q) - 1.0) > 1e-6:
        errors.append("Quaternion not normalized")

    # Check motor commands
    motors = [x.m1, x.m2, x.m3, x.m4]
    for i, m in enumerate(motors, 1):
        if not (0 <= m <= 1):
            errors.append(f"Motor {i} command {m} out of [0, 1]")

    return (len(errors) == 0, errors)
```

### Trajectory Validation

```python
def validate_trajectory(traj: ReferenceTrajectory) -> tuple[bool, list[str]]:
    """Validate reference trajectory."""
    errors = []

    # Check time stamps
    if np.any(np.diff(traj.t) <= 0):
        errors.append("Time stamps not strictly increasing")

    # Check continuity
    if traj.pos.shape[0] != traj.t.shape[0]:
        errors.append("Position and time dimensions mismatch")

    # Check dynamic feasibility
    vel_mag = np.linalg.norm(traj.vel, axis=1)
    if np.any(vel_mag > 25.0):
        errors.append(f"Velocity exceeds max: {vel_mag.max()}")

    return (len(errors) == 0, errors)
```

## Performance Data Structures

### Solver Statistics

```python
@dataclass
class SolverStats:
    """ACADOS solver statistics."""

    solve_time: float       # Total solve time [s]
    iterations: int         # Number of SQP iterations
    qp_iterations: list[int]  # QP iterations per SQP
    status: int             # Solver status (0 = success)
    cost: float             # Final cost value

    def is_success(self) -> bool:
        return self.status == 0
```

### Profile Data

```python
@dataclass
class ProfileData:
    """Performance profiling data."""

    function: str
    total_time: float
    call_count: int
    avg_time: float
    max_time: float
    min_time: float

    @classmethod
    def from_scope_profiler(cls, data: dict) -> 'ProfileData':
        ...
```
