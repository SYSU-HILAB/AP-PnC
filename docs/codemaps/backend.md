# Backend Architecture (C++)

**Last Updated**: 2025-01-25

## Overview

The C++ backend provides real-time trajectory planning, aerodynamic computation, and hardware interfaces for the AP-PnC system. It leverages ACADOS for optimization and ROS2 for robotics middleware integration.

## Directory Structure

```
src/
├── aerodynamics/              # Aerodynamic computation
│   ├── CzCalculate.{hpp,cpp}  # Inverse aerodynamic solver
│   ├── zhang_lyu_aerodynamics.hpp
│   ├── bspline_aerodynamics.hpp
│   ├── phi_aerodynamics.hpp
│   └── advanced_lift_drag.hpp
│
├── trajectory_server/         # Trajectory planning
│   ├── global_planner.{hpp,cpp}     # Main planning interface
│   ├── attitude_computation.{hpp,cpp}
│   ├── trajectory_type_{line,circle,...}.{hpp,cpp}
│   └── gcopter/                    # Planning library
│       ├── MINCO.hpp               # MINCO trajectory
│      ── LBFGS.hpp                 # LBFGS optimizer
│       └── SDLP.hpp                # SDLP solver
│
├── px4_interface/            # PX4 hardware interface
│   └── px4_interface.cpp      # ROS2 node for PX4 SITL
│
├── c_nmpc/                   # ACADOS NMPC interface
│   └── c_proposed_nmpc.{hpp,cpp}
│
├── tailsitter_df/            # Dynamics model
│   ├── tailsitter_dynamics_fun.{hpp,cpp}
│   ├── tailsitter_dynamics_grad.hpp
│   └── derivative.hpp        # Automatic differentiation
│
├── basic_trajectories/       # Trajectory primitives
│   ├── TrajectoryBase.hpp
│   ├── TrajectoryLine.hpp
│   ├── TrajectoryCircle.hpp
│   ├── TrajectoryLemniscate.hpp
│   └── TrajectoryVerticalLoop.hpp
│
├── utils/                    # Utility libraries
│   ├── config_reader.hpp     # TOML/YAML config
│   ├── csv_writer.hpp        # Data logging
│   └── scope_profiler.hpp    # Performance profiling
│
└── external_libraries/       # Third-party dependencies
    ├── tomlplusplus/         # TOML parser
    ├── yamlcpp/              # YAML parser
    └── BSplinebasis/         # B-spline computations
```

## Core Components

### 1. Aerodynamics Module (`src/aerodynamics/`)

#### CzCalculate Class
**File**: `CzCalculate.{hpp,cpp}`

Inverse aerodynamic solver computing angle of attack (α) from lift coefficient (Cz).

**Solver Methods**:
```cpp
enum class SolverMethod {
    LOOKUP_TABLE,      // Binary search with interpolation
    NEWTON_RAPHSON,    // Iterative solver with temporal coherence
    AUTO               // Adaptive selection
};
```

**Key Methods**:
```cpp
class CzCalculate {
public:
    // Main interface: compute alpha from cz
    double calculateAlpha(double cz, double guess = 0.0);

    // Lookup table method (robust)
    double calculateAlphaLookupTable(double cz);

    // Newton-Raphson method (fast for smooth trajectories)
    double calculateAlphaNewtonRaphson(double cz, double initial_guess);

    // Auto method (adaptive)
    double calculateAlphaAuto(double cz, double previous_alpha);

private:
    std::vector<double> alpha_table_;
    std::vector<double> cz_table_;
    SolverMethod method_;
};
```

**Configuration** (`config/aero_alpha_solver.yaml`):
```yaml
max_iterations: 1000
tolerance: 1e-6
damping_factor: 0.5
derivative_epsilon: 1e-3
initial_alpha_deg: 90.0
```

**Dependencies**: None (self-contained)

**Used By**:
- `src/trajectory_server/global_planner.cpp:calculateAlphaFromCz()`

#### Aerodynamic Models
| Model | File | Description |
|-------|------|-------------|
| Zhang-Lyu | `zhang_lyu_aerodynamics.hpp` | Primary aerodynamic model |
| BSpline | `bspline_aerodynamics.hpp` | B-spline interpolated model |
| Phi | `phi_aerodynamics.hpp` | Phi-dependent model |
| Advanced | `advanced_lift_drag.hpp` | High-fidelity lift/drag |

### 2. Trajectory Server (`src/trajectory_server/`)

#### GlobalPlanner Class
**File**: `global_planner.{hpp,cpp}`

Main trajectory planning interface integrating aerodynamic constraints.

**Key Methods**:
```cpp
class GlobalPlanner {
public:
    // Plan trajectory from start to goal
    Trajectory plan(const Eigen::Vector3d& start,
                    const Eigen::Vector3d& goal,
                    const PlanningConstraints& constraints);

    // Compute angle of attack from lift coefficient
    double calculateAlphaFromCz(double cz);

    // Integrate aerodynamic constraints
    void applyAerodynamicConstraints(Trajectory& traj);

private:
    std::unique_ptr<CzCalculate> cz_calculator_;
    std::unique_ptr<MINCO> minco_trajectory_;
};
```

**Dependencies**:
- `src/aerodynamics/CzCalculate.hpp`
- `src/trajectory_server/gcopter/MINCO.hpp`
- `src/trajectory_server/gcopter/LBFGS.hpp`

**Trajectory Types**:
- `TrajectoryTypeLine`: Straight line trajectories
- `TrajectoryTypeCircle`: Circular orbits
- `TrajectoryTypeLemniscate`: Figure-8 patterns
- `TrajectoryTypeVerticalLoop`: Vertical loops

**Exported Symbols**: None (linked directly by PX4 interface node)

#### Gcopter Planning Library
**Directory**: `src/trajectory_server/gcopter/`

External planning algorithms:

| Component | File | Purpose |
|-----------|------|---------|
| MINCO | `MINCO.hpp` | MINimum Control trajectory |
| LBFGS | `LBFGS.hpp` | Limited-memory BFGS optimizer |
| SDLP | `SDLP.hpp` | Sequential Dual Linear Programming |

### 3. PX4 Interface (`src/px4_interface/`)

#### PX4Interface Node
**File**: `px4_interface.cpp`

ROS2 node for PX4 SITL/HIL integration.

**Key Functionality**:
```cpp
class PX4Interface : public rclcpp::Node {
public:
    // Subscribe to reference trajectory
    void trajectoryCallback(const TrajectoryMsg& msg);

    // Publish motor commands
    void publishMotorCommands(const MotorCmd& cmd);

    // Offboard mode control
    void engageOffboardMode();

private:
    rclcpp::Publisher<MotorCmd>::SharedPtr motor_pub_;
    rclcpp::Subscription<TrajectoryMsg>::SharedPtr traj_sub_;
};
```

**ROS2 Topics**:
- Subscribes: `/reference_trajectory`
- Publishes: `/motor_commands`, `/px4/mocap/odometry`

**Dependencies**:
- `rclcpp` (ROS2 C++ client library)
- `mavsdk` (MAVLink bridge)

### 4. C++ NMPC (`src/c_nmpc/`)

#### CProposedNMPC Class
**File**: `c_proposed_nmpc.{hpp,cpp}`

Direct ACADOS interface for real-time MPC optimization.

**Key Methods**:
```cpp
class CProposedNMPC {
public:
    // Initialize ACADOS solver
    void initialize(const std::string& config_file);

    // Solve MPC problem
    MPCSolution solve(const State& x0, const Reference& ref);

    // Get solver statistics
    SolverStats getStats() const;

private:
    ocp_nlp_solver* solver_;
    void* acados_solver_;
};
```

**Dependencies**:
- `.artifacts/acados_src/acados/` (ACADOS library, fetched by `ap-pnc install`)
- `src/tailsitter_df/` (Dynamics model)

**Configuration**: `ap_pnc/lu_mpc/acados_ocp_nlp.json`

### 5. Tailsitter Dynamics (`src/tailsitter_df/`)

#### TailsitterDynamics Class
**File**: `tailsitter_dynamics_fun.{hpp,cpp}`

Tailsitter UAV dynamics model with automatic differentiation.

**State Space** (18D):
```cpp
using State = Eigen::Matrix<double, 18, 1>;
// [x, y, z, vx, vy, vz, phi, theta, psi,
//  qw, qx, qy, qz, m1, m2, m3, m4, thrust_rate]
```

**Key Methods**:
```cpp
class TailsitterDynamics {
public:
    // Continuous-time dynamics
    State dynamics(const State& x, const ControlInput& u);

    // Jacobian computation (forward mode AD)
    StateJacobian jacobian(const State& x, const ControlInput& u);

private:
    AerodynamicModel aero_model_;
};
```

**Dependencies**:
- `src/aerodynamics/` (Aerodynamic coefficients)

### 6. Basic Trajectories (`src/basic_trajectories/`)

Base classes and implementations for trajectory primitives.

**Class Hierarchy**:
```
TrajectoryBase (abstract)
    ├── TrajectoryLine
    ├── TrajectoryCircle
    ├── TrajectoryLemniscate
    └── TrajectoryVerticalLoop
```

**Interface**:
```cpp
class TrajectoryBase {
public:
    virtual Eigen::Vector3d getPosition(double t) const = 0;
    virtual Eigen::Vector3d getVelocity(double t) const = 0;
    virtual double getDuration() const = 0;
};
```

### 7. Utilities (`src/utils/`)

| Component | File | Purpose |
|-----------|------|---------|
| ConfigReader | `config_reader.hpp` | TOML/YAML configuration |
| CSVWriter | `csv_writer.hpp` | Data logging to CSV |
| ScopeProfiler | `scope_profiler.hpp` | Performance timing |

### 8. External Libraries (`src/external_libraries/`)

Third-party dependencies bundled with the project:

| Library | Directory | Purpose |
|---------|-----------|---------|
| tomlplusplus | `tomlplusplus/` | TOML configuration parsing |
| yamlcpp | `yamlcpp/` | YAML file processing |
| BSplinebasis | `BSplinebasis/` | B-spline basis functions |

## Build System

### CMakeLists.txt Structure
```cmake
# Main executable targets
add_executable(trajectory_server_node src/trajectory_server/global_planner.cpp)
add_executable(px4_interface_node src/px4_interface/px4_interface.cpp)

# Link ACADOS
target_link_libraries(trajectory_server PRIVATE acados)

# ROS2 components
ament_target_dependencies(trajectory_server_node rclcpp std_msgs)
```

### Compilation
```bash
# Via ap-pnc CLI
uv run ap-pnc build all

# Direct colcon
colcon build --packages-select ap_pnc_cpp

# With ROS2 wrapper
scripts/tools/build
```

## Dependencies

### External Packages
| Package | Version | Purpose |
|---------|---------|---------|
| ACADOS | v2023.2.1 | MPC optimization framework |
| ROS2 | Humble | Robotics middleware |
| Eigen3 | 3.4+ | Linear algebra |

### Internal Dependencies
```
trajectory_server → aerodynamics, gcopter
px4_interface → c_nmpc, tailsitter_df
c_nmpc → tailsitter_df, aerodynamics
tailsitter_df → aerodynamics
```

## Performance Considerations

### Real-Time Constraints
- **Trajectory Server**: < 100ms planning time
- **NMPC Solve**: < 10ms per iteration
- **Aerodynamic Lookup**: < 1μs per query (cached)

### Memory Usage
- **Lookup Table**: ~100 KB (in-memory)
- **Trajectory Cache**: ~1 MB per trajectory
- **ACADOS Solver**: ~10 MB working memory

## Thread Safety
- `CzCalculate`: Thread-safe (const methods)
- `GlobalPlanner`: Not thread-safe (single-threaded use)
- `PX4Interface`: Thread-safe (ROS2 callbacks)

## Error Handling
```cpp
// Exception hierarchy
class AerodynamicException : public std::runtime_error;
class PlanningException : public std::runtime_error;
class SolverException : public std::runtime_error;
```

## Testing

### Unit Tests
```bash
# Run tests
uv run ap-pnc test aero
uv run ap-pnc test all
```

### Integration Tests
```bash
# Full system visualization test
uv run ap-pnc traj viz-tracking
```
