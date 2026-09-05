# Aerodynamics Module

The aerodynamics module provides a comprehensive framework for modeling aerodynamic forces and moments for tail-sitter UAVs in the AP-PnC (Autonomous Parking with Predictive Control) framework.

## Overview

This module implements multiple aerodynamics models with varying levels of fidelity, from simple linear models to complex MATLAB-generated implementations. It is designed for trajectory planning and tracking control, with a key innovation being the inverse calculation of angle of attack from lift coefficient.

## Directory Structure

```
src/aerodynamics/
├── include/aerodynamics/         # Public headers
│   ├── aero_interface.hpp        # Abstract base interface
│   ├── aero_alpha_solver.hpp     # Newton-Raphson solver
│   ├── zhang_lyu_aerodynamics.hpp   # MATLAB-generated model
│   ├── bspline_aerodynamics.hpp  # B-spline interpolation model
│   ├── phi_aerodynamics.hpp      # Linear phi-theory model
│   ├── advanced_lift_drag.hpp    # Advanced lift-drag model
│   └── non_aerodynamics.hpp      # Null model for testing
├── src/                          # Implementation files
│   ├── aero_alpha_solver.cpp
│   ├── *_aerodynamics.cpp
│   ├── gazebo_plugins/           # ROS2/Gazebo integration
│   └── MatlabGenerated/          # Auto-generated MATLAB code
├── data/                         # Configuration and data
│   ├── aero_sim_params.db        # SQLite parameter database
│   ├── bspline_fit_results/      # B-spline coefficients
│   └── *.py                      # Data generation scripts
├── scripts/
│   └── generate_aero_lookup_table.cpp
└── test/
    └── test_aero_alpha_solver.cpp
```

## Core Architecture

### Base Interface

The module uses an abstract base class pattern with a factory for creating model instances:

- **`AerodynamicsInterface`**: Abstract base defining the interface for all aerodynamics models
  - Virtual method `getAeroWrench()` computes forces and moments
  - Returns aerodynamic force/moment vectors, alpha (angle of attack), and beta (sideslip angle)

- **`AerodynamicsCreator`**: Factory base class for creating aerodynamics instances
  - Implements the factory pattern for model creation
  - Loads parameters from configuration database

## Aerodynamics Models

### LyuAerodynamics (Zhang-Lyu Model)

The most sophisticated model, using MATLAB-generated code for complex aerodynamic effects.

```cpp
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>

// Factory creation
aerodynamics::LyuAerodynamicsCreator creator;
creator.createAerodynamics();
```

**Features:**
- MATLAB-generated implementation from Zhang-Lyu aerodynamics
- Comprehensive force and moment calculations
- Captures complex aerodynamic effects

### BsplineAerodynamics (B-spline Model)

Uses B-spline curves to model force coefficients as functions of angle of attack and sideslip angle.

```cpp
#include <aerodynamics/bspline_aerodynamics.hpp>

// Factory creation
aerodynamics::BsplineAerodynamicsCreator creator;
creator.createAerodynamics();
```

**Features:**
- Cubic B-spline interpolation (order 3)
- Control points and knots loaded from configuration
- Efficient for real-time control
- Data: `data/bspline_fit_results/bspline_fit_c{x,z}.npz`

### PhiAerodynamics (Linear Phi-Theory Model)

Simple linear mapping between body velocity and aerodynamic force.

```cpp
#include <aerodynamics/phi_aerodynamics.hpp>

// Factory creation
aerodynamics::PhiAerodynamicsCreator creator;
creator.createAerodynamics();
```

**Force Model:**
```
F = Phi * v * |v|
```
where:
- `F` is the aerodynamic force vector
- `Phi` is the 3x3 matrix of aerodynamic coefficients
- `v` is the body velocity vector
- `|v|` is the velocity magnitude

**Features:**
- Fast and lightweight
- Suitable for real-time control
- Phi matrix loaded from database

### AdvancedLiftDrag (Advanced Lift-Drag Model)

Separate lift and drag coefficients with stall effects.

```cpp
#include <aerodynamics/advanced_lift_drag.hpp>

// Factory creation
aerodynamics::AdvancedLiftDragCreator creator;
creator.createAerodynamics();
```

**Features:**
- Accounts for stall effects
- Parasitic and induced drag components
- Uses PIMPL pattern for data hiding
- Configurable coefficients

### NonAerodynamics (Null Model)

Returns zero forces and moments for testing or disabling aerodynamics.

```cpp
#include <aerodynamics/non_aerodynamics.hpp>

// Factory creation
aerodynamics::NonAerodynamicsCreator creator;
creator.createAerodynamics();
```

## Key Algorithm: AeroAlphaSolver

The `AeroAlphaSolver` class is the key innovation of this module. It solves for the angle of attack (alpha) from a force balance equation, which is essential for trajectory planning.

### Newton-Raphson Solver

Solves the aerodynamic force balance equation:

```
S_ap_z * cos(alpha) + S_ap_x * sin(alpha) = F_aero_z([V*cos(alpha), 0, V*sin(alpha)]) / mass
```

where `V_b = [V*cos(alpha), 0, V*sin(alpha)]` is the body velocity vector.

### Usage Example

```cpp
#include <aerodynamics/aero_alpha_solver.hpp>

// Create solver with aerodynamics model
auto aero_model = std::make_shared<aerodynamics::LyuAerodynamics>();
aerodynamics::AeroAlphaSolver solver(aero_model);

// Optional: Configure solver parameters
aerodynamics::AeroAlphaSolver::Config config;
config.max_iterations = 50;
config.tolerance = 1e-8;
config.damping_factor = 1.0;
solver.setConfig(config);

// Solve for alpha
double V = 20.0;           // Velocity magnitude (m/s)
double S_ap_x = 0.5;       // Specific force x-component (m/s^2)
double S_ap_z = 9.81;      // Specific force z-component (m/s^2)
double mass = 2.5;         // Vehicle mass (kg)

double alpha = solver.solve(V, S_ap_x, S_ap_z, mass);
```

### Configuration

The solver supports configuration via `config/aero_alpha_solver.yaml`:

```yaml
max_iterations: 1000
tolerance: 1e-6
damping_factor: 0.5
derivative_epsilon: 1e-3
initial_alpha_deg: 90.0
```

### Solver Methods

1. **Lookup Table**: Binary search with linear interpolation (robust, always converges)
   - Used when `solver_method = "lookup_table"`
   - Data: `artifacts/simulations/aero_lookup_table.csv`

2. **Newton-Raphson**: Iterative solver with temporal coherence
   - Used when `solver_method = "newton_raphson"`
   - Fast convergence for smooth trajectories
   - Uses previous alpha value as initial guess

3. **Auto**: Automatically chooses the best method (recommended)
   - Used when `solver_method = "auto"`

### Features

- **Temporal Coherence**: Uses previous alpha value as initial guess for fast convergence
- **Angle Unwrapping**: Handles alpha continuity across trajectories
- **Stateful**: Non-thread-safe by design (each trajectory instance should have its own solver)
- **Diagnostics**: Returns iteration count for performance monitoring

## Integration Points

### ROS2/Gazebo Plugin

The `AerodynamicsPlugin` integrates aerodynamics into Gazebo simulation:

```cpp
#include <gazebo_plugins/AerodynamicsPlugin.hh>
```

### Global Planning (CzCalculate)

Used in trajectory planning via the `CzCalculate` class in `global_planning.cpp`.

### Multi-Backend Support

Data can be generated for different ML backends:

```bash
# Python backend data generation
just generate-mesh-data-jax 361
just generate-mesh-data-casadi
just generate-mesh-data-torch 361

# C++ lookup table generation
just generate-aero-table
```

## Data Flow

```
┌─────────────────────┐
│  Configuration DB   │
│  (aero_sim_params)  │
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│  Factory Creator    │
│  createAerodynamics │
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│  Aerodynamics Model │
│  (Lyu, Bspline, ...)│
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│  getAeroWrench()    │
│  (forces, moments)  │
└─────────────────────┘
```

## Coordinate Convention

All vectors use the **Forward-Right-Down (FRD)** body frame convention:
- **X**: Forward
- **Y**: Right
- **Z**: Down

## License

BSD 3-Clause License

Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.

## References

Paper: Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
