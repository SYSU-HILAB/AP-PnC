# Aerodynamics Module

The aerodynamics module provides a comprehensive framework for modeling aerodynamic forces and moments for tail-sitter UAVs in the AP-PnC (Autonomous Parking with Predictive Control) framework.

## Overview

This module evaluates aerodynamic forces, moments and air-relative angles directly in the quadrotor body-FLU convention. The simulator, Python bindings and offline fitting tools share that contract.

## Directory Structure

```
src/aerodynamics/
├── include/aerodynamics/         # Public headers
│   ├── aero_interface.hpp        # Abstract base interface
│   ├── zhang_lyu_aerodynamics.hpp   # MATLAB-generated model
│   ├── bspline_aerodynamics.hpp  # B-spline interpolation model
│   ├── phi_aerodynamics.hpp      # Linear phi-theory model
│   ├── advanced_lift_drag.hpp    # Advanced lift-drag model
│   └── non_aerodynamics.hpp      # Null model for testing
├── src/                          # Implementation files
│   ├── *_aerodynamics.cpp
│   ├── gazebo_plugins/           # ROS2/Gazebo integration
│   └── MatlabGenerated/          # Auto-generated MATLAB code
├── data/                         # Configuration and data
│   ├── bspline_fit_results/      # B-spline fit results (npz)
│   └── *.py                      # Data generation scripts (ma_params_writer.py)
├── config/
│   └── aero/                     # Per-model parameters (the single source
│       ├── phi.yaml              #   of truth; loaded at runtime by name)
│       ├── ma.yaml
│       └── advanced.yaml
├── scripts/
│   └── generate_aero_lookup_table.cpp
└── test/
```

## Core Architecture

### Base Interface

The module uses an abstract base class pattern with a shared factory:

- **`AerodynamicsInterface`**: Abstract base defining the interface for all aerodynamics models
  - Virtual method `getAeroWrench()` computes forces and moments
  - Returns aerodynamic force/moment vectors, alpha (angle of attack), and beta (sideslip angle)

- **`AerodynamicsCreator`**: Factory base class for creating aerodynamics instances
  - Implements the factory pattern for model creation
  - Loads parameters from `config/aero/<model>.yaml`

- **`make_aero(name)`** (`aerodynamics/aero_factory.hpp`): the single
  naming authority. Canonical names, identical everywhere (simple_sim yaml
  `aero.model`, gazebo `<aero_model>`):

  | name          | implementation            | parameters |
  | ------------- | ------------------------- | ---------- |
  | `lyu`         | Zhang-Lyu MATLAB model    | (baked in) |
  | `phi`         | linear phi-theory         | `config/aero/phi.yaml` |
  | `ma`          | B-spline fitted model     | `config/aero/ma.yaml` |
  | `advanced`    | advanced lift-drag        | `config/aero/advanced.yaml` |
  | `none`        | null model (zero wrench)  | — |

```cpp
#include <aerodynamics/aero_factory.hpp>

auto aero = aerodynamics::make_aero("phi");  // throws on unknown names
```

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
- Native FLU evaluation of the Zhang-Lyu fitted aerodynamic polars
- Comprehensive force and moment calculations
- Captures complex aerodynamic effects

### BsplineAerodynamics ("ma")

Uses B-spline curves to model force coefficients as functions of angle of attack and sideslip angle.

```cpp
#include <aerodynamics/bspline_aerodynamics.hpp>

// Factory creation
aerodynamics::BsplineAerodynamicsCreator creator;
creator.createAerodynamics();
```

**Features:**
- Cubic B-spline interpolation (order 3)
- Control points and knots loaded from `config/aero/ma.yaml`
- Efficient for real-time control
- Fit pipeline: `data/bspline_aero_fit.py` → stamped NPZ files under `.artifacts/aerodynamics/bspline_fit_results/` → `data/ma_params_writer.py` → `.artifacts/aerodynamics/ma.yaml`. Review the staged result before updating `config/aero/ma.yaml`.

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
- FLU Phi matrix loaded from `config/aero/phi.yaml`

### AdvancedLiftDrag ("advanced")

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
- Parameters: `config/aero/advanced.yaml` (baseline: PX4 quadtailsitter)

### NonAerodynamics (Null Model)

Returns zero forces and moments for testing or disabling aerodynamics.

```cpp
#include <aerodynamics/non_aerodynamics.hpp>

// Factory creation
aerodynamics::NonAerodynamicsCreator creator;
creator.createAerodynamics();
```


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
uv run ap-pnc aerodyn gen --type mesh --engine jax --size 361
uv run ap-pnc aerodyn gen --type mesh --engine casadi
uv run ap-pnc aerodyn gen --type mesh --engine torch --size 361

# ML pipeline (dataset assembly + model training)
uv run ap-pnc aerodyn ml pipeline
```

## Data Flow

```
┌──────────────────────┐
│  config/aero/*.yaml  │
│  (per-model params)  │
└──────────┬───────────┘
           │
           v
┌──────────────────────┐
│  make_aero(name)     │
│  (shared factory)   │
└──────────┬───────────┘
           │
           v
┌──────────────────────┐
│  Aerodynamics Model  │
│  (lyu, phi, ma, ...) │
└──────────┬───────────┘
           │
           v
┌──────────────────────┐
│  getAeroWrench()     │
│  (forces, moments)   │
└──────────────────────┘
```

## Coordinate Convention

World vectors use **ENU**; body vectors use **FLU** (X forward, Y left, Z up).
Collective thrust acts along **+Z body**; moments follow the right-hand rule.
`q_WB=[w,x,y,z]` is a Hamilton quaternion rotating body vectors into world.

Every model receives aircraft velocity **relative to air**, expressed in FLU:

```text
v_air_B = R_WB.T * (v_world - wind_world)
alpha   = atan2(v_air_B.x, v_air_B.z)
beta    = atan2(v_air_B.y, hypot(v_air_B.x, v_air_B.z))
```

Forces [N] and moments [N m] are returned directly in FLU. Angles [rad] describe
the input velocity, including low-speed flow; both are zero at rest. Pure lateral
flow has alpha zero and beta ±π/2. A model's low-speed force-law guard does not
change these reported kinematic angles. No model-specific body-frame adapter is
used by the plant or SE3 controller. The simulator's explicit
`lateral_moment_only` option retains only body-Y moment; it is a modeling option,
not a coordinate conversion. Paper GD fit families `CLL`, `CM`, `CN` describe
body Z, Y, X respectively; vector outputs are always ordered X, Y, Z.

The Python binding exposes `compute_wrench(v_air_B)` and zero-sideslip force
coefficients `(cx, cz) = (Fx/V², Fz/V²)`. These are **not mass-normalized**;
controller `TrackingInfo.cx` instead has units 1/m and includes measured actuator
contributions. CSV, parquet, fitted parameters and ML checkpoints carry a frame
stamp; incompatible or unstamped artifacts are rejected rather than relabeled.
One-time source-data import lives in `aerodynamics.data.import_flu`; inference
performs no output-axis conversion.

## License

Project-owned code uses the BSD 3-Clause License. The Zhang-derived kernel also
retains its upstream academic-license notice in the source file.

Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.

## References

Paper: Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
