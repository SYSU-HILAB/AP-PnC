# Aerodynamics Module

The aerodynamics module provides a comprehensive framework for modeling aerodynamic forces and moments for tail-sitter UAVs in the AP-PnC (Autonomous Parking with Predictive Control) framework.

## Overview

This module implements multiple aerodynamics models with varying levels of fidelity, from simple linear models to complex MATLAB-generated implementations. It is designed for trajectory planning and tracking control, with a key innovation being the inverse calculation of angle of attack from lift coefficient.

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
- MATLAB-generated implementation from Zhang-Lyu aerodynamics
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
- Fit pipeline: `data/bspline_aero_fit.py` → npz → `data/ma_params_writer.py` → `config/aero/ma.yaml`

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

All vectors use the **Forward-Right-Down (FRD)** body frame convention:
- **X**: Forward
- **Y**: Right
- **Z**: Down

## License

BSD 3-Clause License

Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.

## References

Paper: Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.