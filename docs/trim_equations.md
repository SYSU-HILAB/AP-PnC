# Aircraft Trim Analysis

## Overview

Trim analysis finds steady-state flight conditions where all forces and moments are balanced. For a tailsitter UAV in level flight, we solve for the equilibrium state where the aircraft maintains constant velocity with zero acceleration.

---

## Problem Definition

### Flight Conditions

- **Coordinated flight**: Sideslip angle β = 0
- **Level flight**: Pitch angle θ equals angle of attack α
- **Constant velocity**: V = target velocity (no acceleration)

### Unknown Variables (3)

| Variable | Symbol | Description |
|----------|--------|-------------|
| Velocity magnitude | V | Total airspeed (m/s) |
| Angle of attack | α | Angle between velocity vector and body x-axis |
| Thrust | T | Propeller thrust force (N) |

---

## Equilibrium Equations

### Aerodynamic Force Form

Aerodynamic forces in the **body frame** are modeled as velocity-squared scaling with angle-dependent coefficients:

```
F_aero_x(α, V) = cx(α) · V²    # body x-axis
F_aero_z(α, V) = cz(α) · V²    # body z-axis
```

The functions `cx(α)` and `cz(α)` encapsulate the aerodynamic coefficient model (computed by the `aerodynamics_B_vb` module), which returns forces directly in the body frame.

### 1. Force Balance (X-axis)

```
R_Fx = cx(α) · V² + T + W_x = 0
```

Where:
- `cx(α) · V²` = Aerodynamic force in body x-direction
- `T` = Thrust (assumed along body x-axis)
- `W_x = -mass · g · sin(θ) = -mass · g · sin(α)` (weight component in x)

### 2. Force Balance (Z-axis)

```
R_Fz = cz(α) · V² + W_z = 0
```

Where:
- `cz(α) · V²` = Aerodynamic force in body z-direction
- `W_z = mass · g · cos(θ) = mass · g · cos(α)` (weight component in z)

### 3. Velocity Constraint

```
R_V = V - V_target = 0
```

Ensures the trimmed velocity matches the target velocity.

---

## Summary of Trim Equations

```
┌────────────────────────────────────────────────────────────────────┐
│                     TRIM EQUATIONS (β = 0)                         │
│                     (All forces in body frame)                     │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  cx(α)·V²  +  T           -  mass·g·sin(α)    =  0   (X-force)    │
│                                                                    │
│  cz(α)·V²      +  mass·g·cos(α)            =  0   (Z-force)    │
│                                                                    │
│  V            -  V_target                  =  0   (constraint)    │
│                                                                    │
│  where cx(α), cz(α) are body-frame force coefficients             │
│        provided by the aerodynamics module                        │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

---

## Algorithm: Levenberg-Marquardt Trim Solver

### Pseudocode

```
FUNCTION aircraft_equations(trim_vars, target_V, mass, g):
    """
    Computes residuals for the trim problem.

    Args:
        trim_vars: [V, alpha, T]
        target_V: Desired velocity
        mass: Aircraft mass
        g: Gravity constant

    Returns:
        [R_Fx, R_Fz, R_V]
    """
    # Unpack trim variables
    V      ← trim_vars[0]
    alpha  ← trim_vars[1]
    T      ← trim_vars[2]

    # Coordinated flight assumption
    beta ← 0.0

    # === Step 1: Construct body-frame velocity ===
    Velocity_body ← [
        V * cos(alpha) * cos(beta),   # u
        V * sin(beta),                 # v
        V * sin(alpha) * cos(beta)    # w
    ]

    # === Step 2: Compute aerodynamics ===
    FORCES, _ ← aerodynamics_model(Velocity_body)
    F_aero_x ← FORCES[0]
    F_aero_z ← FORCES[2]

    # === Step 3: Compute weight components (level flight) ===
    theta ← alpha  # Level flight: pitch = angle of attack
    W_x ← -mass * g * sin(theta)
    W_z ←  mass * g * cos(theta)

    # === Step 4: Compute residuals ===
    R_Fx ← F_aero_x + T + W_x
    R_Fz ← F_aero_z + W_z
    R_V  ← V - target_V

    RETURN [R_Fx, R_Fz, R_V]
END FUNCTION
```

---

## Aerodynamic Model Integration

### Velocity to Angle Conversion

```
alpha ← atan2(v_bz, v_bx)      # Angle of attack
beta  ← asin(v_by / |V_b|)     # Sideslip angle
V²    ← v_bx² + v_by² + v_bz²  # Velocity squared
```

### Aerodynamic Coefficients (Body Frame)

The functions `cx(α)` and `cz(α)` are provided by the aerodynamics module (`aerodynamics_B_vb`), which returns forces and moments directly in the **body frame**:

```
FORCES_BODY, _, _, _ ← aerodynamics_B_vb(V_body)

cx(α) ← FORCES_BODY[0] / V²    # X-force coefficient (body frame)
cz(α) ← FORCES_BODY[2] / V²    # Z-force coefficient (body frame)
```

The aerodynamic module handles all internal transformations (stability frame → body frame) and coefficient computations.

---

## Batch Trim Algorithm

For computing trim conditions across a velocity range efficiently:

### Pseudocode

```
PROCEDURE batch_trim(target_velocities, mass, g):
    """
    Vectorized trim computation using JAX vmap.

    Args:
        target_velocities: Array of N target velocities
        mass: Aircraft mass (scalar)
        g: Gravity (scalar)

    Returns:
        Array of N trim solutions [V, alpha, T]
    """
    N ← length(target_velocities)

    # === Step 1: Create batch initial guesses ===
    init_V      ← target_velocities                    # Match target
    init_alpha  ← deg_to_rad(50.0) * ones(N)           # Constant guess
    init_T      ← 10.0 * ones(N)                       # Constant guess

    init_params ← stack([init_V, init_alpha, init_T], axis=1)

    # === Step 2: Setup solver ===
    solver ← LevenbergMarquardt(residual_fun=aircraft_equations)

    # === Step 3: Vectorize solver across batch ===
    FUNCTION solve_single(init_p, t_V):
        RETURN solver.run(init_p, target_V=t_V, mass=mass, g=g)

    batch_solver ← vmap(solve_single)

    # === Step 4: Solve all trim conditions ===
    solutions ← batch_solver(init_params, target_velocities)

    RETURN solutions.params, solutions.residuals
END PROCEDURE
```

---

## Summary of Trim Algorithm

```
┌─────────────────────────────────────────────────────────────┐
│                    TRIM ALGORITHM                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. INPUT: target_V, mass, g                                │
│                                                             │
│  2. INITIALIZE: [V≈target, α≈50°, T≈10N]                    │
│                                                             │
│  3. ITERATE (Levenberg-Marquardt):                          │
│     │                                                        │
│     ├─→ Construct V_body from [V, α, β=0]                  │
│     │                                                        │
│     ├─→ Compute cx(α), cz(α) via aerodynamics model        │
│     │                                                        │
│     ├─→ Evaluate residuals:                                 │
│     │   R_Fx = cx(α)·V² + T - mass·g·sin(α)                 │
│     │   R_Fz = cz(α)·V² + mass·g·cos(α)                     │
│     │   R_V  = V - target_V                                 │
│     │                                                        │
│     └─→ Update [V, α, T] to minimize ‖R‖²                  │
│                                                             │
│  4. OUTPUT: Trimmed [V, α, T]                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Implementation Details

1. **JAX JIT Compilation**: The `aircraft_equations` function is JIT-compiled for performance
2. **Batch Processing**: `jax.vmap` enables solving thousands of trim conditions in parallel
3. **Aerodynamic Model**: Uses logistic-weighted polynomial regression for coefficients across full α range (-180° to 180°)
4. **Stability Frame Transform**: Aerodynamic forces computed in stability frame, then transformed to body frame

---

## References

- **File**: `test/lu_mpc/analysis/user_test_jax_trim.py`
- **Aerodynamics**: `feature/lu_mpc/model/sim/gd_model_jax.py`
- **Solver**: `jaxopt.LevenbergMarquardt`
