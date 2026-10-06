# Coordinate-frame specification

Status: normative target; migration is incomplete. This document defines the
contract, not a claim that every legacy consumer already satisfies it.
All project paths below are anchored at the absolute `AP_PNC_DIR`.

## 1. Main-system contract

- World `W`: ENU, right handed, metres and seconds; gravity is `(0,0,-g)`.
- Main body `B`: FLU, right handed. Collective thrust acts along **+Z_B**.
- `R_WB` maps body components into world: `u_W = R_WB u_B`.
- Attitude quaternion: Hamilton, `wxyz`, representing `R_WB`.
- Body rates: `omega_B`, rad/s, right-hand rule about each body axis.
- `R_dot_WB = R_WB skew(omega_B)`.
- Quaternion sign is not physical: `q` and `-q` give the same body axes.
- The world representation of body Y is `yb_W = R_WB[:,1]`.
  **yb and -yb are not equivalent references.**
- Ground velocity `v_W` and reference acceleration `a_ref_W` are world vectors.
  Do not relabel either as a body-X scalar.
- Air-relative body velocity is
  `v_air_B = R_WB.T (v_W - wind_W)`.
  This is aircraft velocity relative to air, not the negated incoming-flow vector.
- Collective specific thrust is `a_T = F_T / mass`, N/kg, applied along +Z_B.
- The current NMPC adaptive aerodynamic contribution is the **lumped body-X**
  specific-force coefficient `cx`, scaled by `|v|^2` (ground speed, zero wind
  only). Because `+X_B = +Z_L`, that single axis is the legacy wing-normal
  (lift) direction; it deliberately also absorbs any actuator body-X component
  measured on the IMU. It is distinct from collective thrust and world
  reference acceleration.
- The NMPC is **mass-normalized**: `NMPC` needs no mass, no area and no inertia.
  Only the plant/model coefficients carry dimensional scaling.

These conventions apply to planner references, NMPC, simulation state/control,
scientific output and Rust/autodiff public interfaces. Rerun's `xyzw` layout is
only a quaternion storage-order conversion, not a physical frame change.

## 2. Legacy wing/body frame: simulation model boundary only

The old aerodynamic model expects a second **rigid-body component frame**, `L`.
The historical name `wing` does not imply an independently moving wing, a wind
frame, a world frame, or a second simulation state.

Define its axes solely by their relation to main body B:

| Legacy axis | Main-body direction |
|---|---|
| +X_L | +Z_B, the collective-thrust axis |
| +Y_L | -Y_B |
| +Z_L | +X_B |

```
C_LB = [0  0  1
        0 -1  0
        1  0  0]

u_L = C_LB u_B
u_B = C_LB.T u_L
```

`C_LB.T C_LB = I`, `det(C_LB)=+1`: this is a proper rotation, not a reflection.
Its vector rule is `(x,y,z)_L = (z,-y,x)_B`.
Forces, torques and angular velocities follow the same rule. Imported inertia
must use `J_B = C_LB.T J_L C_LB`; never just rename diagonal entries.

The legacy aerodynamic boundary is:

```
main simulation state (W/B)
  -> subtract world wind
  -> R_WB.T: world to main body
  -> C_LB: main body to legacy model input
  -> legacy aerodynamic function
  -> C_LB.T: legacy force AND torque to main body
  -> main-body rigid-body dynamics
```

Apply each conversion exactly once. Use explicit names such as
`v_air_body_flu`, `v_air_body_legacy`, `force_body_legacy`, `force_body_flu`.
Do not pass legacy outputs directly into B rigid-body dynamics.

Legacy full aerodynamic functions and their `L` conversion are **simulation
plant mechanisms**. Planner/NMPC/autodiff do not call these vendor functions or
import their coordinate adapter. They may retain a simplified prior-free model
expressed directly in B. A simulation-only motor adapter may likewise convert
private legacy motor geometry/PID parameters at its boundary; this does not put
legacy actuator states or frames into NMPC.

Do not import MATLAB NED world position/attitude as ENU without a separate
world-and-body conversion. The current numeric-parameter importer does not
reuse its world trajectory or initial attitude.

## 3. Planner/autodiff migration contract

New flatness derivation must construct `R_WB=[xb_W,yb_W,zb_W]` directly, with
`zb_W` the +Z thrust direction and determinant +1. Define collective specific
thrust separately from tangent acceleration `a_t = vhat_W dot a_ref_W`.

Derive rates from the new rotation's time derivative:

```
R_dot_WB = dR/dv_W * a_ref_W + dR/da_ref_W * jerk_ref_W
omega_B = vee(R_WB.T R_dot_WB)
```

Specify gravity, any aerodynamic coefficient and the wind assumption explicitly.
Specify low-speed/projected-acceleration branch policies; do not claim smooth
Jacobians at switching surfaces or exact dynamics for an arbitrary fallback.

Generate value/Jacobian/direct-VJP kernels in build preparation only. Runtime
kernels remain C/libm with per-call workspace and safe fixed-size Rust FFI.
Finite-difference and kinematic tests are required in addition to legacy golden.

The existing `tailsitter_df` four-output prototype is NOT this complete ABI:
its scalar is tangent acceleration and its angular outputs mix spatial X/Z
with a body-Y term. Legacy golden passing does not certify those as FLU rates.
Changing the physical output meaning requires a distinct documented interface,
not silent relabelling. The active optimizer/trajectory consumers still require
an explicit migration audit.

## 4. Reference contract: no downstream yb sign fix

The new planner's public `yb` is `yb_W` from main `R_WB`. NMPC must copy it
unchanged into the body-Y direction reference; it must not negate, permute,
reconstruct or apply `C_LB` to it. It is a direction in WORLD, not a vector of
body components to pass through a body-frame adapter.

Implemented public producer: `planner/core/src/trajectory.cpp` calls the direct
ENU/FLU `flu_reference()` and sets `rp.yb = rp.rotation.col(1)` without a sign
adapter. The independent 13-output CasADi/C/Rust ABI returns collective +Z
specific thrust, physical FLU omega and row-major R_WB. Its C++ oracle, finite
differences, force balance and Rdot=R*skew(omega) checks are recorded separately
from the preserved legacy four-output optimization ABI.

Still pending: migrate MINCO's legacy mixed-frame optimization outputs/costs;
complete imported-geometry dynamic impulse and all guard/reverse/transition
acceptance. The new low-speed/projection fallback is deterministic and
branchwise, not globally smooth. Public physical rates/thrust are currently
logged but do not replace NMPC's zero rate / hover command cost references.

## 5. Mandatory sign/branch acceptance

1. Test all three basis vectors through the simulation adapter, both directions.
2. Test non-identity yaw/pitch/roll attitudes and nonzero world wind.
3. Test force/torque separately; preserve norms and dot-product work invariance.
4. Test `q` versus `-q`: same R and same yb, not inverted yb.
5. Test actual imported motor geometry with positive and negative B-axis inputs.
6. Test planner yb against R's Y column and against the exact NMPC yb input.
7. Sweep both sides of low-speed and projected-acceleration branch guards.
8. Test curves, reversed velocity, hover/transition and legitimate heading turns.
9. Diagnose any abrupt yb reversal against expected finite rotation and branch
   policy; a sign flip is not excused as quaternion double cover. Do not outlaw
   a legitimate large physical turn by a blanket `dot(yb_k,yb_k+1)>0` rule.
10. Check `R.T R=I`, determinant +1, +Z thrust, and omega from finite-difference R.

Target tracking evaluation starts at planner `v_max=6 m/s`; see
[tracking_evaluation.md](tracking_evaluation.md). Successful file export or a
static frame check is not full dynamic-frame or tracking acceptance.
