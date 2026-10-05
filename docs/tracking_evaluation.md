# Tracking evaluation and NMPC reference plots

Status: requested acceptance contract. The new multi-model sweep and complete
reference instrumentation are not yet implemented/accepted. Existing evidence
covers short successful runs and failed full Lyu-aero runs, not this sweep.

Follow [coordinate_frames.md](coordinate_frames.md). Use absolute project-root
paths for all inputs and `.artifacts/benchmark/` or the offline stack job trees
for generated scientific output. Do not publish private upstream source or
raw numeric profiles with a report.

## 1. First gate: v_max = 6 m/s

Freeze the planner configuration with `problem_formulation.cost.v_max=6.0`.
This is the planner bound, not a promise the actual aircraft never exceeds 6.
Compute the saved polynomial's maximum/reference speed and separately measure
actual speed. Show both; do not call a run v_max=6 merely by its label.

First complete coordinate/yb acceptance and a `none`-aero control case, then
attempt complete reference-supported trajectories with each canonical model:

- `none`: no aerodynamic force or moment.
- `lyu`: Zhang-Lyu model.
- `phi`: coefficient model.
- `ma`: spline/model approximation.
- `advanced`: advanced lift/drag model.

If a model requires absent parameters, identify it as unavailable rather than
substitute another model silently. Load each model's owned configuration.

Use one saved planned trajectory for plant-model isolation, one physical plant
profile, equal initial conditions, controller horizon/weights/bounds/timing,
wind, duration and sampling. Separate the ideal-rate-loop baseline from the
four-motor plant. Compare only matched groups; report mass/profile differences
explicitly. Do not retune each row invisibly to manufacture improvement.

The current pure NMPC coefficient estimator uses world ground speed, while the
simulation aerodynamic adapter uses air-relative speed. Initial comparisons
must therefore use zero wind. A nonzero-wind experiment needs a separately
specified predictor/estimator wind contract, not merely a plant wind switch.

## 2. Log references that the solver actually receives

Persist both the planner reference and the exact solver objective reference.
Do not reconstruct solver yref afterwards from a nicer trajectory curve.
For every control tick and horizon stage, record:

- control tick, control time, stage index, and stage reference time;
- actual `yref` fields in ABI order;
- terminal reference separately, with its actual dimension;
- reference position and velocity in W, rates in B, body Y direction in W;
- command reference units and distinction from actual u0;
- source planner acceleration/jerk and any derived body-X acceleration,
  with an explicit mathematical definition if included;
- initial solver state, parameter values and solve status;
- per-stage effective weights, bounds/grid, or a pinned generated-OCP snapshot;
- whether the numerical period committed, failed, or never ran.

Current cost layout is:

```
yref[0:3]   = position_W
     [3:6]  = velocity_W
     [6:9]  = body_rate_B
     [9:12] = body_Y_direction_W
     [12]   = collective_specific_force_sp [N/kg]
     [13:16]= body_rate_sp_B [rad/s]
terminal   = yref[0:12]
```

Audit these meanings against the generated solver before plotting. Log actual
values even when they are zero or constant. Currently `cost_reference()` fills
rate references with zero and the collective command reference with hover;
it does NOT use planner omega or its legacy scalar called thrust. Stage 0 cost
has zero W in the current OCP. Zero-weight references can be plotted, but must
not be described as active tracking targets.

Do not conflate state-rate reference, command-rate reference and measured rate.
Do not silently map the legacy tangent acceleration to collective thrust.
A trajectory acceleration is a three-vector in W; aerodynamic X specific force
is a different quantity. Fix the definition before using a body-X scalar as a
reference/cost field.

Horizon-stage reference timestamps are `control_time + stage*traj_res_s`.
Do not overlay stage-1 future reference against current actual state as though
both belonged to the same time. Use stage-0 for same-time overlays and show
future stages as labeled horizon traces. The terminal knot has its own time.

## 3. Required figures, emphasizing NMPC reference

### Reference audit (first figure, before tracking claims)

- Planner p/v and logged solver p/v yref: exact same-time overlay/difference.
- Planner yb_W, solver yb_W, actual `R_WB[:,1]`: x/y/z component overlays.
- yb norm; planner-to-solver vector mismatch; reference-to-actual direction angle.
- yb continuity/branch events; quaternion q/-q invariance check results.
- Solver state-rate and command-rate references versus planned physical B rates
  only when those planned rates have passed the new frame audit.
- Actual collective/rate command references, including constant hover/zero values.
- Relevant stage weights: distinguish plotted-but-unweighted targets.

### Tracking for every available model

- 3D ENU reference/actual paths, same axes and scale.
- Position xyz, velocity xyz and speed: reference versus actual.
- Position/velocity error curves, cumulative and complete-run RMSE/max.
- Body rates xyz versus desired rates and solver state-rate reference.
- World Y-axis components and direction-error angle.
- Collective specific force command/reference versus applied force per mass.
  Label net IMU Z separately from actuator force Z; aero may contribute to IMU.
- RPM/targets/duty, PID integrals/errors, saturation flags and event timestamps.
- Aerodynamic force/moment and actuator force/moment, all in B, separate channels.
- Solver status and solve time; failed solve/context and available QP diagnostics.

### Cross-model dashboard

Summarize completion, actual model/profile/hash, steps/duration, complete-run
position and velocity RMSE, max error, rate/yb error, saturation and solver
failures. Include ideal/motor categories, not a mixed ranking.

## 4. Scientific-data and failure policy

CSV/YAML/config/reference/manifest/metrics are authoritative. Rerun reads only
saved completed data, with simulation time; no planner/plant rerun in export.
New reference traces require schema, finite/time/horizon checks and source hashes.
Show actual recorded data; never fill gaps with another run's reference.

Preserve failed/interrupted raw data. A failed full trajectory is not comparable
to a successful full trajectory by its truncated RMSE. Mark partial plots and
stop time prominently; no fabricated final state or successful RRD receipt.
If partial-failure figures are needed, use an explicitly separate diagnostic
renderer/schema. Do not weaken the completed-run exporter to accept failures.

Perform repeated matched runs for deterministic numerical reproducibility,
excluding wall-clock solve time. Independent RRD decoder/footer verification
is required for a successful offline replay but is not control acceptance.

## 5. Completion gate and delivery

- Main-frame definitions and planner->NMPC yb identity tests pass.
- No unresolved unexplained yb flips remain in the v_max=6 reference.
- Solver references are logged, not inferred, including effective cost weights.
- Reference horizon stays inside the same saved trajectory.
- Each requested aerodynamic model either completes or has an explicit failure
  or unavailable-model explanation with evidence.
- Tracking figures show matched-time actual and reference channels.
- Improvement claims use a matched full window, model/profile and trajectory.

Deliver the scientific runs, offline replay where valid, figures and an HTML
report with conclusion, results, evidence and prioritized next actions. Upload
only an explicitly authorized summary report. Current OSS publishing is blocked
by `403 UserDisable`; do not claim a public URL until publication succeeds and
actual browser verification passes.
