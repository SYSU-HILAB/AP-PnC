# Tracking evaluation and NMPC reference plots

Status: the current controller is the [ABI3 command-integrator NMPC](nmpc_model.md).
Its [2 kg four-motor speed campaign](tracking_integrator_speed_report.md) has
11 status4 failures and one completed but severely divergent ma/6 run.
It does not establish a usable speed range. Actual alpha/beta and FLU force histories are recorded.
Actual horizon logging, terminal-point hold and the **historical ABI2** native ARM64
6/8/10/12 m/s five-model sweep are implemented. Read the historical
[speed + terminal-hold report](tracking_speed_report.md). The earlier
[6 m/s report](tracking_v6_report.md) preserves the historical 17.48 s truncated
window; new runs execute the whole reference plus 2 s dwell. New completion is
5/5 at 6, 4/5 at 8, and 0/5 at 10 and 12 m/s. This is not hard-speed-limit,
AMD64 or flight acceptance. Broader diagnostics/frame gates below remain pending.

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
attempt complete reference-supported trajectories only with the selected models:

- `none`: zero wrench control baseline (not a physics model).
- `lyu`: Zhang-Lyu model. **under test**
- `phi`: phi quadratic form. **under test**

`ma` (B-spline) and `advanced` remain compiled but are marked **not tested**:
they are rejected by the offline driver and excluded from the campaign until a
separate decision. The single source of truth is
`infra/sim_infra/aerodynamics/include/aerodynamics/aero_models.hpp`, and each run
records its own status in `manifest.json` as `aero_validation`.

Historical artifacts that used `ma` or `advanced` remain historical evidence and
must not be presented as current campaign results.

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

Current ABI3 cost layout is:

```
yref[0:3]   = position_W
     [3:6]  = velocity_W
     [6:9]  = body_Y_direction_W
     [9:13] = zeros: d[collective_specific_thrust, body_rate_command_xyz]/dt
terminal   = yref[0:9]
```

`cost_reference()` has no hover argument. There is no absolute command or
state-rate tracking target, and stage 0 DOES penalize the command derivative.
Actual command publication is `previous_command + u0_derivative / ctrl_frq`,
not u0 and not the future x1. Persist both operands/output in the solver trace;
verify command memory continuity and correspondence to `steps.csv`.
Older ABI2 files retain their 16/12 layout and zero-W0 semantics; never reinterpret
them using the new yb offset or claim they validate ABI3.

Do not conflate state-rate reference, command-rate reference and measured rate.
Do not silently map the legacy tangent acceleration to collective thrust.
A trajectory acceleration is a three-vector in W; aerodynamic X specific force
is a different quantity. Fix the definition before using a body-X scalar as a
reference/cost field.

Horizon-stage reference timestamps are `control_time + stage*traj_res_s`.
When this requested time exceeds the trajectory end, all reference fields repeat
the final point, matching the ROS buffer. `sample_time_ns` records the actual
clamped source time; `reference_time_ns` remains the requested prediction time.
Serialized references explicitly declare `terminal_policy: hold_last_point`.
Legacy datasets without this policy still reject unsupported extrapolation.
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
- Actual source samples stay inside the saved trajectory; future requests beyond
  its end explicitly use the final point, with both times logged.
- Each requested aerodynamic model either completes or has an explicit failure
  or unavailable-model explanation with evidence.
- Tracking figures show matched-time actual and reference channels.
- Improvement claims use a matched full window, model/profile and trajectory.

Deliver the scientific runs, offline replay where valid, figures and an HTML
report with conclusion, results, evidence and prioritized next actions. Upload
only an explicitly authorized summary report. Current OSS publishing is blocked
by `403 UserDisable`; do not claim a public URL until publication succeeds and
actual browser verification passes.
