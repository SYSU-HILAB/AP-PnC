# Local MatrixCity mirrored-ring experiment

For copy-and-run setup, see [Start the 3DGS demo](CITY_DEMO_START.md).
For the demonstration video and image attribution, see [Media](CITY_DEMO_MEDIA.md).

**Linux ARM64 acceptance passed (2026-10-18): two uninterrupted 10 m/s practical
simulation runs, plus a successful browser pause/resume run.** This is not PX4,
Gazebo, real-aircraft acceptance, or a claim that every future run passes. The UI
shows a green acceptance banner only when the independent receipt matches the
current completed recording. Resetting cannot reuse the previous run's pass.

This local-only experiment requires the authorized MatrixCity Gaussian asset and
practical actuator profile/provenance. It downloads neither. Private inputs and
generated models are not part of a fresh public checkout.

## Scene and continuous route

- `prepare_city_ring.py` mirrors the northwest curved-road quadrant into four
  quadrants. It transforms positions, covariance orientation, normals and SH
  parity consistently. The original MatrixCity asset is untouched. This is a
  **synthetic mirrored scene**, with visible seams and reconstruction gaps, not
  an original surveyed circular street.
- The prepared scene uses 40 metres/source-unit and **no further enlargement**.
  The route radius is 88.8 m, about 558 m around. Scale remains a visual assumption.
- `tooling.city_demo` defaults to this ring, at world Z=12 m. The asset generator's
  earlier Z=10 m proposal intersected the inflated map near a seam; that failed
  trial is preserved. Raising the route 2 m retained the same road and map scale.
- Sixteen waypoints define fifteen asynchronously planned legs. Initial and final
  velocity/acceleration are zero. Intermediate velocities are tangential 10 m/s;
  accelerations are centripetal, magnitude `100 / 88.8` m/s². They do not reset
  to zero at every segment. The first 22.5 degrees accelerate; the last 45 degrees
  decelerate on return. There are **no intermediate reference holds**.
- The reference has an initial 8 s hold and a final 5 s hold. Completion occurs
  after 3.5 s of terminal hover, once return position/speed tolerances pass.
  Future C2 appends preserve the active NMPC horizon, plant/motor state, command
  integrator and simulation clock.
- This explicitly selects `problem_formulation.map.speed_controlled`: shared
  `planner_core` builds a quintic PVA connector and checks inflated occupancy,
  an FIRI cover of the actual curve, speed, acceleration and thrust. It is **not
  GCOPTER optimization**, nor an implicit fallback after failed optimization.
  Ordinary GCOPTER remains the default elsewhere.
- The map uses 0.5 m voxels and four-cell/2 m dilation. Native checks reject both
  invalid references and occupied actual plant positions. The independent Python
  verifier reconstructs the same 26-neighbour dilation from frozen occupancy.
  This is a static Gaussian-derived surface map, not live LiDAR, complete solid
  geometry, contact physics or dynamic-obstacle avoidance.

## Controller change and evidence boundary

The successful configuration keeps the **baseline NMPC bundle and objective**.
The physical motor constants, 0.076 s motor lag, mixer, inertia, Lyu aerodynamics,
INDI and authorized source profile are unchanged.

The local mission overrides only **P and D gains on body-FLU Z**, multiplying
both by three (`--rate-tuning body-z-pd3`). I and both other axes stay
unchanged. This is controller tuning, **not a new measured motor calibration**.
The source profile is copied unchanged; the override is visible in frozen
`config/simple_sim.yaml` and `mission-metadata.json` (`inner_rate_tuning`). It is
not promoted into the project's global simulator or solver configuration.

The baseline ring run developed growing body-Z rate oscillation and stopped at
42.28 s. The same route with the P/D override completed; its short final braking
arc still reached 2.169 m position error and was not accepted. Extending only the
final braking arc to 45 degrees reduced maximum error to 0.565 m. Increasing NMPC
command-derivative costs instead did not solve this task. These comparisons
support an inner-loop response mismatch; they do not establish a complete
frequency-domain mechanism or real-aircraft robustness margins.

Use `--rate-tuning baseline` for an explicit ablation. `--scene original` retains
the earlier enlarged rounded rectangle, with baseline tuning by default; its
high-speed failure is not relabelled as a success.

## Dual body-fixed video streams

`web/camera_rig.json` specifies **assumed extrinsics, not hardware calibration**.
The tail-sitter nose/thrust axis is `+Z_FLU`, its belly is `+X_FLU`.

| Camera | Body position [m] | Optical forward | Image up |
|---|---|---|---|
| Nose/top | `[0,0,0.6]` | `+Z_FLU` | `-X_FLU` |
| Belly/front | `[0.18,0,0.05]` | `+X_FLU` | `+Z_FLU` |

Both use `T_WC = T_WB × T_BC`, including lever arms, from the actual plant
quaternion. There is no horizon lock. Each has an independent Spark renderer,
640×360 render target and Canvas MediaStream track, with 75° vertical FOV. Both
share an interpolated committed pose; neither extrapolates. Sensor images hide
mission overlays. External overview/chase/orbit cameras remain separate.

A browser/CSV cross-check at t=61.66 s in the pause/resume run found actual speed
9.737 m/s, nose tilt 74.27 degrees from vertical, two live 640×360 tracks and
approximately 11 FPS. Maximum body/lever-arm/optical-axis/image-up discrepancy
was `6.99e-15`. The pause froze simulation time and pose without replacing
velocity with zero. Evidence is under `.artifacts/dev-sim/city-ring/`:
`tilted-camera-debug.json`, `tilted-dual-camera.png`, `camera-verification.json`.

## Build, launch and verify

All paths below are anchored at the absolute project root. Numerical execution,
ROS and JS tests run inside native Linux containers; `uv` only stages/orchestrates.

- `.artifacts/dev-sim/city-ring/assets/`: synthetic PLYs and original proposal metadata.
- `.artifacts/dev-sim/city-loop/assets/`: selected viewer configuration/stream links;
  `ring-city.ply` links to the mirrored scene without overwriting the old scene.
- `.artifacts/dev-sim/city-loop/build/`: native Linux CMake outputs.
- `.artifacts/dev-sim/city-loop/jobs/<id>/`: frozen inputs, executable snapshot,
  hashes, exact polynomial reference, CSV, plan events and verification receipts.

Build `simple_sim` inside the matching native tools image with
`SIMPLE_SIM_WITH_ROS=ON`, `SIMPLE_SIM_WITH_NMPC=ON`, `BUILD_TESTING=ON`. Build all
test targets before running all CTests; mount a writable artifact benchmark
output. Do not reuse caches or static bundles across architectures. The accepted
executable and all 250 members of its bundle were verified as AArch64.

After preparing the mirrored scene in Docker:

```bash
PYTHONPATH="$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -m tooling.city_demo
# Use the absolute job path printed above:
PYTHONPATH="$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -m tooling.city_demo \
  --launch "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/jobs/<id>"
```

Launch is paused, with host IPC, `--network none`, localhost-only DDS and ROS
domain 213. Select only one mission at a time. Launch snapshots the executable
and hashes the frozen inputs. Relaunch refuses changed inputs/binaries; prepare
a new job instead. Rebuild statically linked consumers after solver changes.

The viewer is **http://127.0.0.1:5178**. Stage web sources and pinned dependencies
under `.artifacts/dev-sim/city-loop/web`. Its Node 22 container needs both the
city-loop directory and a read-only city-ring mount at their `/workspace` paths.
Expose port 5178 on host loopback only. The browser observes actual simulation;
it is not `flight.json` playback.

After completion, run in the selected mission container:

```bash
docker exec -e PYTHONPATH=/workspace -e PYTHONDONTWRITEBYTECODE=1 \
  ap-pnc-city-<id> python3 \
  /workspace/infra/sim_infra/lidar_sim/scripts/verify_city_run.py \
  /workspace/.artifacts/dev-sim/city-loop/jobs/<id>
```

The verifier checks completed status, every solver status, CSV/reference identity,
C2 joins, future-only planning, map membership, ring deviation, no intermediate
dwell, cruise/reference speeds and terminal hover. Maximum position error must
stay below 2 m; acceptance was not weakened to admit the first braking trial.
Receipts are saved in both the recording and the job. The UI requires matching
recording and stream identifiers, so an old receipt cannot approve a reset run.

## Accepted runs and retained failures

Two uninterrupted runs passed independently:

| Job | Recording |
|---|---|
| `1792327913708314000` | `simple_sim_1792327925165603_0` |
| `1792328168423913000` | `simple_sim_1792328216024403_0` |

A browser reset/pause/resume repeat in the second job also passed:
`simple_sim_1792328460401298_0`. Simulation-time trajectory/speed metrics are
identical across these deterministic runs; wall planning times differ.

| Metric | Result |
|---|---|
| Simulation duration / committed steps | 77.36 s / 3,868 |
| Online plans | 15 |
| Cruise actual speed min / mean / max | 9.583 / 10.007 / 10.567 m/s |
| Cruise reference speed | 9.99999936–10.00000000 m/s |
| Position RMSE / maximum | 0.133 / 0.565 m |
| Actual maximum radial deviation | 0.783 m |
| Final position error / speed | 0.000217 m / 0.001004 m/s |
| Nonzero solver statuses | 0 |
| Independent map checks | 15,643 positions, all outside inflated occupancy |
| Saved-reference discrepancy / C2 join error | `7.56e-14` m / `6.17e-13` |

Failures remain in their own jobs, including the original rectangle, the first
10 m-height ring, the baseline 12 m ring (`1792326964544025000`), isolated solver
cost trials (`1792327230090287000`, `1792327457371190000`), and the excessive
braking-error trial (`1792327718416362000`). No failed solver variant is promoted.

Regression checks: 10 native CTests; full Linux Python 3.12 CLI suite **156 passed,
12 skipped** (including 23 city/dev-sim and 18 synthetic geometry/integrity tests);
four camera tests; Ruff/lint, JS syntax and diff checks. Initial test-container
mount/dependency failures were corrected without changing unrelated source code.
The new ring workflow has **not** been revalidated on AMD64 or real hardware.
