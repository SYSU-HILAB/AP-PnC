# Planner / NMPC adapter contract

Both packages use `src/` for ROS-free C++ and `node/` for ROS interfaces.
Add `python/` only for actual bindings; currently only planner needs it.
The solver wrapper, tracking controller and simulator share the same owner sources.

## Python trajectory export

`planner_bindings.plan(absolute_yaml_path, sample_dt=0.02)` plans a continuous
trajectory and evaluates `ReferenceTrajectory::sample(t)` directly. The returned
keys remain `t`, `p`, `v`, `a`, `yb`, `omega`, `thrust` and `duration`.

The export grid starts at zero, uses the requested interval, and includes the
exact terminal time once. The final interval can be shorter. An interval longer
than the trajectory returns both endpoints. Intervals must be finite and positive;
unrepresentable array sizes are rejected before allocation. Paths must be absolute.
Two separate planning calls can select different optimizer solutions.

`python/setup.py` compiles `src/trajectory.cpp` alongside the planner and shape
sources. This implementation replaces the removed stored `samples` table and
`duration` field; do not restore those obsolete APIs.

## ROS tracking feedback migration

`interface/TrackingInfo.cx` reports the current controller's mass-normalized,
IMU-derived **body-X** coefficient in 1/m. It replaces the obsolete `cz` field.
Do not fill a body-Z/lift field with an X-axis value or add a misleading `cz()`
alias just to make the node compile. Rebuild the interface package and every
publisher/subscriber after this schema change. `State.cur_cz` is a separate
legacy field and is not changed by this migration.

The ROS node also reads the configured excitation gate and regressor lag, so
its `TrackingConfig` matches the simulator's settings.

## Linux verification

Use the host-native Linux architecture, including Linux ARM64 Docker on Apple
Silicon. Never reuse macOS bindings or a different architecture's CMake cache.
All commands below require an absolute `AP_PNC_DIR` and the matching solver bundle.
Do not run the ROS probe in an aircraft's domain or a production control network.
Use a disposable container, host IPC, no external network, and ROS domain 213.
The probe sends synthetic sensor/reference messages and runs the real solver;
it does not arm a vehicle and does not establish flight acceptance.

Build `interface`, `planner` and `nmpc` with colcon, then source the ROS and
workspace setup scripts. The probe supports isolated and merged install trees:

```bash
ROS_DOMAIN_ID=213 PYTHONPATH="$AP_PNC_DIR:${PYTHONPATH:-}" python3 \
  "$AP_PNC_DIR/test/integration/check_core_ros.py" \
  --ros-install "$AP_PNC_DIR/.artifacts/colcon/install"
```

It checks real NMPC command delivery, correctly labeled coefficient feedback,
wrong-grid rejection/recovery, feedback saturation, planner startup and mode
switching, and planner-to-NMPC horizon delivery. It saves process logs beneath
`$AP_PNC_DIR/.artifacts/tests/core-adapters/` and shuts down its own processes.

Build and test the Python module with the same Python interpreter:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc build planner-bindings
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/test/integration/check_planner_bindings.py" \
  --bindings-dir "$AP_PNC_DIR/.artifacts/bindings"
```

The binding checks exercise real planning, default/explicit grids, exact endpoint
inclusion, finite arrays, circle boundary conditions and invalid input rejection.
Saved samples live under `.artifacts/tests/core-adapters/`, not the source tree.
Run `uv run --project "$AP_PNC_DIR" ap-pnc lint` and the CLI pytest suite as well.
