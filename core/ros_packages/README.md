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

## Quadrotor frame and message contract

World vectors use **ENU**; body vectors use **FLU**; collective thrust acts
along **+Z body**. Rates and moments follow the right-hand rule about X/Y/Z.
The Hamilton quaternion `q_WB=[w,x,y,z]` rotates body vectors into world.

- `State.state`: position ENU [m], velocity ENU [m/s], body rates FLU [rad/s],
  then `q_WB`, starting at indices 0, 3, 6 and 9 respectively.
- `Control`: FLU body-rate setpoints and +Z specific force [m/s²].
- `ReferencePoint`: ENU position/velocity and desired body-Y **expressed in ENU**.
- `TrackingInfo`: position/velocity and body-Y directions expressed in ENU;
  `alpha=atan2(v_B.x,v_B.z)` and `beta=atan2(v_B.y,hypot(v_B.x,v_B.z))` [rad].
  `cx` is the IMU-derived, mass-normalized **body-X** coefficient [1/m].
  The controller assumes zero wind; aerodynamic plant models use air-relative
  velocity with the same body-FLU angle definition.

Rebuild `interface` and all publishers/subscribers together when changing a
message schema.

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

Build and run the probe in Docker (the helper sources the matching workspace):

```bash
uv run --project "$AP_PNC_DIR" ap-pnc build all --arch arm64
uv run --project "$AP_PNC_DIR" python -c 'from tooling import container_tools as c; c.run(["env", "ROS_DOMAIN_ID=213", "python3", "/workspace/test/integration/check_core_ros.py", "--ros-install", "/workspace/.artifacts/colcon/install"], arch="arm64")'
```

Use `--arch amd64` / `arch="amd64"` together for the other target.

It checks real NMPC command delivery, correctly labeled coefficient feedback,
wrong-grid rejection/recovery, feedback saturation, planner startup and mode
switching, and planner-to-NMPC horizon delivery. It saves process logs beneath
`$AP_PNC_DIR/.artifacts/tests/core-adapters/` and shuts down its own processes.

Build and test the Python module with the same Linux container interpreter:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc build planner-bindings --arch arm64
uv run --project "$AP_PNC_DIR" python -c 'from tooling import container_tools as c; c.run(["python3", "/workspace/test/integration/check_planner_bindings.py", "--bindings-dir", "/workspace/.artifacts/bindings"], arch="arm64")'
```

The host stores these Linux outputs under `.artifacts/docker-tools/arm64/`.
Do not import the generated module from host uv/Python.

The binding checks exercise real planning, default/explicit grids, exact endpoint
inclusion, finite arrays, circle boundary conditions and invalid input rejection.
Saved samples live under `.artifacts/tests/core-adapters/`, not the source tree.
Run `uv run --project "$AP_PNC_DIR" ap-pnc lint` and the CLI pytest suite as well.
