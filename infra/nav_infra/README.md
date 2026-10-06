# nav_infra — base-services sidecar

Single self-contained unit providing the generic flight interface. NOT
research code: the core container (`core/ros_packages/`: planner line,
nmpc) consumes these services over the shared host-network ROS 2 graph.

## Contents

| Package           | Role                                                                |
| ----------------- | ------------------------------------------------------------------- |
| `px4ctrl`         | FCU interface + L2 control: arming/offboard/takeoff/hover/land, sole `/mavros/setpoint_raw/attitude` publisher, state adapter (`/px4ctrl/state`, `/px4ctrl/state_odom`, `/px4ctrl/odom_world`). Controller modes: `ctrl_mode=0` tracking, `ctrl_mode=1` NMPC rates + RLS eta/INDI thrust from `/nmpc/control` |
| `quadrotor_msgs`  | `PositionCommand` / `TakeoffLand` / `Px4ctrlDebug` (px4ctrl contract) |
| `livox_ros_driver2` | Mid-360 driver (real profile)                                    |
| `fast_lio`        | FAST-LIO2 (real profile)                                             |
| `ekf_quat_pose`   | odom fusion → /ekf_quat/ekf_odom (real profile)                      |

Plus a from-source mavros2 build (the jammy repo ships `mavros-msgs` only),
installed under `/opt/mavros` inside the image.

px4ctrl natively provides the odometry state adapter (`/px4ctrl/state*`,
`/px4ctrl/odom_world`), the RLS/INDI throttle law
(`include/rls_throttle_model.h`, `ctrl_mode=1` path), and the throttle
telemetry on `/px4ctrl/throttle_model_status`.

## Profiles (same image)

```
sim  : mavros (udp://:14540@localhost:14580) + px4ctrl
real : sim + livox + fast_lio + ekf_quat   (px4ctrl odom := /ekf_quat/ekf_odom)
```

Selection: `ros2 launch bringup nav_infra.launch.py profile:=sim|real`
(compose command override, or the `NAV_PROFILE` key in the k3s ConfigMap).

## Build

```bash
docker compose -f infra/sim_infra/gazebo/docker/docker-compose.yml build nav_infra
# behind a proxy: add --build-arg https_proxy=http://<docker0-ip>:<port> ...
```

## Topic contract with the core

| Direction | Topic                    | Type                      | Note                          |
| --------- | ------------------------ | ------------------------- | ----------------------------- |
| → core    | `/px4ctrl/state`             | interface/State           | ENU world, FLU body (50 Hz)   |
| → core    | `/px4ctrl/odom_world`    | nav_msgs/Odometry         | world-frame twist (px4ctrl)    |
| ← core    | `/nmpc/control`          | interface/Controls        | rates_sp FLU + specific_force_sp |
| ← core    | `/setpoint_cmd`          | quadrotor_msgs/PositionCommand | tracking controller input |

px4ctrl (nav_infra container) owns arming/offboard through the mavros
services and is the single publisher of /mavros/setpoint_raw/attitude.
Handover to the remote-rates source is a synchronous service on px4ctrl:

```bash
ros2 service call /px4ctrl/toggle_pass_through std_srvs/srv/SetBool "{data: true}"   # go
ros2 service call /px4ctrl/toggle_pass_through std_srvs/srv/SetBool "{data: false}"  # back to hover
```

`go` uses the internal NMPC controller when `ctrl_mode=1` (px4ctrl
subscribes `/nmpc/control` itself and applies the RLS eta + INDI thrust
law) or forwards an external rates stream when `ctrl_mode=0`. In
PASS_THROUGH px4ctrl publishes at its ctrl rate and falls back to
AUTO_HOVER when the source goes stale (`msg_timeout.pass_through`).

## Tracking controller layout (`px4ctrl/src/controller/`)

`Controller` (`src/controller.h`) is a facade: it owns the shared modules
plus one `pose_solver` strategy, and the FSM calls `update()` only.
Each solver owns independent gains and filter state:

| `pose_solver` | Class | Algorithm |
| ------------- | ----- | --------- |
| 0 | `Alg0DifferentialFlatness` | differential flatness with rotor-drag feedforward (Zhepei Wang alg0) |
| 1 | `Alg1Geometric` | geometric tracking on SE(3) (Zhepei Wang alg1) |
| 2 | `Alg2RotorDrag` | rotor-drag-compensated tracking (rotor-drag paper) |

Shared modules: `PidPosition` (position-to-acceleration law),
`AttitudeFeedback` (quaternion-error to body-rate feedback),
`ThrustLimiter` (collective-acceleration magnitude/tilt gate plus the
body-rate angular-acceleration limiter), `YawTarget` (bounded shortest-error
yaw regulator for `YAW_CONTROL_TARGET` mode), `ThrottleManager` (legacy
`thr2acc`/accurate-curve mapping plus the RLS eta + INDI incremental law
and the delayed thrust queue), and free functions in `flatness_math`
(normalization gradient, robust body-x axis). The RLS/INDI math itself stays
in `include/rls_throttle_model.h` with its unit test
(`test/throttle_model_test.cpp`); yaw contracts live in
`test/yaw_contract_test.cpp`.

## Throttle estimation

`throttle_estimator=0` keeps the legacy `thr2acc` mapping
(`u = a_des / eta`), `throttle_estimator=1` uses the RLS eta model
(`include/rls_throttle_model.h`) with the INDI incremental law
(`u += kp * (a_sp - a_meas) / eta * dt`). Both are startup parameters in
`bringup/config/px4ctrl{,_real}.yaml`; telemetry is published on
`/px4ctrl/throttle_model_status` (interface/ThrottleModelStatus).
