# nav_infra — base-services sidecar

Single self-contained unit providing the generic flight interface. NOT
research code: the core container (`core/ros_packages/`: planner line,
c_nmpc, px4ctrl) consumes these services over the shared host-network ROS 2 graph.

## Contents

| Package           | Role                                                                |
| ----------------- | ------------------------------------------------------------------- |
| `px4_interface`   | `nmpc_bridge` (/mpc_controls → RLS throttle model + INDI thrust → AttitudeTarget on /nmpc_bridge/setpoint_raw/attitude; odom adapter → /cur_state, /nmpc_bridge/odom_world) |
| `livox_ros_driver2` | Mid-360 driver (real profile)                                    |
| `fast_lio`        | FAST-LIO2 (real profile)                                             |
| `ekf_quat_pose`   | odom fusion → /ekf_quat/ekf_odom (real profile)                      |

Plus a from-source mavros2 build (the jammy repo ships `mavros-msgs` only),
installed under `/opt/mavros` inside the image.

## Profiles (same image)

```
sim  : mavros (tcp://localhost:4560) + nmpc_bridge + flight_mux
real : sim + livox + fast_lio + ekf_quat   (odom := /ekf_quat/ekf_odom)
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
| → core    | `/cur_state`             | interface/State           | ENU world, FLU body           |
| → core    | `/nmpc_bridge/odom_world`| nav_msgs/Odometry         | world-frame twist (for px4ctrl) |
| ← core    | `/mpc_controls`          | interface/Controls        | rates_sp FLU + thrust_acc_sp  |
| → core    | `/nmpc_bridge/setpoint_raw/attitude` | mavros_msgs/AttitudeTarget | NMPC rates+throttle (px4ctrl forwards it in PASS_THROUGH) |

px4ctrl (core container) keeps ownership of arming/offboard through the
mavros services exposed here and is the single publisher of
/mavros/setpoint_raw/attitude. Handover to the NMPC rates stream is a
synchronous service on px4ctrl itself:

```bash
ros2 service call /px4ctrl/toggle_pass_through std_srvs/srv/SetBool "{data: true}"   # go
ros2 service call /px4ctrl/toggle_pass_through std_srvs/srv/SetBool "{data: false}"  # back to hover
```

In PASS_THROUGH px4ctrl repeat-forwards the bridge stream at its ctrl
rate and falls back to AUTO_HOVER when the stream goes stale
(`msg_timeout.pass_through`).
