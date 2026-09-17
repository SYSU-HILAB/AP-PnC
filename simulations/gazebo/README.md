# Gazebo Simulation Layer

Generic-interface SITL simulation: **stock PX4 v1.17.0** under **Gazebo
Jetty** (`gz_x500`), bridged by **mavros2** and controlled by the AP-PnC
NMPC stack. No customized firmware, no uXRCE-DDS.

## Stack

| Service          | Image                  | Role                                        |
| ---------------- | ---------------------- | ------------------------------------------- |
| `px4-simulator`  | `px4-simulator:latest` | stock PX4 v1.17.0 + Gazebo Jetty, `gz_x500` |
| `nav-sidecar`    | `nav-sidecar:latest`   | single nav gateway: mavros2 (sim profile); livox + fast_lio + ekf_quat baked in for the real profile |
| `qground_control`| `qground_control:latest` | QGroundControl ground station             |
| `ap-pnc`         | `ap-pnc:latest`        | ROS2 workspace: c_nmpc / px4ctrl / nmpc_bridge / flight_mux |

## Control path

```
planner → c_nmpc → /mpc_controls (rates FLU + thrust_acc_sp)
    → nmpc_bridge (RLS throttle model + INDI incremental thrust,
       reference: PX4-Neupilot acc_rates_control)
    → flight_mux (single /mavros/setpoint_raw/attitude publisher)
    → mavros2 (nav-sidecar) → PX4 (rates + normalized thrust)
```

- **px4ctrl** owns arming/offboard/takeoff/hover/land (TakeoffLand topic);
  its setpoint stream is remapped to `/px4ctrl/setpoint_raw/attitude`.
- **flight_mux** latches to the NMPC stream on `/mode` (true) and falls
  back to px4ctrl when the NMPC stream is stale (`stream_timeout`).
- **Odometry**: sim = `/mavros/local_position/odom` (PX4 EKF2); real =
  `/ekf_quat/ekf_odom` — one config switch (`odom_topic` /
  `nav_sidecar.launch.py profile:=real`).

## Quick Start

```bash
# from repo root
docker compose -f simulations/gazebo/docker/docker-compose.yml up -d
```

Inside `px4-simulator` (interactive, user 1002):

```bash
cd /home/px4/PX4-Autopilot
make px4_sitl gz_x500
```

`nav-sidecar` connects mavros via `tcp://localhost:4560` (host network).
Then start the control stack (in `ap-pnc` or via `ap-pnc stack start`):
`ros2 launch bringup bringup.launch.py` (px4ctrl + mux + bridge + c_nmpc).

Flight sequence: takeoff via `/px4ctrl/takeoff_land`, then the standard
`ap-pnc stack trigger go` hands the setpoint stream over to the NMPC.

## Firmware Versioning

Upstream PX4 (no fork) is cloned in the image and pinned via
`ARG PX4_VERSION` (default `v1.17.0`) in `px4-simulator.dockerfile`:

```bash
docker compose -f simulations/gazebo/docker/docker-compose.yml build px4-simulator
```

Behind a campus/corporate proxy, pass proxy build args (docker-proxy host
address as seen from the build container):

```bash
docker compose -f simulations/gazebo/docker/docker-compose.yml build \
  --build-arg http_proxy=http://172.17.0.1:7890 \
  --build-arg https_proxy=http://172.17.0.1:7890 \
  --build-arg HTTP_PROXY=http://172.17.0.1:7890 \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7890 \
  px4-simulator nav-sidecar
```

## Notes

- The `nav-sidecar` image is shared by simulation and the real machine;
  only the bringup profile differs (see `bringup/launch/nav_sidecar.launch.py`).
- K3s orchestration mirrors this stack (see `simulations/k3s/`); the
  `NAV_PROFILE` ConfigMap key selects the sidecar profile.
- The retired tail-sitter fork (`px4-vtol`) and the uXRCE-DDS bridge are
  gone; aerodynamics injection is not part of this stack (see
  `simulations/simple_sim/` for the aero line).
