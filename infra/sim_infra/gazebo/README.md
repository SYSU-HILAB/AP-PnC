# Gazebo Simulation Layer

Generic-interface SITL simulation: **stock PX4 v1.17.0** under **Gazebo
Harmonic** (`gz_x500`), bridged by **mavros2** and controlled by the AP-PnC
NMPC stack. No customized firmware, no uXRCE-DDS.

## Stack

| Service          | Image                  | Role                                        |
| ---------------- | ---------------------- | ------------------------------------------- |
| `px4-simulator`  | `px4-simulator:latest` | stock PX4 v1.17.0 + Gazebo Harmonic, `gz_x500` |
| `nav_infra`      | `nav_infra:latest`     | base-services gateway: mavros2 + px4ctrl (FCU interface, tracking/NMPC controller modes); livox + fast_lio + ekf_quat join in the real profile |
| `qground_control`| `qground_control:latest` | QGroundControl ground station             |
| `ap-pnc`（核心）  | `ap-pnc:latest`        | research code: planner 线 / nmpc |

## Control path

```
planner → nmpc → /mpc_controls (rates FLU + specific_force_sp)
    → px4ctrl ctrl_mode=1 (RLS throttle model + INDI incremental thrust,
       reference: PX4-Neupilot acc_rates_control)
    → /mavros/setpoint_raw/attitude (single publisher)
    → mavros2 → PX4

px4ctrl lives in the nav_infra container (base services); the core
container holds planner + nmpc. px4ctrl is the sole mavros setpoint
publisher and also owns arming/offboard/takeoff/hover/land.
```

- **px4ctrl** owns arming/offboard/takeoff/hover/land (TakeoffLand topic)
  and the odometry state adapter (`/cur_state`, `/cur_state_odom`,
  `/px4ctrl/odom_world`).
- **Handover**: `ros2 service call /px4ctrl/toggle_pass_through
  std_srvs/srv/SetBool "{data: true}"` (or `ap-pnc stack trigger go`)
  switches px4ctrl to its remote-rates source (`ctrl_mode=1` internal NMPC
  + INDI, `ctrl_mode=0` external stream); stale streams fall back to hover.
- **Throttle estimation**: startup parameter `throttle_estimator`
  (0 = thr2acc mapping, 1 = RLS eta + INDI) applies to the tracking
  controller as well.
- **Odometry**: sim = `/mavros/local_position/odom` (PX4 EKF2, body twist
  converted internally); real = `/ekf_quat/ekf_odom` — one profile switch
  (`nav_infra.launch.py profile:=real`).

## Quick Start

```bash
# from repo root
docker compose -f infra/sim_infra/gazebo/docker/docker-compose.yml up -d
```

Inside `px4-simulator` (interactive, user 1002):

```bash
cd /home/px4/PX4-Autopilot
make px4_sitl gz_x500
```

`nav_infra` connects mavros via `udp://:14540@localhost:14580` (host
network). Then start the core stack (in `ap-pnc` or via `ap-pnc stack
start`): `ros2 launch bringup bringup.launch.py` (nmpc only — px4ctrl is
launched by the nav_infra container).

Flight sequence: takeoff via `/px4ctrl/takeoff_land` (publish from the
`nav_infra` container: `quadrotor_msgs` lives there), then
`ap-pnc stack trigger go` (synchronous service) hands control to the NMPC
rates source; `ap-pnc stack trigger back` returns to hover.

## Firmware Versioning

Upstream PX4 (no fork) is cloned in the image and pinned via
`ARG PX4_VERSION` (default `v1.17.0`) in `px4-simulator.dockerfile`:

```bash
docker compose -f infra/sim_infra/gazebo/docker/docker-compose.yml build px4-simulator
```

Behind a campus/corporate proxy, pass proxy build args (docker-proxy host
address as seen from the build container):

```bash
docker compose -f infra/sim_infra/gazebo/docker/docker-compose.yml build \
  --build-arg http_proxy=http://172.17.0.1:7890 \
  --build-arg https_proxy=http://172.17.0.1:7890 \
  --build-arg HTTP_PROXY=http://172.17.0.1:7890 \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7890 \
  px4-simulator nav_infra
```

## Notes

- The `nav_infra` image is shared by simulation and the real machine;
  only the bringup profile differs (see `infra/nav_infra/bringup/launch/nav_infra.launch.py`).
- K3s orchestration mirrors this stack (see `infra/sim_k3s/`); the
  `NAV_PROFILE` ConfigMap key selects the sidecar profile.
- The retired tail-sitter fork (`px4-vtol`) and the uXRCE-DDS bridge are
  gone; aerodynamics injection is not part of this stack (see
  `infra/sim_infra/simple_sim/` for the aero line).
