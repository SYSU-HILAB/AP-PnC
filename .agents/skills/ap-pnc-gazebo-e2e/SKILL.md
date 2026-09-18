---
name: ap-pnc-gazebo-e2e
description: End-to-end flight procedure and pitfall reference for the AP-PnC gazebo stack (stock PX4 v1.17 + Gazebo Harmonic + mavros2 + px4ctrl PASS_THROUGH). Use when running or debugging the SITL flight loop, arming/offboard failures, mavros setpoint issues, gz bridge problems, or the NMPC handover.
---

# AP-PnC Gazebo E2E

Flight loop: `px4-simulator` (stock PX4 v1.17.0, Gazebo Harmonic `gz_x500`)
→ `nav_infra` (mavros2 + `nmpc_bridge` RLS/INDI + odometry adapter) →
`ap-pnc` core (planner + `c_nmpc` + `px4ctrl`). px4ctrl is the single
`/mavros/setpoint_raw/attitude` publisher; its PASS_THROUGH state
repeat-forwards the NMPC rates stream.

## Full flight runbook

```bash
# 1) containers (skip qgc for headless runs)
docker compose -f simulations/gazebo/docker/docker-compose.yml up -d \
  --no-deps px4-simulator nav_infra ap-pnc

# 2) SITL inside the simulator container — GZ_IP is MANDATORY (see pitfalls)
docker exec -d px4-simulator bash -c \
  'cd /home/px4/PX4-Autopilot && export GZ_IP=127.0.0.1 && HEADLESS=1 make px4_sitl gz_x500 > /tmp/sitl.log 2>&1'
# wait for "Startup script returned successfully" in /tmp/sitl.log

# 3) GCS heartbeat — PX4 blocks arming without a GCS heartbeat
#    (udp from port 14550 to 18570, MAV_TYPE_GCS, srcSystem=255)
cd /tmp && uv run --with pymavlink python gcs_heartbeat.py   # any 1 Hz GCS heartbeat sender

# 4) control stack (sim profile; px4ctrl.yaml hover_percentage 0.9)
docker exec -d ap-pnc bash -c \
  'cd /workspace && source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 launch bringup bringup.launch.py profile:=sim > /tmp/stack.log 2>&1'

# 5) takeoff → stable 1 m hover (px4ctrl AUTO_TAKEOFF arms + enters OFFBOARD)
docker exec ap-pnc bash -c 'source ... && ros2 topic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 1}"'

# 6) NMPC handover (synchronous service; falls back to hover on stream loss)
docker exec ap-pnc bash -c 'ros2 service call /px4ctrl/toggle_pass_through \
  std_srvs/srv/SetBool "{data: true}"'        # back: data: false
```

Smoke topic checks (inside any ROS container): `/mavros/state` connected=true,
`/mavros/imu/data` ~200 Hz, `/nmpc_bridge/odom_world` fresh, `/cur_state`
origin locked in the bridge log.

## Pitfalls (all were hit and fixed — do not rediscover)

1. **mavros silently drops setpoints**: the `setpoint_raw` plugin drops every
   `AttitudeTarget` with thrust != 0 when `thrust_scaling` is NaN (its
   default). The parameter lives on the plugin sub-node
   `/mavros/setpoint_raw`, so the config must use a `/**:` wildcard key —
   `mavros: ros__parameters:` never reaches plugin sub-nodes. Set
   `setpoint_raw.thrust_scaling: 1.0`.
2. **fcu_url**: stock PX4 SITL has NO TCP 4560. Onboard mavlink instance is
   UDP 14580→14540: `fcu_url: "udp://:14540@localhost:14580"`.
3. **GZ_IP unicast discovery**: container multicast is broken, which kills
   the bridge→gz actuator direction (sensors still flow — deceiving).
   Export `GZ_IP=127.0.0.1` for BOTH the gz server and PX4 — simplest is
   exporting it before `make px4_sitl gz_x500` (the make flow propagates it).
   Also `GZ_SIM_RESOURCE_PATH` must be set (the PX4 make flow does this).
4. **RC/GCS-loss failsafes**: no-RC offboard-only setups get armed → instant
   RTL failsafe → auto-disarm. Fixed via `rootfs/etc/extras.txt` boot hook:
   `param set COM_RCL_EXCEPT 12` (Offboard|ExternalMode bits) and
   `param set COM_DLL_EXCEPT 4` (Offboard bit), then `param save`.
5. **Arming denied diagnosis**: PX4 v1.17 reports via events (not
   STATUSTEXT). Decode from the ulg: event id = `0x1000000 | fnv1a32(name)`
   (hash in `platforms/common/include/px4_platform_common/events.h`); names
   live in `PX4-Autopilot/src/**` as `events::ID("...")`. Key ones:
   `check_modes_offboard_signal` (setpoints not arriving),
   `check_rc_dl_no_dllink` (no GCS heartbeat), `commander_arm_denied_resolve_failures`.
6. **gz topic CLI needs `GZ_IP`** too, otherwise `gz topic -e` shows nothing
   even while data flows — a false negative trap.
7. **Hover throttle**: gz x500 needs `hover_percentage: 0.9` in the sim
   profile (real airframe: 0.5). Lower values leave the drone trembling on
   the ground while everything else looks green.
8. **`~/.gz` mount ownership**: compose mounts `${HOME}/.gz/sim` which is
   owned by the host user (1000) but the container user is px4 (1002) — the
   gz server dies creating log dirs. chown in the image or drop the mount.
9. **RLS throttle model**: starts at eta = g/hover_throttle (19.6 for 0.5).
   With the real sim hover of 0.9 (eta≈10.9) the handover wobbles until the
   RLS converges (`/nmpc_bridge/throttle_model_status`,
   `estimate_valid: true`). Warm it up by streaming controls before
   handover, or seed `rls.eta_init` ≈ 10.9 in `mavros_bridge.yaml`.
10. **Do not manually spawn PX4/gz outside the make flow** — the bridge's
    actuator path depends on the environment the official
    `make px4_sitl gz_x500` injects.

## Diagnostic toolbox

- **ulg events**: `docker cp` the newest
  `build/px4_sitl_default/rootfs/log/*/*.ulg`, parse with
  `uv run --with pyulog python` — topics `event` (id/log_levels/arguments)
  and `failsafe_flags` (offboard_control_signal_lost, gcs_connection_lost).
- **Wire truth**: `sudo tcpdump -i lo -w x.pcap 'src port 14540 and dst port
  14580'` + `uv run --with scapy --with pymavlink` to decode — proves which
  MAVLink message types actually flow.
- **PX4 console**: `make px4_sitl` runs px4 with stdin=/dev/null; use a FIFO
  (`mkfifo /tmp/pxin; tail -f /tmp/pxin | PX4_SIM_MODEL=gz_x500 bin/px4`)
  only for `param show`/`listener` inspection — never for production runs
  (see pitfall 10).
