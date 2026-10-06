---
name: ap-pnc-gazebo-e2e
description: End-to-end flight procedure and pitfall reference for the AP-PnC gazebo stack (stock PX4 v1.17 + Gazebo Harmonic + mavros2 + px4ctrl PASS_THROUGH). Use when running or debugging the SITL flight loop, arming/offboard failures, mavros setpoint issues, gz bridge problems, or the NMPC handover.
---

# AP-PnC Gazebo E2E

> **The `ap-pnc docker` command group was removed.** The Compose layer it
> wrapped is an internal API now: `tooling.docker_runtime` (`build`, `up`, `down`,
> `config`, `dev`). The examples below call it directly. The stack itself is still
> roadmap, so none of this is part of the accepted pipeline.


Flight loop: `px4-simulator` (stock PX4 v1.17.0, Gazebo Harmonic `gz_x500`)
→ `nav_infra` (mavros2 + `px4ctrl` with RLS/INDI + odometry adapter) →
`ap-pnc` core (planner + `nmpc`). px4ctrl is the single
`/mavros/setpoint_raw/attitude` publisher and owns arming/offboard; in
`ctrl_mode=1` its PASS_THROUGH state consumes `/nmpc/control` internally
(RLS eta + INDI thrust) instead of forwarding an external stream.

## Full flight runbook

```bash
# 1-4) the Compose runtime now starts SITL, GCS heartbeat, nav and core itself.
# Set absolute AP_PNC_DIR; default target is the host CPU (Mac Apple Silicon: ARM64).
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build()"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.up()"
# Do NOT start a duplicate SITL/core process or an external heartbeat sender.
docker exec nav_infra /entrypoint.sh python3 \
  /workspace/infra/sim_infra/gazebo/scripts/verify_runtime.py
# Continue with flight commands only in the confirmed simulator, after acceptance.

# 5) takeoff → stable 1 m hover (px4ctrl AUTO_TAKEOFF arms + enters OFFBOARD).
#    quadrotor_msgs now lives in nav_infra, publish from there.
docker exec nav_infra /entrypoint.sh ros2 topic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 1}"

# 6) remote-rates handover (synchronous service; falls back to hover on loss).
#    ctrl_mode=1: internal NMPC controls (/nmpc/control) + RLS/INDI thrust.
docker exec ap-pnc /entrypoint.sh ros2 service call /px4ctrl/toggle_pass_through \
  std_srvs/srv/SetBool "{data: true}"        # back: data: false

# throttle-estimator A/B (startup parameter, two flights):
#   Flight A: throttle_estimator: 0 (thr2acc mapping)
#   Flight B: throttle_estimator: 1 (RLS eta + INDI)  -> rebuild nav_infra
# Compare /debugPx4ctrl hover_percentage and
# /px4ctrl/throttle_model_status (eta, estimate_valid).
```

Smoke topic checks (inside any ROS container): `/mavros/state` connected=true,
`/mavros/imu/data` ~200 Hz, `/px4ctrl/odom_world` fresh, `/cur_state`
origin locked in the px4ctrl log.

## Pitfalls (all were hit and fixed — do not rediscover)

1. **mavros drops setpoints when `thrust_scaling` is NaN**: the
   `setpoint_raw` plugin ignores every `AttitudeTarget` with thrust != 0
   (`src/plugins/setpoint_raw.cpp`), so PX4 never sees an offboard signal
   and denies OFFBOARD/arming (`check_modes_offboard_signal`). The
   parameter lives on the plugin sub-node `/mavros/setpoint_raw`, and
   mavros 2.15 plugin sub-nodes are created with
   `use_global_arguments(false)` (`src/lib/plugin.cpp`) — a YAML params
   file (wildcard or `/**/setpoint_raw:` block) never reaches them.
   `nav_infra.launch.py` sets it at startup
   (`ros2 param set /mavros/setpoint_raw thrust_scaling 1.0`); verify with
   `ros2 param get /mavros/setpoint_raw thrust_scaling` before takeoff.
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
8. **Gazebo home/log ownership**: simulator runs as px4 (1002), with an
   artifact-owned home. Do not restore host `~/.gz`/X11 mounts; initialize
   writable bind-mounted log directories for the runtime UID.
9. **RLS throttle model**: px4ctrl seeds eta = g/hover_percentage
   (sim: 9.81/0.9 ≈ 10.9, real: ≈ 19.6). During `go` handover the estimate
   refines until `estimate_valid: true` (`/px4ctrl/throttle_model_status`);
   if it wobbles, seed/check `thrust_model.hover_percentage` in
   `px4ctrl{,_real}.yaml`.
10. **Do not manually spawn PX4/gz outside the make flow** — the bridge's
    actuator path depends on the environment the official
    `make px4_sitl gz_x500` injects.

## Diagnostic toolbox

- **ulg events**: `docker cp` the newest
  `/workspace/.artifacts/px4_src/build/px4_sitl_default/rootfs/log/*/*.ulg`, parse with
  `uv run --with pyulog python` — topics `event` (id/log_levels/arguments)
  and `failsafe_flags` (offboard_control_signal_lost, gcs_connection_lost).
- **Wire truth**: `sudo tcpdump -i lo -w x.pcap 'src port 14540 and dst port
  14580'` + `uv run --with scapy --with pymavlink` to decode — proves which
  MAVLink message types actually flow.
- **PX4 console**: `make px4_sitl` runs px4 with stdin=/dev/null; use a FIFO
  (`mkfifo /tmp/pxin; tail -f /tmp/pxin | PX4_SIM_MODEL=gz_x500 bin/px4`)
  only for `param show`/`listener` inspection — never for production runs
  (see pitfall 10).
