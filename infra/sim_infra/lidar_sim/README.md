# ROS2 pillar scene and LiDAR development simulator

This is an interactive **development** entry point, separate from offline
benchmark sweeps. The existing lockstep C++ plant/controller runs under ROS2
with **Lyu aerodynamics + practical four-motor actuation**. An independent ROS2
node generates a pillar/ground point-cloud map and pose-dependent LiDAR first
returns. It requires no PX4, MAVROS, Gazebo, external dataset or real aircraft.

## Start (native Linux CPU in Docker)

Set `AP_PNC_DIR` to the absolute checkout root. The numerical `simple-sim` image
and the authorized practical numeric profile must exist. Missing practical data
fails explicitly rather than switching to ideal actuation.

```bash
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" import-matlab
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim build
JOB=$(uv run --project "$AP_PNC_DIR" ap-pnc dev-sim prepare)
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim run "$JOB"
```

The simulator starts **paused**. The CLI prints the job directory; its
`dev-sim.json` stores the exact container name. Use that name in a second terminal:

```bash
NAME=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["container_name"])' "$JOB/dev-sim.json")
docker exec "$NAME" /entrypoint.sh ros2 service call /simple_sim/resume std_srvs/srv/Trigger '{}'
docker exec "$NAME" /entrypoint.sh ros2 service call /simple_sim/pause std_srvs/srv/Trigger '{}'
docker exec "$NAME" /entrypoint.sh ros2 service call /simple_sim/step std_srvs/srv/Trigger '{}'
docker exec "$NAME" /entrypoint.sh ros2 service call /simple_sim/reset std_srvs/srv/Trigger '{}'
```

Ctrl-C stops only this job's container. Reset reuses frozen trajectory/config,
returns to paused time zero and creates a new scientific recording. Restart to
apply config changes. `prepare --controller se3` selects the aerodynamic SE3
controller instead of the default NMPC.

The development default is a 10-piece, radius-8m reference at altitude 3m with
planner `v_max=4m/s` and a 2s terminal hold. These are run-local overrides;
benchmark and canonical planning defaults are not changed. All generated
outputs live under `.artifacts/dev-sim/linux-<arch>/<job>/`:

- `config/`: frozen plant/planner/controller/sensor input copies.
- `profile.yaml`, `upstream-provenance.json`: local practical-profile provenance.
- `recordings/`: original C++ experiment snapshots, committed CSV and manifests.
- `dev-sim.json`: entry point, isolation and model identity.

## ROS2 interfaces and rendering

| Topic | Meaning |
|---|---|
| `/lidar_sim/map` | Latched pillar/ground surface map; frame `world` |
| `/lidar_sim/points` | Sensor-frame first-return `PointCloud2`; frame `lidar` |
| `/simple_sim/odom` | Committed world pose and body-frame twist |
| `/simple_sim/imu` | Body FLU angular velocity and specific force |
| `/simple_sim/aero_wrench` | Actual plant aerodynamic wrench |
| `/simple_sim/reference`, `/simple_sim/path` | Planned and committed plant paths |
| `/clock` | Committed simulation time |

TF is `world -> base_link -> lidar`, using the full odometry attitude and
configured mounting transform. The map is reliable/transient-local; scans use
sensor-data QoS. Every scan carries its source odometry stamp, not wall time.
No new odometry means no new scan. Reset accepts decreasing simulation time.

`core/bringup/rviz/dev_sim.rviz` displays the map, live returns, both paths and TF.
A Linux desktop may launch `bringup dev_sim.launch.py rviz:=true` with the same
absolute run-local config. The default Docker CLI is headless and does not
pretend to provide a Mac GUI, web viewer or external DDS access.

## LiDAR model and limits

`core/bringup/config/lidar_sim.yaml` owns the deterministic scene and sensor.
The default has six finite cylinders with caps plus a bounded ground plane:
16 vertical channels, 720 horizontal columns, 360° azimuth, 30° vertical FOV,
10Hz simulation-time scan spacing, 0.3–50m range and 1cm range noise.

Scans analytically intersect world geometry and keep the **nearest** surface.
Occlusion, finite pillar height/caps, range limits and full pose are included.
The display map is not secretly returned as a scan. Noise is seeded by committed
simulation time, making identical states/stamps reproducible.

Each scan is a simultaneous **snapshot**. Rolling-scan motion distortion,
reflectivity, multi-return, beam divergence, dynamic objects and real-device
packet formats are not modeled. Point fields are little-endian float32 xyz.

**No collision/contact dynamics or obstacle-aware replanning are added.** Pillars
are sensor geometry, not forces in the plant. The frozen reference may intersect
a custom obstacle. This is a perception/interface development environment, not
collision-avoidance acceptance. External PLY/3DGS scenes are not yet supported;
use the reproducible pillars first rather than downloading an unverified large
scene or treating Gaussian splats as LiDAR-solid geometry.

## Isolation and verification

The launcher uses Docker `--network none`, host IPC, `ROS_DOMAIN_ID=213` and
`ROS_LOCALHOST_ONLY=1`. Do not connect this experiment to a production DDS domain.
No hardware interface or flight dispatch is installed.

Pure geometry/staging contracts are in `test/cli/test_lidar_sim.py` and
`test/cli/test_dev_sim.py`. The real ROS integration probe is
`test/integration/check_dev_sim.py`; run it inside the isolated development image
with absolute `--config` and artifact `--output`. It checks map latching, committed
odometry, cloud fields/frames, pause/resume/reset and scan-to-scene alignment.
It does not establish full-run tracking performance or real LiDAR realism.
