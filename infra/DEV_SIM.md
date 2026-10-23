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
collision-avoidance acceptance.

## Experimental: an external point-cloud/3DGS scene

`type: pillars` (above) stays the default/regression scene. A second, explicitly
experimental `type: point_cloud` scene exists for trying the stack against a
real captured environment, MARSIM-style: instead of analytic primitives, the
map is a voxel occupancy grid built from an arbitrary point cloud, and a scan is
the first range at which a ray's march enters an occupied voxel (see
`lidar_sim.geometry.PointCloudScene`). This is still a geometric approximation
(gaps show through at low point density; occupancy does not become a collision
force), not a verified LiDAR-realism or obstacle-avoidance claim.

`infra/sim_infra/lidar_sim/scripts/convert_3dgs_scene.py` turns a raw 3D
Gaussian Splatting training output (one binary PLY row per Gaussian: position,
opacity, scale, rotation, SH color) into the plain x,y,z PLY this scene type
consumes. It keeps Gaussians that behave like solid surface (opaque, small),
recovers the ground plane by RANSAC so the result is Z-up with ground at
z=0, and crops to a radius/height band around the origin. The source scene's
metric scale is whatever its own capture pipeline produced — sanity-check the
result against the flight envelope and pass `--scale` if it looks off.

A ready-made example: the Mip-NeRF360 "garden" scene (real-world capture,
non-commercial research license — see
[Mip-NeRF360](https://jonbarron.info/mipnerf360/) and the
[3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting)
repo it was trained with) has a community-trained pretrained PLY at
[`alexmkwizu/gaussian_training_datasets`](https://huggingface.co/datasets/alexmkwizu/gaussian_training_datasets)
(`tested_outputs/garden.ply`, Apache-2.0 output license, ~180MB). Convert it once:

```bash
curl -L -o /tmp/garden_raw.ply \
  https://huggingface.co/datasets/alexmkwizu/gaussian_training_datasets/resolve/main/tested_outputs/garden.ply
python3 infra/sim_infra/lidar_sim/scripts/convert_3dgs_scene.py \
  /tmp/garden_raw.ply "$AP_PNC_DIR/.artifacts/dev-sim/scenes/garden_points.ply"
```

Then select it when preparing a job, e.g. to try the 10m/s circuit this scene
was validated against:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim prepare --scene garden-3dgs --v-max 10
```

`prepare` copies the converted point cloud into the job directory and mounts it
read-only into the container; it fails explicitly if the file above is missing
rather than silently falling back to pillars. Neither the raw download nor the
converted output is committed to the repository — both stay under `.artifacts/`.

This was run once end-to-end with practical actuation + Lyu aerodynamics +
NMPC at `v_max=10`: the full closed radius-8m/altitude-3m circuit completed
(no solver failures, `position_rmse_m≈0.28`, `position_max_m≈0.65`), with the
LiDAR node returning real hits against the converted garden geometry
throughout (hundreds to a few thousand points per 11520-beam scan,
~50-70ms/scan on a laptop CPU). That one run is not a regression-tested
claim — rerun it yourself before relying on it.

## Experimental: obstacle-aware corridors from the sensed map (`corridor_source: map_sfc`)

The planner's safe flight corridor (SFC) defaults to a fixed box around the
seed trajectory (`problem_formulation.corridor_source: box_sfc`), unrelated to
anything sensed. Setting it to `map_sfc` (`dev-sim prepare --corridor-source
map_sfc`) instead builds the corridor from the LiDAR sim's own ground-truth
map and continuously replans while flying:

- `simple_sim_node` subscribes to `/lidar_sim/map` once (latched) and builds a
  dilated voxel occupancy grid (`voxel_map::VoxelMap`,
  `core/ros_packages/planner/src/gcopter/include/gcopter/voxel_map.hpp`,
  ported from upstream [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)).
- A dependency-free 3D grid A* (`planner_core/grid_astar.hpp`, project-original
  — upstream's path search needs OMPL, not otherwise a dependency here) finds
  a collision-free route between legs; `sfc_gen::convexCover` + `shortCut`
  (also ported from upstream, verbatim besides the vendoring header) turn it
  into a chain of overlapping H-polytopes via FIRI convex decomposition.
- `Planner::plan_corridor` feeds that corridor into the same
  `GCOPTER_PolytopeSFC` optimizer `plan()` already uses. Every replanned leg's
  initial state is the previous leg's exact terminal state (GCOPTER enforces
  boundary conditions exactly), so splicing is velocity/acceleration-continuous
  with no stop-and-go.
- `simple_sim::Experiment::replan_if_due` triggers a new leg once the current
  one's remaining duration drops below `map.replan_margin_s` (default 1.0s),
  advancing one mission waypoint at a time around the same closed-loop route
  `box_sfc` flies. `corridor_source: box_sfc` never touches any of this code.

**Validated mechanism, not a tuned demo.** A real end-to-end run against
`garden-3dgs` confirmed the full chain fires correctly (map ingested, corridor
built, leg replanned, new trajectory tracked) and that `box_sfc` stays
byte-identical (`metrics.json` matched a pre-change run exactly). But treating
every opaque Gaussian in a dense, unfiltered real-world point cloud as a hard
3D obstacle makes free space very constrained — one observed replan produced a
valid but extreme 31-second detour (large lateral excursion and altitude
climb) to get around it. That is the corridor construction correctly
respecting a conservative map, not a crash or a silent failure, but it is not
a demonstration of clean obstacle avoidance either. Before relying on this for
anything presentable, tune `map.voxel_scale_m`/`dilate_radius_m` or pre-filter
the source point cloud (tighter opacity/scale thresholds in
`convert_3dgs_scene.py`) to keep only genuinely solid obstacles rather than
every sensed point.

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
