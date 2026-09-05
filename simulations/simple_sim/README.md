# Simple Sim Layer

Light-weight, principle-verification simulator for the AP-PnC tail-sitter UAV,
inspired by [MARSIM](https://github.com/hku-mars/MARSIM). No Gazebo physics —
a simplified rigid-body dynamics model is integrated directly, with a
Livox Mid-360 style synthetic LiDAR (non-repetitive Lissajous scan, ray
casting against a hand-built scene).

Default scenario: a closed 100 m x 20 m x 20 m shell (floor, ceiling, four
walls) with a regular course of upright pillars; the vehicle flies from one
end (`-45, 0, 2`) to the other (`45, 0, 2`). Density is configurable via
`scene.*` parameters.

## Layout

```
src/simple_sim/               # ROS2 package (ament_cmake)
  include/simple_sim/
    scene.hpp                 #   world: ground + box/sphere obstacles + raycast
    lidar_sim.hpp             #   synthetic spherical-scan LiDAR
    dynamics.hpp              #   simplified flight dynamics (Euler integration)
    controller.hpp            #   PD position + attitude controller
  src/simple_sim_node.cpp     #   closed-loop ROS2 node
launch/                       # ros2 launch entry
config/simple_sim.yaml        #   node parameters
rviz/simple_sim.rviz          #   visualization preset
docker/                       #   container build + compose
```

## Run (host)

```bash
# from repo root
source /opt/ros/humble/setup.bash
colcon build --paths src simulations/simple_sim/src --symlink-install
source install/setup.bash
ros2 launch simple_sim simple_sim.launch.py
```

RViz2 shows the synthetic point cloud (`/simple_sim/points`), vehicle odometry
(`/simple_sim/odom`) and scene markers (`/simple_sim/markers`).

## Run (docker)

```bash
make sim-simple-up          # build + start container (interactive bash)
# inside the container:
ros2 launch simple_sim simple_sim.launch.py
```

## Inspect a LiDAR Frame (PCD)

Dump one frame as ASCII PCD and view it on the host with open3d (via `uv`):

```bash
make sim-simple-pcd
# equivalent to: dump 3 s of frames into .artifacts/simple-sim/frame.pcd, then
uv run scripts/tools/viz_pcd.py .artifacts/simple-sim/frame.pcd
```

The viewer colors points by intensity (distance). Use `--info` for
statistics only, or tune the dump on the node:
- `dump_pcd_path`: where to write the cloud (default: off)
- `dump_pcd_frames`: frames accumulated before writing (default 30) — a
  single Lissajous frame is a sparse curve; accumulation fills surfaces
  so pillars show up as solid vertical slabs.

## Topics

| Topic                  | Type                           | Description                    |
| ---------------------- | ------------------------------ | ------------------------------ |
| `/simple_sim/points`   | `sensor_msgs/PointCloud2`      | Synthetic LiDAR point cloud    |
| `/simple_sim/odom`     | `nav_msgs/Odometry`            | Simulated vehicle state        |
| `/simple_sim/markers`  | `visualization_msgs/MarkerArray` | Vehicle, waypoints, obstacles |

## Scene & Obstacles

The world is configured through `config/simple_sim.yaml`:

- `map.length/width/height` + `build_walls`: walled world bounds [m].
- `waypoints`: `[x0,y0,z0, x1,y1,z1, ...]` — the vehicle tracks them in a loop.
- `scene.pillar_count`: pillar positions per row (density) - total pillars
  = 2 x count (two rows).
- `scene.pillar_radius`: uniform pillar radius [m].
- `scene.pillar_offset`: row distance from the y=0 flight line [m].
- `scene.edge_margin`: first/last pillar clearance from the end walls [m].
- `build_shell`: floor + ceiling + four walls (ceiling at `map.height`).

Layout is fully deterministic (no randomness): two straight rows of
identical full-height upright pillars, symmetric about the flight line.
Generation lives in `Scene::makePillarCourse`.

For custom layouts beyond the procedural field, extend
`makePillarCourse` in `scene.cpp`. This is the extension point for
obstacle-avoidance scenarios: feed the point cloud to a planner, push
avoidance commands back as new waypoints, or swap the controller in
`controller.hpp`.
