# Simple Sim Layer

Light-weight, principle-verification simulator for the AP-PnC tail-sitter UAV,
inspired by [MARSIM](https://github.com/hku-mars/MARSIM). No Gazebo physics —
a simplified rigid-body dynamics model is integrated directly, with a
Livox Mid-360 style synthetic LiDAR (non-repetitive Lissajous scan, ray
casting against a hand-built scene).

Default scenario: a closed 100 m x 20 m x 20 m shell (1 m thick floor and
ceiling slabs, four walls) with randomly scattered upright pillars; the
vehicle flies from one end (`-45, 0, 2`) to the other (`45, 0, 2`).
Density is configurable via `scene.*` parameters.

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

## Run (via uv)

```bash
uv run ap-pnc sim simple        # build & start simple-sim via Docker
# or
uv run ap-pnc start --target simple
```

## Run (host ROS2)

```bash
# from repo root
source /opt/ros/humble/setup.bash
uv run ap-pnc build simple-sim
source install/setup.bash
ros2 launch simple_sim simple_sim.launch.py
```

RViz2 shows the synthetic point cloud (`/simple_sim/points`), vehicle odometry
(`/simple_sim/odom`), aerodynamic wrench (`/simple_sim/aero_wrench`), and scene markers (`/simple_sim/markers`).

## Run (docker direct)

```bash
make sim-simple-up          # or: uv run ap-pnc sim simple
# inside the container:
ros2 launch simple_sim simple_sim.launch.py
```

## Inspect a LiDAR Frame (PCD)

Dump one frame as ASCII PCD and view it on the host with open3d (via `uv`):

```bash
uv run ap-pnc sim pcd
# equivalent to: make sim-simple-pcd
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
- `scene.pillar_count`: total number of pillars (density), scattered along
  the whole corridor (not rows).
- `scene.radius_min/max`: random pillar radius range [m].
- `scene.corridor_half_width`: keep-out around the flight line - pillar
  surfaces never enter it, so the straight task stays feasible.
- `scene.edge_margin`: clearance from the end walls [m].
- `scene.seed`: change for a different reproducible layout.
- `build_shell`: closed shell - 1 m thick floor slab (top at z=0) and
  ceiling slab (bottom at `map.height`) plus four 0.2 m walls.

All pillars are full-height upright cylinders scattered completely at
random by seeded rejection sampling - no rows, no mirroring, minimum
center spacing enforced. Generation lives in `Scene::makePillarCourse`.

For custom layouts beyond the procedural field, extend
`makePillarCourse` in `scene.cpp`. This is the extension point for
obstacle-avoidance scenarios: feed the point cloud to a planner, push
avoidance commands back as new waypoints, or swap the controller in
`controller.hpp`.
