# Simple Sim Layer

Light-weight, principle-verification simulator for the AP-PnC tail-sitter UAV,
inspired by [MARSIM](https://github.com/hku-mars/MARSIM). No Gazebo physics —
a simplified rigid-body dynamics model is integrated directly, with synthetic
LiDAR sensing (ray casting against a hand-built scene) for future obstacle
avoidance experiments.

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

## Topics

| Topic                  | Type                           | Description                    |
| ---------------------- | ------------------------------ | ------------------------------ |
| `/simple_sim/points`   | `sensor_msgs/PointCloud2`      | Synthetic LiDAR point cloud    |
| `/simple_sim/odom`     | `nav_msgs/Odometry`            | Simulated vehicle state        |
| `/simple_sim/markers`  | `visualization_msgs/MarkerArray` | Vehicle, waypoints, obstacles |

## Scene & Obstacles

The world is configured through `config/simple_sim.yaml`:

- `waypoints`: `[x0,y0,z0, x1,y1,z1, ...]` — the vehicle tracks them in a loop.
- `obstacle_boxes`: `[cx,cy,cz, hx,hy,hz, ...]` — axis-aligned boxes.
- `obstacle_spheres`: `[cx,cy,cz, r, ...]` — spheres.

This is the extension point for simple obstacle-avoidance scenarios: publish the
point cloud to a planner, feed avoidance commands back as new waypoints, or swap
the controller in `controller.hpp`.
