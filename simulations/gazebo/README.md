# Gazebo Simulation Layer

Full-fidelity SITL simulation: PX4-VTOL firmware (tail-sitter fork) under Gazebo Garden,
bridged to ROS2 Humble and controlled by the AP-PnC NMPC stack.

## Stack

| Service          | Image                  | Role                                        |
| ---------------- | ---------------------- | ------------------------------------------- |
| `px4-simulator`  | `px4-simulator:latest` | PX4-VTOL SITL + Gazebo Garden, jetson-style |
| `qground_control`| `qground_control:latest` | QGroundControl ground station               |
| `ap-pnc`         | `ap-pnc:latest`        | ROS2 workspace: c_nmpc / px4_interface etc. |

## Quick Start

```bash
# from repo root
docker compose -f simulations/gazebo/docker/docker-compose.yml up
```

or via the repo Makefile:

```bash
make sim-gazebo-up
make sim-gazebo-down
```

Inside `px4-simulator` (interactive, `user 1002`), run the Gazebo SITL:

```bash
cd /home/px4/px4-vtol
make px4_sitl gz_tailsitter    # or the airframe shipped in the fork
```

## Firmware Versioning

The `px4-vtol` fork (https://github.com/WarriorHanamy/px4-vtol) is NOT part of this repo
as a submodule/subtree — it is a build-time dependency cloned inside the image. The
`px4-simulator.dockerfile` pins it to a fixed commit via `ARG PX4_VTOL_COMMIT`, so image
builds are reproducible. To bump the firmware:

```bash
# get the new target commit sha, then edit the ARG default in px4-simulator.dockerfile
docker compose -f simulations/gazebo/docker/docker-compose.yml build px4-simulator
```

## Notes

- The tail-sitter airframe, Gazebo world and aerodynamics plugins are customized inside
  the `px4-vtol` fork itself (already done, nothing to configure here).
- ROS2 bridge (`src/px4_interface`, `src/mocap-px4`) lives in the main `src/` tree and is
  baked into the `ap-pnc` image via `colcon build`.
