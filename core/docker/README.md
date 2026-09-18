# AP-PnC ROS 2 Base Images

This directory provides architecture-specific base images (`amd64` and `arm64`) for AP-PnC ROS 2 development and deployment.

## Architecture

The container environment cleanly decouples runtime dependencies from source code:

- `/workspace/src`: Mapped to host `ros_packages/` (core: planner line, `c_nmpc`, `px4ctrl`, `utils/msgs`).
- `/workspace/bringup`: Mapped to host `bringup/` (contains launch files and configurations).
- `/workspace/artifacts`: Mapped to host `.artifacts/nmpc_wrapper/` (contains compiled `libnmpc_bundle.a`).

## Building Base Images

### amd64
```bash
# Build base image
docker build -f core/docker/amd64/ros2-base.dockerfile --target base -t ap-pnc-ros2-base:amd64 .

# Or build self-contained runner image with precompiled packages
docker build -f core/docker/amd64/ros2-base.dockerfile --target runner -t ap-pnc:amd64 .
```

### arm64
```bash
# Build base image (using buildx or on Apple Silicon / arm64 Linux)
docker buildx build --platform linux/arm64 -f core/docker/arm64/ros2-base.dockerfile --target base -t ap-pnc-ros2-base:arm64 .
```

## Running Dev Container

Using docker-compose:
```bash
# Start amd64 development container
docker compose -f docker/docker-compose.yml run --rm ros2-dev-amd64

# Start arm64 development container
docker compose -f docker/docker-compose.yml run --rm ros2-dev-arm64
```

Inside the container, compile all packages:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
