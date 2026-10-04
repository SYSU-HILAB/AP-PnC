# AP-PnC ROS 2 Base Images

This directory provides architecture-specific base images (`amd64` and `arm64`) for AP-PnC ROS 2 development and deployment.

## Architecture

The container environment cleanly decouples runtime dependencies from source code:

- `/workspace/src`: Mapped to host `core/ros_packages/` (planner line, `nmpc`, `interface`; `px4ctrl` and `quadrotor_msgs` moved to `infra/nav_infra/ros_packages/`).
- `/workspace/bringup`: Mapped to host `core/bringup/` (contains launch files and configurations).
- `/workspace/.artifacts/nmpc_solver`: Mapped to host `.artifacts/nmpc_solver/` (contains compiled `libnmpc_bundle.a`).

`AP_PNC_DIR=/workspace` is the only project path variable inside the image;
build, install and log trees live under `/workspace/.artifacts/colcon/`.

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

Using docker-compose (export an absolute `AP_PNC_DIR` first — every context
and volume resolves against it, never against the current directory):
```bash
export AP_PNC_DIR="$(cd "$(dirname "$0")/../.." && pwd)"  # repository root
docker compose -f "$AP_PNC_DIR/core/docker/docker-compose.yml" run --rm ros2-dev-amd64

docker compose -f "$AP_PNC_DIR/core/docker/docker-compose.yml" run --rm ros2-dev-arm64
```

Inside the container, compile all packages (build/install/log stay under
`$AP_PNC_DIR/.artifacts/colcon/`):
```bash
colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build --symlink-install \
  --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
  --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
