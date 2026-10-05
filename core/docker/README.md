# ROS2 Linux development images

One Dockerfile, `core/docker/ros2-base.dockerfile`, serves both architectures.
Do not use separate ARM64/AMD64 source recipes or mix their build trees.

| Host-native target | Docker platform | Bundle |
|---|---|---|
| Apple Silicon / Linux ARM64 | linux/arm64 | libnmpc_bundle_aarch64.a |
| Intel Mac / Linux x86_64 | linux/amd64 | libnmpc_bundle_x86_64.a |

Set an absolute checkout root. The CLI discovers this root from its module or
`AP_PNC_DIR`, not the shell cwd:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv run --project "$AP_PNC_DIR" ap-pnc docker dev
# Explicit cross/deployment target, never an automatic fallback:
uv run --project "$AP_PNC_DIR" ap-pnc docker dev --arch amd64
```

`dev` creates a missing matching solver archive in the pinned Linux builder
and builds/reuses the corresponding base image. To build only the base:

```bash
export AP_PNC_ARCH=arm64  # amd64 on an x86 host
# Selecting a service explicitly activates its architecture profile.
docker compose -f "$AP_PNC_DIR/core/docker/docker-compose.yml" build ros2-dev-arm64
```

The runtime mounts `core/ros_packages` at `/workspace/src`, bringup at
`/workspace/bringup`, and the selected archive at the container-local alias.
Colcon output is isolated on the host under
`.artifacts/docker-dev/<arm64|amd64>/colcon/`, not the host's native install tree.
Both services use host network and host IPC; bare `up` starts neither profile.

Inside the development shell:

```bash
colcon --log-base "$AP_PNC_DIR/.artifacts/colcon/log" build \
  --base-paths "$AP_PNC_DIR/src" "$AP_PNC_DIR/bringup" \
  --build-base "$AP_PNC_DIR/.artifacts/colcon/build" \
  --install-base "$AP_PNC_DIR/.artifacts/colcon/install" \
  --packages-select interface nmpc planner bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$AP_PNC_DIR/.artifacts/colcon/install/setup.bash"
```

For self-contained simulation images, use `ap-pnc docker build` instead.
See [Gazebo deployment](../../infra/sim_infra/gazebo/README.md).
