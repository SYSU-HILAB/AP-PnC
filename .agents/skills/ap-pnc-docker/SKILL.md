---
name: ap-pnc-docker
description: Build and run the AP-PnC ROS2 execution environment in Docker. Covers prerequisites, building, running, and hot-update of the NMPC solver bundle. Use when working with the ROS2 stack or when the user mentions Docker/container build/run/up.
---

# AP-PnC Docker

## Prerequisites

- Host bootstrap must be done: `uv run ap-pnc install`
- OCP bundle must exist: `ls build/nmpc_wrapper/libnmpc_bundle.a`
  If not, run `uv run ap-pnc gen-nmpc-lib` first.

## Build

One command from repo root:

```bash
./scripts/docker-build.sh
```

Or manually:

```bash
uv run ap-pnc gen-nmpc-lib
docker compose -f simulations/gazebo/docker/docker-compose.yml build ap-pnc
```

## Run

Start all services (px4-simulator, qgroundcontrol, ap-pnc):

```bash
docker compose -f simulations/gazebo/docker/docker-compose.yml up
```

Start ap-pnc only (if other services are already running):

```bash
docker compose -f simulations/gazebo/docker/docker-compose.yml up ap-pnc
```

## Hot Update

When the OCP model/weights change:

```bash
uv run ap-pnc gen-nmpc-lib              # regenerate bundle
docker compose -f simulations/gazebo/docker/docker-compose.yml build ap-pnc  # rebuild image
docker compose up ap-pnc       # restart container
```

## Tips

- The bundle `.a` is volume-mounted at runtime for easy updates
- `NMPC_BUNDLE_PATH=/ap-pnc/build/nmpc_wrapper/libnmpc_bundle.a` is already set in the image
- The final `c_nmpc_node` binary is fully self-contained — no acados `.so` dependencies
