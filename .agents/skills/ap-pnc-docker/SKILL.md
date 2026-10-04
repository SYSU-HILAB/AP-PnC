---
name: ap-pnc-docker
description: Build and run the AP-PnC gazebo Docker stack (px4-simulator, nav_infra, ap-pnc). Covers prerequisites, building, running, and hot-update of the NMPC solver bundle. Use when working with the ROS2 stack or when the user mentions Docker/container build/run/up.
---

# AP-PnC Docker

## Prerequisites

- Host bootstrap must be done: `uv run ap-pnc install`
- NMPC bundle must exist: `ls .artifacts/nmpc_solver/libnmpc_bundle.a`
  If not, run `uv run ap-pnc gen-nmpc-lib` first.

## Build

From repo root (compose handles all services):

```bash
uv run ap-pnc gen-nmpc-lib
# compose resolves every path against an absolute AP_PNC_DIR
export AP_PNC_DIR="$(git rev-parse --show-toplevel)"
docker compose -f "$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml" build ap-pnc nav_infra
```

`px4-simulator` builds from upstream PX4 v1.17.0 (pinned by
`ARG PX4_VERSION`, Gazebo Harmonic) and is slow — only rebuild when bumping
the firmware. Behind a proxy pass
`--build-arg https_proxy=http://<docker0-ip>:<port>` (see
`infra/sim_infra/gazebo/README.md`).

## Run

```bash
# all services (add --scale qground_control=0 for headless)
docker compose -f "$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml" up -d

# core subset
docker compose -f "$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml" up -d \
  --no-deps px4-simulator nav_infra ap-pnc
```

Containers are interactive (`command: bash`) — the SITL and the control
stack are started inside them per the `ap-pnc-gazebo-e2e` skill.

## Hot Update

The bundle is bind-mounted at runtime, so most solver updates need no
rebuild:

```bash
uv run ap-pnc gen-nmpc-lib     # rewrites .artifacts/nmpc_solver/libnmpc_bundle.a
docker exec ap-pnc bash -c 'pkill -f nmpc' && ...   # restart the node/stack
```

Rebuild `ap-pnc` only when C++ sources or dependencies change; core config
(`core/bringup/`) is live through the symlink-install mount. `px4ctrl`
(FCU interface/controller) lives in the `nav_infra` image — source or
config changes there require `docker compose ... build nav_infra` (the
image is self-contained, no bind mounts).

## Tips

- The bundle is staged at `<AP_PNC_DIR>/.artifacts/nmpc_solver/libnmpc_bundle.a`;
  CMake derives that path from `AP_PNC_DIR` (no per-file environment variable).
  `nmpc_node` links acados statically — no `.so` dependencies. Regenerate the
  bundle after solver-wrapper API changes.
- Core image (`ap-pnc-ros2-base`) stays lean; PCL/tf2/mavros runtime
  dependencies live only in the `nav_infra` image.
- The install dir is root-owned after container builds — clean it via a
  container, not host `rm`.
