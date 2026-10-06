---
name: ap-pnc-docker
description: Build and run the AP-PnC all-Docker, native ARM64/AMD64 gazebo stack. Covers target selection, host IPC, headless/default and noVNC GUI profiles, and static solver rebuilds.
---

# AP-PnC Docker

> **The `ap-pnc docker` command group was removed.** The Compose layer it
> wrapped is an internal API now: `tooling.docker_runtime` (`build`, `up`, `down`,
> `config`, `dev`). The examples below call it directly. The stack itself is still
> roadmap, so none of this is part of the accepted pipeline.


Read `infra/sim_infra/gazebo/README.md` from absolute `AP_PNC_DIR` for the
current deployment contract. No host ROS/acados install is required.

## Target and build

Apple Silicon defaults to Linux ARM64, never silent AMD64 emulation.
The CLI resolves the architecture from the host and writes it into the generated
Compose override; AP_PNC_DIR is the only variable involved.

```bash
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build()"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(gui=True)"
# Explicit non-native deployment target only when requested:
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(arch='amd64')"
```

A missing arch-tagged solver is generated in Docker. To regenerate deliberately:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(services=['ap-pnc'])"
```

Caches live under `.artifacts/nmpc_build/<solver_arch>/`. Core/simple-sim image
builds check every archive member's ELF target. Never consume the host alias
as a portable build input. NMPC is statically linked: regeneration requires an
image rebuild, not a bind-mounted archive or merely restarting an old node.

## Run

```bash
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.up()"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.up(gui=True)"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.down()"
```

Default services automatically start headless gz_x500 SITL, GCS heartbeat,
MAVROS/px4ctrl and planner/NMPC. Do not launch duplicate processes with exec.
No takeoff/arming is issued by startup. The optional GUI is served at
http://127.0.0.1:6088/vnc.html?autoconnect=1&resize=scale.

All services share host IPC. ROS/SITL share host networking and FastDDS SHM.
On Mac, host means Docker's Linux VM. Desktop HTTP is bridge-published on
host loopback only; no host X11 or NVIDIA is assumed.

## Verification

```bash
docker exec nav_infra /entrypoint.sh python3 \
  /workspace/infra/sim_infra/gazebo/scripts/verify_runtime.py
```

This read-only probe checks connected/unarmed PX4, actual sensor/state messages,
core nodes and thrust_scaling. Full flight acceptance remains separate in the
gazebo-e2e skill. A recipe, image manifest or process healthcheck alone is not
proof of runtime/flight success.

## Updates and scope

- Core config is read-only mounted; C++/solver edits require core image rebuild.
- nav config/C++ edits require nav image rebuild. sim is the default build target;
  `--nav-target real` additionally compiles hardware/LIO packages.
- Raw Compose works as-is: the base files carry literal defaults and the tooling
generates the host-specific override.
- Dev caches are isolated under `.artifacts/docker-dev/<docker_arch>/colcon/`.
- Never mount all host artifacts over an image install tree.
- K3s manifests and GPU sensor performance require their own migration/acceptance.
- Current Mac exception: the new production PX4 rebuild is deferred to native
  Linux runners. Do not invoke the all-image build on this Mac unless explicitly
  resuming that work; core/nav and optional GUI can be selected separately.
  Read the Gazebo README's validation table before making acceptance claims.
- QGC/desktop/probe images and caches are disposable after retaining logs,
  JSON and screenshots; their cleanup does not invalidate prior verification.
- Watch disk usage. Do not remove unrelated Docker images/containers.
