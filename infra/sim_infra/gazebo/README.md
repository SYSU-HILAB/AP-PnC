# Gazebo / PX4 — all-Docker, native ARM64 and AMD64

The default is **headless Linux on the host-native CPU**. Apple Silicon selects
`linux/arm64`, not AMD64 emulation. GUI applications remain in containers and
are displayed through noVNC; Mac NVIDIA drivers, XQuartz and host X11 sockets
are not prerequisites.

## Validation status (2026-10-05)

**The native ARM64 Docker main path is feasible; complete production-stack and
flight acceptance are not finished.** No AMD64 emulation was used for these tests.

| Evidence | Verified boundary |
|---|---|
| Gazebo Harmonic 8.15 | ARM64 gravity/collision probe passed; GUI displayed through noVNC |
| PX4 v1.17.0 | ARM64 probe compiled and ran gz_x500; IMU, position and MAVLink observed |
| Production core / nav images | ARM64 source builds passed, including MAVROS 2.15.1 and px4ctrl |
| Production ROS data path | Connected/unarmed SITL; 116 IMU, 82 odometry and 10 adapted-state messages; planner/NMPC nodes present; thrust_scaling=1.0 |
| FastDDS | Production nav/core shared 10 SHM mappings; host IPC/network delivery passed |
| NMPC / simple-sim | Production ARM64 image ran the real solver for 1 second / 50 control steps |
| QGC v5.1.5 / desktop | Production ARM64 image displayed through noVNC with software rendering; release checksum verified |
| CLI / Compose | 45 tests passed, including ARM64/AMD64 headless and GUI profile parsing |

The production ROS test used the previously validated ARM64 SITL probe image,
**not a newly built production PX4 image**. px4ctrl also reported sensor/odometry
rate warnings below 100 Hz; data delivery does not prove flight-rate performance.

**Deferred: the new production PX4 image rebuild will temporarily not be completed
on Mac.** Its local attempt was interrupted when disk space fell below 1 GiB.
Finish the unchanged ARM64/AMD64 recipes on native Linux runners instead; this
is a resource/acceptance deferral, not removal of Mac ARM64 support. Do not retry
the large PX4 build on this Mac unless the deferral is explicitly lifted.

Still unverified: all-new production-image startup, takeoff/hover/NMPC handover/
landing, native AMD64 execution, real-profile hardware/LIO and complex sensor
performance. The manually triggered Linux CI workflow has not run remotely.

Temporary PX4, QGC and desktop images/caches are disposable after validation.
Keep the evidence in `.artifacts/feasibility/gazebo-multiarch/` (logs, JSON,
recipes and screenshots); cleanup is not a reversal of the recorded results.

## Prerequisites and target

- Git, uv, Docker with BuildKit and Docker Compose v2+.
- On macOS, Docker Desktop with host-network support (4.34+); enable host
  networking in Desktop settings. Here host network/IPC mean the Linux VM.
- Allow ample build disk space: PX4, OpenCV, ROS and optional GUI dependencies
  are large. Do not prune unrelated projects to make a build pass.
- A checkout at an **absolute** `AP_PNC_DIR`.

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.config()"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build()"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.up()"
# Stop only this Compose project (also stops its optional GUI services):
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.down()"
```

Build/up/config accept `--arch native|arm64|amd64` (aarch64/x86_64 aliases also
work). Native means the CLI host CPU. For a remote Docker daemon or another
deployment target, choose `--arch` explicitly and keep build/run consistent.
The recipes support both targets; each target still needs native acceptance.

A missing bundle is generated inside the pinned Docker builder. Existing
bundles must be regenerated after solver/API changes:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(services=['ap-pnc'])"
```

`arm64` selects `libnmpc_bundle_aarch64.a`; `amd64` selects
`libnmpc_bundle_x86_64.a`. Image builds verify **every ELF member** before copying
the selected archive to a container-local alias. The host's unsuffixed alias
is never used as an image build input. Solver build caches are isolated in
`.artifacts/nmpc_build/<aarch64|x86_64>/` and publication uses atomic replacement.
NMPC is statically linked: replacing a runtime archive cannot update an
existing executable. Rebuild core and simple-sim after solver changes.

## What starts

| Service | Default role |
|---|---|
| px4-simulator | Stock PX4 v1.17.0 + Gazebo Harmonic gz_x500, headless |
| gcs-heartbeat | Simulation GCS heartbeat only; no arm/mode/control commands |
| nav_infra | Source-built MAVROS 2.15.1 + px4ctrl, sim profile |
| ap-pnc | planner + NMPC composable components |
| ros2-base | Shared target-native build base; not a runtime service |

The stack starts automatically; do **not** spawn a second SITL or core process
with `docker exec`. Startup does not issue a takeoff request. Simulator process
health is not flight acceptance; observe real data before arming.

All services have `ipc: host`. ROS2, PX4, GCS and GUI clients share host network;
FastDDS is explicit and SHM is not disabled. The VNC desktop itself uses bridge
networking because it performs no DDS/Gazebo discovery. Its HTTP port is bound
to **127.0.0.1 only**; VNC and TCP X11 are not published. Do not expose this
unauthenticated local desktop to a LAN/public interface.

Runtime data goes under `.artifacts/benchmark/gazebo/<docker_arch>/`.
Images retain their own install trees. Core YAML mounts read-only; nav config
and C++ changes require an image rebuild. ROS logs are container artifact-owned.

## Optional container desktop

```bash
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(gui=True)"
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.up(gui=True)"
# Open http://127.0.0.1:6088/vnc.html?autoconnect=1&resize=scale
```

The `gui` profile adds Xvfb/Fluxbox/noVNC, QGC v5.1.5 and a Gazebo GUI client.
QGC's two release hashes are pinned in its Dockerfile. AppImage contents are
extracted during the build, so runtime needs no FUSE mount or privileged mode.
GUI clients share the container desktop's UNIX X11 socket, stored under
`.artifacts/desktop/<docker_arch>/`, not the Mac's `/tmp/.X11-unix`.

Mesa/llvmpipe is the CPU baseline. A visible GUI does not establish camera or
GPU-LiDAR throughput. NVIDIA hardware acceleration needs a separately tested
Linux GPU overlay; the default configuration never requests an NVIDIA runtime.

## sim versus real image targets

`nav_infra` defaults to the lean `sim` Docker target, excluding Livox/PCL/LIO.
The optional `real` target adds Livox, FAST-LIO and EKF packages:

```bash
uv run --project "$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.build(services=['nav_infra'], nav_target='real')"
```

The image tag includes both architecture and target, e.g. `nav_infra:arm64-sim`.
Gazebo Compose always launches `profile:=sim`, even when selecting the fuller
image with `up --nav-target real`. Real-aircraft launches are separate and must
not be enabled as part of simulation startup.

## Raw Compose and acceptance

The CLI supplies the environment and profiles. For raw Compose, explicitly set
both root and target. Build the shared base first with the `build` profile:

```bash
The architecture is a tooling argument: `tooling.docker_runtime` resolves it and
writes it into the generated Compose override. Nothing is exported.
# Paths remain absolute regardless of cwd.
docker compose -f "$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml" \
  --profile build build ros2-base
docker compose -f "$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml" \
  --profile build build ap-pnc nav_infra px4-simulator
```

Read-only acceptance after startup:

```bash
docker exec nav_infra /entrypoint.sh python3 \
  /workspace/infra/sim_infra/gazebo/scripts/verify_runtime.py
```

The probe requires connected/unarmed PX4, IMU, local odometry, px4ctrl state,
planner/NMPC nodes and `setpoint_raw.thrust_scaling=1.0`. It neither arms nor
publishes setpoints. Full takeoff/hover/NMPC handover/landing is a separate
acceptance step in the `ap-pnc-gazebo-e2e` skill.

`.github/workflows/docker-multiarch.yml` provides manually triggered native
AMD64 and ARM64 build/smoke jobs; it is not evidence that CI has already run.
The independent K3s deployment manifests are not migrated by this Compose
change; sync their image tags separately before using that backend.
