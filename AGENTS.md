# AGENTS.md — working conventions

Project conventions for coding agents.

## Path discipline: absolute or explicitly anchored at `AP_PNC_DIR`

For project navigation, file access, runtime resources, and generated output,
**the only permitted path base is the project root (`AP_PNC_DIR`)**:

- Use an absolute path, or explicitly anchor the path at `AP_PNC_DIR`
  (for example, `$AP_PNC_DIR/core/bringup/config/planning.yaml`).
- `AP_PNC_DIR` itself must be an absolute project-root path. In Python, obtain
  that root through `tooling.env.get_git_root()` and join paths to it.
- Never rely on the current working directory. Bare paths such as `core/...`,
  `./config/...`, `../...`, and `Path.cwd() / ...` are not permitted for project
  navigation or resource/output lookup, even when the current directory happens
  to be the project root.
- Agent `read` / `edit` / `write` tool paths must be resolved absolute paths;
  these tools must not be given unexpanded shell variables.
- Shell navigation and file operations must use absolute or explicitly
  `AP_PNC_DIR`-anchored operands. Use `git -C "$AP_PNC_DIR" ...` for repository
  operations. Setting the shell cwd does not make subsequent bare paths valid.
- Resolve user-supplied project-root-relative paths against `AP_PNC_DIR`, never
  against cwd. If an interface requires absolute paths, validate/reject relative
  input rather than silently interpreting it against cwd.
- Apply the same rule inside containers, using their absolute `AP_PNC_DIR`.
  Do not introduce additional per-file path environment variables.

Examples:

```bash
cd "$AP_PNC_DIR"
rg "pattern" "$AP_PNC_DIR/core"
git -C "$AP_PNC_DIR" status --short
cmake -S "$AP_PNC_DIR/infra/sim_infra/simple_sim" \
      -B "$AP_PNC_DIR/.artifacts/colcon/build/simple_sim"
```

## Artifacts: everything under `<proj_root>/.artifacts/`

All generated outputs live under the **absolute** `<proj_root>/.artifacts/`,
never inside a source tree:

| Output | Location |
|---|---|
| colcon build / install / log | `<AP_PNC_DIR>/.artifacts/colcon/` |
| acados staging source + build | `<AP_PNC_DIR>/.artifacts/acados_src/`, `<AP_PNC_DIR>/.artifacts/acados/`, `<AP_PNC_DIR>/.artifacts/acados_static/` |
| generated OCP code | `<AP_PNC_DIR>/.artifacts/c_generated_code/` |
| NMPC static bundle | `<AP_PNC_DIR>/.artifacts/nmpc_solver/libnmpc_bundle_<arch>.a` (host-native alias: `libnmpc_bundle.a`) |
| pybind11 modules (+ their build temp) | `<AP_PNC_DIR>/.artifacts/bindings/` |
| benchmark runs | `<AP_PNC_DIR>/.artifacts/benchmark/<name>/` |
| aerodynamics CSV datasets / ML weights | `<AP_PNC_DIR>/.artifacts/aerodynamics/` |
| paper figure cache and figures | `<AP_PNC_DIR>/.artifacts/paper/{cache,figures}/` |
| pytest / ruff caches | `<AP_PNC_DIR>/.artifacts/tests/`, `<AP_PNC_DIR>/.artifacts/ruff_cache/` |

Rules:

- No `build/`, `install/`, `log/`, or generated files inside `core/`, at the
  project root, or in any tracked source directory.
- colcon log/install/build bases all point under `.artifacts/colcon/`
  (`COLCON_LOG_PATH` is set by `tooling.env.setup_env`).
- Resolve paths **absolutely**, anchored at the project root
  (`tooling.env.get_git_root()` in Python, `AP_PNC_DIR` in C++/containers).
- Python code resolves root-relative inputs via `tooling.env.project_path()` and
  confines generated outputs via `tooling.env.artifact_path()`; C++ aerodynamics
  code uses `aerodynamics/paths` from `aerodynamics/project_paths.hpp`.
- Docker Compose files resolve every context, dockerfile, and volume against an
  absolute `AP_PNC_DIR` (`${AP_PNC_DIR:?...}`); the tooling exports it (`tooling.docker_runtime`, `tooling.simple_stack`, and the
  K3s simulation management).
- Do not introduce cwd-relative paths for configs, artifacts, or build output.

## Host, target architecture, and iteration phases

The ROS2/acados toolchain runs on **Linux**; this does not mean the host
must be Linux or the CPU must be AMD64. **On Apple Silicon Macs, the default
iteration target is Linux ARM64 in Docker**, not emulated Linux AMD64.

| Development host | Default Docker target | ROS2 base image / service | NMPC bundle |
|---|---|---|---|
| macOS Apple Silicon (ARM64) | `linux/arm64` | `ap-pnc-ros2-base:arm64` / `ros2-dev-arm64` | `libnmpc_bundle_aarch64.a` |
| Linux ARM64 | `linux/arm64` | `ap-pnc-ros2-base:arm64` / `ros2-dev-arm64` | `libnmpc_bundle_aarch64.a` |
| Linux x86_64 / Intel Mac | `linux/amd64` | `ap-pnc-ros2-base:amd64` / `ros2-dev-amd64` | `libnmpc_bundle_x86_64.a` |

`arm64` (Docker / macOS) and `aarch64` (Linux / bundle name) identify the
same CPU architecture; likewise `amd64` and `x86_64`. macOS ARM64 binaries
are not Linux ARM64 binaries: native dependencies must be built inside
Linux, even when both sides use ARM64.

### Phase 1 — select the target before building

- Use the host-native CPU architecture by default. On Apple Silicon, select
  `linux/arm64` and the ARM64 Compose service. Both architectures use the
  same `core/docker/ros2-base.dockerfile`; do not restore duplicated
  architecture-specific Dockerfiles.
- AMD64 is a separate, explicitly requested deployment/cross-platform target,
  not a fallback for Mac development. Do not silently switch to AMD64,
  use emulation, or reuse x86_64 artifacts to work around an ARM64 failure.
  Report the unsupported dependency or configuration instead.
- Specify matching platforms for Docker build and run. A base-image tag alone
  is not sufficient evidence of the actual image architecture.

### Phase 2 — build Linux artifacts for that target

- On Apple Silicon, generate the solver with
  `uv run ap-pnc gen-nmpc-lib --arch aarch64`. The default
  `uv run ap-pnc gen-nmpc-lib` also selects the host CPU architecture and
  uses Docker on macOS. For an explicitly requested AMD64 target, use
  `--arch x86_64` instead.
- The pinned builder (`core/docker/nmpc-builder.dockerfile`) contains the
  Linux toolchain; acados codegen needs a Linux t_renderer, not a macOS
  binary. Toolchain versions (acados commit, t_renderer, acados_template)
  must stay in sync with `tooling/acados_setup.py:ACADOS_COMMIT`.
- Keep the base image, builder platform, ROS2 compilation, and linked bundle
  on the same target architecture. Consume the arch-tagged bundle;
  `libnmpc_bundle.a` is only a host-native convenience alias, not a portable
  or architecture-independent library.
- Never reuse colcon/CMake caches, install trees, or compiled bindings from
  another OS/architecture. Docker solver generation uses
  `.artifacts/nmpc_build/<solver_arch>/`; dev containers use
  `.artifacts/docker-dev/<docker_arch>/colcon/`. Do not mount all host
  `.artifacts/` over an image's install tree.
- the Compose layer (`tooling.docker_runtime`) and `ap-pnc build simple-sim` select the native CPU by default;
  `--arch arm64|amd64` is explicit. Raw Compose requires absolute
  `AP_PNC_DIR`. The architecture is a tooling argument (see
  `tooling/docker_runtime.py`), not a variable; these architecture settings
  are not additional project-path variables.

### Phase 3 — run and verify on the same target

- Set `ipc: host` consistently on Docker services; ROS2 containers using
  FastDDS must share the host IPC namespace so its SHM transport can access
  the same `/dev/shm` segments. Host networking alone does not share IPC.
  On macOS this host is Docker's Linux VM, not the macOS IPC namespace.
  Keep DDS discovery/network configuration consistent as well: shared IPC
  alone does not establish discovery across separate network namespaces.
  Verify message delivery and shared SHM mappings, not just container startup;
  do not silently disable SHM or make UDP-only transport the default workaround.
- Verify the built image/container architecture and the bundle's object-file
  architecture before claiming the ARM64 workflow works. Code generation
  success alone does not prove ROS2 linking or runtime success.
- Host `ap-pnc build ...` assumes Linux with ROS2/colcon;
  on macOS, perform those toolchain steps inside the target Linux container.
- Gazebo defaults to headless PX4 + GCS heartbeat + nav_infra + core.
  `--gui` adds a container Xvfb/Mesa desktop, QGC and Gazebo GUI via noVNC
  on host loopback port 6088, without Mac NVIDIA/X11 assumptions. Every
  service uses host IPC; ROS2 and SITL also share host networking.
- `nav_infra` defaults to the lean `sim` build target; `--nav-target real`
  adds the hardware/LIO packages. Gazebo still launches the sim profile.
  A recipe supporting both architectures is not proof that both builds,
  complex sensors, or the complete flight sequence have passed.
- Current acceptance boundary (2026-10-05): ARM64 production core/nav
  (MAVROS 2.15.1), real solver, FastDDS SHM and QGC/noVNC are verified;
  see the Gazebo README for evidence and rate warnings. Complete flight
  and native AMD64 runtime acceptance remain unverified.
- **Temporarily do not finish the new production PX4 rebuild on this Mac.**
  Its build was stopped for disk pressure; continue that acceptance on
  native Linux ARM64/AMD64 runners. Do not silently retry a large Mac PX4
  build or claim the tested SITL probe is the new production image.
  QGC/desktop/probe images may be reclaimed after preserving evidence.
- NMPC is statically linked: regenerating or mounting a bundle does not
  update an existing executable. Rebuild the core/simple-sim image after
  solver changes. Config-only core edits can use the read-only config mount.

## Planner / NMPC package layout

Both `core/ros_packages/planner` and `core/ros_packages/nmpc` use:

- `src/`: ROS-free C++ algorithms, public headers and internal dependencies.
- `node/`: ROS interfaces only.
- `python/`: Python bindings only when needed; do not add an empty placeholder.

Do not reintroduce package-root `core/`, `solver/` or `controller/` directories.
Keep CMake, solver generation, Python bindings and simulator consumers pointed
at the same owner sources when moving files.

## Code style: lint after every change

Lint rules are agent-enforced, not README-documented:

- After **every** code edit, run `uv run ap-pnc lint` before declaring the
  change done; format with `uv run ap-pnc lint fmt` when files need it.
- Python follows the ruff configuration in `pyproject.toml`; C++ follows the
  repo `.clang-format` (enforced by pre-commit).
- Fix the code instead of silencing or downgrading lint findings; new lint
  errors must never be committed.

## OpenTelemetry: project-wide observability

- Follow the local (untracked) `docs/observability.md` notes for the full coverage
  and rollout contract.
  The user requests deep CLI/dora/planner/NMPC/plant/actuator/ROS/artifact
  observability, not just generic process logs. State implemented coverage
  separately from planned hooks and synthetic probes.
- Reuse dora's existing Rust tracing/OTLP, metrics and reserved metadata context
  before adding a parallel subscriber or custom trace envelope. Prefer public
  node APIs; `dora-tracing` itself is explicitly internal/unstable. The C++ node
  API is a Rust-backed CXX bridge, not automatic numerical-function tracing.
- Preserve scientific data and transactional control behavior. Do not use
  telemetry as a replacement for saved reference/CSV/manifest/Rerun validation.
- OTel span time is wall clock; simulation time is an explicit attribute.
  Keep future NMPC reference knots distinct from current plant state.
- No network/file export or force-flush on real-time numerical paths. Use
  bounded queues, aggregate metrics and measured diagnostic sampling.
- Filter sensitive attributes; no raw argv/environment/private model content
  or exception text. Run IDs/ticks/hashes are not metric labels. Local-only
  Collector is the default; remote telemetry export needs explicit authorization.
- Each newly instrumented layer needs off/on numerical identity and overhead/
  outage tests. Do not claim full coverage from a Collector health check.

## Runtime configuration

Nodes read their yaml from, by convention:

```
<AP_PNC_DIR>/bringup/config/planning.yaml   (planner_node)
<AP_PNC_DIR>/bringup/config/nmpc.yaml       (nmpc_node)
```

`AP_PNC_DIR` is the **only** path environment variable; no per-file env vars
(`AERO_SIM_DATA_DIR` and `NMPC_BUNDLE_PATH` were removed for this reason).
