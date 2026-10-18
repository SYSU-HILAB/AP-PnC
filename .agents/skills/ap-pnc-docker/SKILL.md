---
name: ap-pnc-docker
description: Build and test the public AP-PnC core in Linux Docker containers. Use for ARM64/AMD64 targeting, colcon and Python bindings, isolated artifact caches, static solver linkage, and distinguishing public core tools from experimental Gazebo deployment.
---

# Docker core development

Read `$AP_PNC_DIR/AGENTS.md` and inspect the relevant `--help` before running.
The public CLI groups are `gen-nmpc-lib`, `build`, `benchmark`, `dev-sim`, `test`
and `lint`. Do not restore removed `docker`, `stack`, `install`, `start` or
`aerodyn` command groups. `$AP_PNC_DIR/tooling/docker_runtime.py` is an internal
API, not a public `ap-pnc docker` command.

## Build and test

Set absolute `AP_PNC_DIR`. These commands dispatch Linux containers:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc build planner --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc build nmpc --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc build planner-bindings --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc test vmap --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc test colcon --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc test colcon-result --arch arm64
```

Choose `amd64` for the other target; these build/test options also accept
`aarch64`/`x86_64`. The default is the host CPU. `build all` builds the core
workspace, not nav_infra. `test all` runs the legacy lu_mpc numerical tests,
not every repository test. Run colcon tests after building the relevant packages.
For the offline stack use `ap-pnc build simple-sim`; for the interactive sensor
image use `ap-pnc dev-sim build`. Neither currently exposes `--arch`.

Use `$AP_PNC_DIR/tooling/commands/build.py`,
`$AP_PNC_DIR/tooling/commands/test.py` and
`$AP_PNC_DIR/tooling/container_tools.py` as the source of truth.

## Isolation and outputs

- Tool jobs mount source read-only, use host IPC and disable runtime networking.
  Image/dependency builds still need network access.
- Colcon, Linux bindings and test artifacts stay under
  `$AP_PNC_DIR/.artifacts/docker-tools/<arm64|amd64>/` on the host.
- Consume bindings inside the matching Linux container, not the host uv Python.
  Do not mix host/target OSes, architectures, Python ABIs or CMake caches.
- Solver generation uses `$AP_PNC_DIR/.artifacts/nmpc_build/<aarch64|x86_64>/`;
  consumers use the matching tagged archive under
  `$AP_PNC_DIR/.artifacts/nmpc_solver/`. No untagged alias or host build fallback.
- NMPC is statically linked. Rebuild consumers after regenerating the library;
  replacing or mounting an archive does not update an existing executable.

The optional interactive core shell is an internal helper:

```bash
uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.dev(arch='arm64')"
```

It uses a separate `$AP_PNC_DIR/.artifacts/docker-dev/<docker_arch>/colcon/`
cache and host networking/IPC. It is not the network-isolated test launcher;
keep it away from aircraft networks.

## Verification boundary

Check image/container architecture, archive ELF members, actual linkage and the
relevant runtime behavior separately. Compose parsing and codegen are not
runtime acceptance; emulated AMD64 tests are not native AMD64 acceptance.
Do not disable FastDDS SHM to hide IPC problems or prune unrelated resources.
Run `uv run --project "$AP_PNC_DIR" ap-pnc lint` after code edits.

Gazebo recipes are experimental and need external nav sources not tracked here.
Use the `ap-pnc-gazebo-readiness` skill before considering deployment, rather
than calling an unqualified internal `build()`/`up()` from a fresh clone.
