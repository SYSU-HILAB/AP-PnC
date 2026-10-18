---
name: ap-pnc-bootstrap
description: Bootstrap AP-PnC through the Docker-only toolchain, generate architecture-tagged NMPC static libraries, and build the offline simulator. uv is only the orchestration layer; supports explicit cross-architecture targets.
---

# AP-PnC bootstrap (Docker only)

## Requirements

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and
[Docker](https://docs.docker.com/get-started/get-docker/). Set an absolute
`AP_PNC_DIR` pointing to the checkout. Do not install ROS, acados, t_renderer,
compilers or numerical Python dependencies on the host.

## Build

```bash
uv sync --project "$AP_PNC_DIR"
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
```

`uv` orchestrates Docker. The pinned Linux builder contains the compiler,
acados shared/static libraries, acados_template and t_renderer. The pin lives in
`tooling/acados_setup.py`; that module no longer installs a host toolchain.

## Targets, including cross-architecture builds

The default selects the host CPU architecture, never a host-native OS build.
Both explicit targets are available from either supported host architecture:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch x86_64
```

- `aarch64`: Docker `linux/arm64`, output
  `$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_aarch64.a`.
- `x86_64`: Docker `linux/amd64`, output
  `$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_x86_64.a`.
- No untagged alias or host compilation fallback. A cross-target build requires
  Docker support for executing the requested platform; never silently change it.
- Generated code and caches are isolated under
  `$AP_PNC_DIR/.artifacts/nmpc_build/<solver_arch>/`.

## Other builds and tests

`ap-pnc build aero|planner|nmpc|all|bindings|planner-bindings` and numerical/ROS
`ap-pnc test` commands dispatch Linux containers. Their `--arch arm64|amd64`
option also accepts `aarch64|x86_64`; output lives under
`$AP_PNC_DIR/.artifacts/docker-tools/<docker_arch>/`. Linux Python extensions
must be consumed in the matching Linux container, never imported by host uv.
The separate nav_infra repository is not part of `build all`.

## Verify

Verify image architecture, every archive member's ELF architecture, actual
linking and the relevant runtime/tests separately. Codegen success does not
prove ROS linking or flight acceptance. AMD64 execution under emulation is not
native AMD64 acceptance. Retain evidence under `$AP_PNC_DIR/.artifacts/tests/`.

Solver changes require rebuilding statically linked consumers. See the
ap-pnc-docker skill and the Gazebo README before building production services;
the large production PX4 rebuild remains deferred on this Mac.
