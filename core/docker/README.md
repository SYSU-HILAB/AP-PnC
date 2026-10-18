# Linux build and development containers

All hosts use Docker for codegen, compilation, Python bindings and numerical/ROS
tests. `uv` only orchestrates these jobs; no host ROS/acados/compiler installation
is required. One Dockerfile serves both target architectures.

| Target | Docker platform | Static solver archive |
|---|---|---|
| ARM64 | linux/arm64 | libnmpc_bundle_aarch64.a |
| AMD64 | linux/amd64 | libnmpc_bundle_x86_64.a |

The default uses the host CPU architecture. Explicit cross-architecture targets
are supported when Docker can execute the selected platform; there is no silent
fallback, host-native compile branch or untagged bundle alias.

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch x86_64
uv run --project "$AP_PNC_DIR" ap-pnc build planner --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc build nmpc --arch amd64
uv run --project "$AP_PNC_DIR" ap-pnc build planner-bindings --arch arm64
uv run --project "$AP_PNC_DIR" ap-pnc test colcon --arch arm64
```

`build all` builds the core workspace, not the separate nav_infra repository.
Tools use read-only sources, host IPC and no external runtime network. Their
artifacts are isolated under `.artifacts/docker-tools/<arm64|amd64>/`, including
colcon output and Linux bindings. Consume bindings inside the matching container,
never from the host Python interpreter. Codegen caches separately live under
`.artifacts/nmpc_build/<aarch64|x86_64>/`.

## Interactive ROS development

```bash
uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.dev()"
# Explicit cross/deployment target:
uv run --project "$AP_PNC_DIR" python -c "from tooling import docker_runtime as d; d.dev(arch='amd64')"
```

The dev shell has a separate cache under
`.artifacts/docker-dev/<arm64|amd64>/colcon/`. It mounts the matching tagged solver
archive without renaming it. Both services use host network and host IPC;
bare `up` starts neither architecture profile. Do not use these interactive
network settings for synthetic probes near aircraft networks.

For self-contained simulation images, use `ap-pnc build simple-sim`.
See [Gazebo deployment](../../infra/sim_infra/gazebo/README.md) for deployment and
acceptance limits. Do not resume the deferred production PX4 rebuild on this Mac.
