# Tooling

Python orchestration for AP-PnC. The `ap-pnc` CLI builds and tests the
[core](../core/README.md) and runs the simulation stacks in
[infra](../infra/README.md). uv is the host-side launcher; code generation,
compilation, bindings and numerical/ROS tests run in Linux Docker.

## Usage

Install uv and Docker, then set `AP_PNC_DIR` to the absolute checkout path:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv sync --project "$AP_PNC_DIR"
uv run --project "$AP_PNC_DIR" ap-pnc --help
```

| Command | Purpose |
| --- | --- |
| `gen-nmpc-lib` | Generate acados solver code and an architecture-tagged static archive. |
| `build` | Build core packages, bindings or the simple-sim image. |
| `benchmark` | Run offline planning/tracking comparisons and save results. |
| `dev-sim` | Build, prepare and run the conditional ROS2 development simulator. |
| `test` | Run pytest, numerical checks or colcon tests in Docker. |
| `lint` | Check tests with Ruff; `lint fmt` formats them. |

Use `ap-pnc <command> --help` for current options and subcommands.
For a runnable sequence, see the [quick start](../README.md#quick-start).
The development simulator requires a local practical actuator profile; it is
not the asset-free benchmark.

## Implementation

- [cli.py](cli.py) and [commands](commands/) — CLI registration and handlers.
- [env.py](env.py) — project-root discovery, environment setup and path helpers.
  Python uses `get_git_root()` and `project_path()` for project inputs, and
  `artifact_path()` to confine generated outputs under `.artifacts/`.
- [container_tools.py](container_tools.py) and [docker_runtime.py](docker_runtime.py)
  — architecture selection and Docker build/test/development orchestration.
- [acados_setup.py](acados_setup.py) and [nmpc_gen](nmpc_gen/) — pinned solver
  toolchain and code generation.
- [simple_stack.py](simple_stack.py), [dev_sim.py](dev_sim.py) and
  [city_demo.py](city_demo.py) — offline and interactive simulation orchestration.
- [actuator_profile.py](actuator_profile.py) — locally extracted actuator profiles.
- [observability.py](observability.py) — CLI telemetry support.
- [paper](paper/) — paper benchmarks, methods and figure tooling.

## Development checks

```bash
uv run --project "$AP_PNC_DIR" ap-pnc lint
uv run --project "$AP_PNC_DIR" ap-pnc test all
```

The default Docker target matches the host CPU architecture. See the
[core Docker guide](../core/README.md#linux-build-and-development-containers) for explicit targets, isolated
caches and consuming Linux bindings. Do not import those bindings on the host.
All generated outputs belong under `$AP_PNC_DIR/.artifacts/`.
