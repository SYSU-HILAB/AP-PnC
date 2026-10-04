# AGENTS.md — working conventions

Project conventions for coding agents.

## Path discipline: absolute or explicitly anchored at `AP_PNC_DIR`

For project navigation, file access, runtime resources, and generated output,
**the only permitted path base is the project root (`AP_PNC_DIR`)**:

- Use an absolute path, or explicitly anchor the path at `AP_PNC_DIR`
  (for example, `$AP_PNC_DIR/core/bringup/config/planning.yaml`).
- `AP_PNC_DIR` itself must be an absolute project-root path. In Python, obtain
  that root through `ap_pnc.env.get_git_root()` and join paths to it.
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
| NMPC static bundle | `<AP_PNC_DIR>/.artifacts/nmpc_solver/libnmpc_bundle.a` |
| pybind11 modules (+ their build temp) | `<AP_PNC_DIR>/.artifacts/bindings/` |
| benchmark runs | `<AP_PNC_DIR>/.artifacts/benchmark/<name>/` |
| aerodynamics CSV datasets / ML weights | `<AP_PNC_DIR>/.artifacts/aerodynamics/` |
| paper figure cache and figures | `<AP_PNC_DIR>/.artifacts/paper/{cache,figures}/` |
| pytest / ruff caches | `<AP_PNC_DIR>/.artifacts/tests/`, `<AP_PNC_DIR>/.artifacts/ruff_cache/` |

Rules:

- No `build/`, `install/`, `log/`, or generated files inside `core/`, at the
  project root, or in any tracked source directory.
- colcon log/install/build bases all point under `.artifacts/colcon/`
  (`COLCON_LOG_PATH` is set by `ap_pnc.env.setup_env`).
- Resolve paths **absolutely**, anchored at the project root
  (`ap_pnc.env.get_git_root()` in Python, `AP_PNC_DIR` in C++/containers).
- Python code resolves root-relative inputs via `ap_pnc.env.project_path()` and
  confines generated outputs via `ap_pnc.env.artifact_path()`; C++ aerodynamics
  code uses `aerodynamics/paths` from `aerodynamics/project_paths.hpp`.
- Docker Compose files resolve every context, dockerfile, and volume against an
  absolute `AP_PNC_DIR` (`${AP_PNC_DIR:?...}`); `ap-pnc docker/sim` exports it.
- Do not introduce cwd-relative paths for configs, artifacts, or build output.

## Host

The development host is **Linux**. macOS is not a supported development host:
the acados codegen and the static bundle (`libnmpc_bundle.a`) are Linux-only
(t_renderer publishes no macOS build). On macOS the Python CLI still imports and
runs its non-acados paths; `ap-pnc install` / `gen-nmpc-lib` require Linux.

## Code style: lint after every change

Lint rules are agent-enforced, not README-documented:

- After **every** code edit, run `uv run ap-pnc lint` before declaring the
  change done; format with `uv run ap-pnc lint fmt` when files need it.
- Python follows the ruff configuration in `pyproject.toml`; C++ follows the
  repo `.clang-format` (enforced by pre-commit).
- Fix the code instead of silencing or downgrading lint findings; new lint
  errors must never be committed.

## Runtime configuration

Nodes read their yaml from, by convention:

```
<AP_PNC_DIR>/bringup/config/planning.yaml   (planner_node)
<AP_PNC_DIR>/bringup/config/nmpc.yaml       (nmpc_node)
```

The aerodynamics alpha-solver config lives with its owner:
`<AP_PNC_DIR>/infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml`.

`AP_PNC_DIR` is the **only** path environment variable; no per-file env vars
(`AERO_SIM_DATA_DIR` and `NMPC_BUNDLE_PATH` were removed for this reason).
