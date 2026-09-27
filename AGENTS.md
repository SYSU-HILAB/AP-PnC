# AGENTS.md — working conventions

Local, gitignored project conventions for coding agents.

## Artifacts: everything under `<proj_root>/.artifacts/`

All generated outputs live under the **absolute** `<proj_root>/.artifacts/`,
never inside a source tree:

| Output | Location |
|---|---|
| colcon build / install / log | `.artifacts/colcon/` |
| acados staging source + build | `.artifacts/acados_src/`, `.artifacts/acados/`, `.artifacts/acados_static/` |
| generated OCP code | `.artifacts/c_generated_code/` |
| NMPC static bundle | `.artifacts/nmpc_solver/libnmpc_bundle.a` |
| pybind11 modules (+ their build temp) | `.artifacts/bindings/` |
| benchmark runs | `.artifacts/benchmark/<name>/` |

Rules:

- No `build/`, `install/`, `log/`, or generated files inside `core/`, at the
  project root, or in any tracked source directory.
- colcon log/install/build bases all point under `.artifacts/colcon/`
  (`COLCON_LOG_PATH` is set by `ap_pnc.env.setup_env`).
- Resolve paths **absolutely**, anchored at the project root
  (`ap_pnc.env.get_git_root()` in Python, `AP_PNC_DIR` in C++/containers).
- Do not introduce cwd-relative paths for configs, artifacts, or build output.

## Host

The development host is **Linux**. macOS is not a supported development host:
the acados codegen and the static bundle (`libnmpc_bundle.a`) are Linux-only
(t_renderer publishes no macOS build). On macOS the Python CLI still imports and
runs its non-acados paths; `ap-pnc install` / `gen-nmpc-lib` require Linux.

## Runtime configuration

Nodes read their yaml from, by convention:

```
<AP_PNC_DIR>/bringup/config/planning.yaml   (planner_node)
<AP_PNC_DIR>/bringup/config/nmpc.yaml       (nmpc_node)
```

The aerodynamics alpha-solver config lives with its owner:
`infra/sim_infra/aerodynamics/config/aero_alpha_solver.yaml`.

`AP_PNC_DIR` is the **only** path environment variable; no per-file env vars.

## ROS middleware

Use the RMW defaults. Do not set `RMW_IMPLEMENTATION`,
`FASTRTPS_DEFAULT_PROFILES_FILE`, or FastDDS QoS profiles in launch files,
Dockerfiles, or manifests.
