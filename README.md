# AP-PnC

> **Aerodynamic Prior-Free Coordinated Trajectory Generation and Tracking Control
> for a Tail-Sitter UAV**
> Erchao Rong, Zihao Liu, Junning Liang, Jianguo Wang, Xiao Jie, Haoran Fu,
> Ziliang Chen, Ximin Lyu — arXiv:2609.11698 · <https://arxiv.org/abs/2609.11698>

```bibtex
@article{rong2026apPnc,
  title  = {Aerodynamic Prior-Free Coordinated Trajectory Generation and
            Tracking Control for a Tail-Sitter UAV},
  author = {Rong, Erchao and Liu, Zihao and Liang, Junning and Wang, Jianguo and
            Jie, Xiao and Fu, Haoran and Chen, Ziliang and Lyu, Ximin},
  journal = {arXiv preprint arXiv:2609.11698},
  year   = {2026},
  url    = {https://arxiv.org/abs/2609.11698}
}
```

Two phase-specific models, as in the paper: the planner uses the phi-theory
flatness under coordinated flight, and the NMPC tracks with a simplified model
whose single longitudinal aerodynamic parameter is estimated online.

## Run it: the offline simulator

This is the shortest path from a fresh clone to a trajectory, and it needs no
ROS, no Gazebo, no flight hardware and no private data. Everything runs in Linux
containers; on macOS this runs a native `linux/arm64` container, not emulation.
`uv` is the only Python entry point: `uv run --project "$AP_PNC_DIR" ap-pnc …`; the
CLI resolves every path it needs from `AP_PNC_DIR`.

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
git clone https://github.com/WarriorHanamy/AP-PnC.git "$AP_PNC_DIR"

# 1. Python environment + the acados solver bundle (Linux; macOS uses Docker).
uv sync --project "$AP_PNC_DIR" --group dev
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib

# 2. Build the offline stack (Rust dora nodes + the C++ simulator).
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" build

# 3. Plan, simulate and record one run. It prints the JOB path.
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" run 0 ideal none 6 0
#    run [duration_s|0=whole] [ideal|matlab-vtol] [aero] [v_max|configured] [hold_s]

# 4. Re-run the independent decoder + acceptance checks on that job.
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test "$JOB"
```

`run` prints `JOB=/…/runs/<timestamp>-<pid>`; that directory holds the planner
reference, `steps.csv`, the actual NMPC reference trace, `manifest.json`,
`metrics.json`, the Rerun recording and the verification receipt. Rerun is
offline only: the scientific authority is the CSV/YAML/metrics, and the viewer
exports never re-plan or re-simulate.

Optional figures for a group of runs (plotting only, never re-simulates):

```bash
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" plot \
  --output "$AP_PNC_DIR/.artifacts/benchmark/MY_GROUP"
```

The `ideal` profile is the rate-loop plant and needs no calibration data. The
`matlab-vtol` profile is the four-motor plant and reads extracted numeric
parameters from `.artifacts/vtol-matlab/`; it is only available where that local
extraction exists.

## Configuration is the single source of truth

The horizon, objective, command bounds and estimator settings live in
`core/bringup/config/nmpc.yaml`; the corridor, seed curve, solver cost and
flatness parameters live in `core/bringup/config/planning.yaml`; the plant,
actuator, aerodynamics model and mass live in
`core/bringup/config/simple_sim.yaml`. Read those files rather than a table
here; each run snapshots them into its own job directory, so any result can be
traced back to the exact configuration that produced it.

Two constraints are worth knowing before editing them:

- Weight, bound and horizon changes are compiled into the solver: regenerate
  with `ap-pnc gen-nmpc-lib` and relink, otherwise the run is rejected by the
  ABI check.
- The prediction uses ground speed, so non-zero wind is rejected explicitly.

## CLI

```bash
uv run --project "$AP_PNC_DIR" ap-pnc --help
```

| Group | Purpose |
|---|---|
| `gen-nmpc-lib` | fetch and build acados on first use, then generate the static NMPC solver bundle |
| `build` | colcon builds: `nmpc`, `planner`, `simple-sim`, `aero`, bindings |
| `benchmark` | offline plan → track → metrics pipeline |
| `docker` | container simulation: `build`, `up`, `down`, `config`, `dev` |
| `stack` | full bootstrap and the local ROS2 stack lifecycle |

## Repository layout

```
core/                     C++/ROS research code
  ros_packages/planner/     planner core (pure C++, pybind) + ROS node
  ros_packages/nmpc/        NMPC controller + solver wrapper
  ros_packages/interface/   the message contract
  bringup/                  planning.yaml, nmpc.yaml, launch files
  docker/                   multiarch ROS2 base image
infra/
  sim_infra/simple_sim/     offline stack: C++ simulator + Rust dora nodes
  sim_infra/aerodynamics/   aero models (C++ package + ML service)
  sim_infra/gazebo/         Gazebo SITL stack (roadmap)
  nav_infra/                px4ctrl / FAST-LIO / mavros (roadmap)
ap_pnc/                   Python: the ap-pnc CLI, acados codegen, paper layer
test/                     pytest
```

## Roadmap

Implemented now: the planner, the NMPC, the offline dora/Rerun stack, the
aerodynamics models and the CLI above.

Not yet part of the supported path, and not validated by the quick start:

- **Gazebo SITL** — stock PX4 v1.17 under Gazebo Harmonic with mavros2, run as a
  container stack (`infra/sim_infra/gazebo/`). Recipes exist; the production PX4
  rebuild and native AMD64 acceptance are still pending.
- **QGroundControl and the container desktop** — a noVNC desktop image for QGC
  and Gazebo, not required for any of the commands above.
- **Real hardware** — `nav_infra` carries the px4ctrl tracking/INDI controller,
  mavros profiles, Livox drivers and FAST-LIO. No hardware or flight acceptance
  is claimed.
- **K3s orchestration** — ConfigMap-driven simulation switching. It runs
  against cri-dockerd and assumes the simulation images you built yourself, so it
  adds scheduling on top of the same containers instead of a second runtime.
- **Web visualization and benchmark sweeps** — browser views of trajectories and
  tracking error, and NMPC-vs-SE3 aggregation across runs.
