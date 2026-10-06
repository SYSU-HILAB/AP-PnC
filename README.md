# AP-PnC

Aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV.

> **Aerodynamic Prior-Free Coordinated Trajectory Generation and Tracking Control
> for a Tail-Sitter UAV**
> Erchao Rong, Zihao Liu, Junning Liang, Jianguo Wang, Xiao Jie, Haoran Fu,
> Ziliang Chen, Ximin Lyu — arXiv:2609.11698 · <https://arxiv.org/abs/2609.11698>

<details>
<summary>BibTeX</summary>

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

</details>

## What this is

Two phase-specific models, as in the paper:

**Planning** uses the phi-theory flatness under coordinated flight, parameterised
by a single flat-plate drag coefficient. **Tracking** uses an NMPC over a
simplified model whose single longitudinal aerodynamic parameter is estimated
online — no airframe-specific aerodynamic identification campaign is needed.

Implemented and exercised by the quick start below: the planner, the NMPC, the
aerodynamics models, the offline simulator and the `ap-pnc` CLI. Everything else
is listed under [Roadmap](#roadmap).

## Quick start

Needs **`uv`**, **`git`** and **Docker**. No ROS, no Gazebo, no flight hardware,
no private data. On macOS this runs a native `linux/arm64` container rather than
emulating AMD64, and the acados toolchain is built inside Docker too.

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
git clone https://github.com/WarriorHanamy/AP-PnC.git "$AP_PNC_DIR"

# 1. Python environment + the self-contained acados solver bundle.
#    On Linux this builds locally; on macOS it builds in the pinned container.
uv sync --project "$AP_PNC_DIR" --group dev
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib

# 2. Build the offline stack: Rust dora nodes + the C++ simulator.
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" build

# 3. Plan, simulate and record one run. Prints the JOB path.
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" run 0 ideal none 6 0
#    run [duration_s | 0 = whole reference]
#        [ideal | matlab-vtol]
#        [aero model | configured]
#        [v_max | configured]
#        [terminal hold s]

# 4. Re-decode that job independently and run the acceptance checks.
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test "$JOB"
```

### What you get

Step 3 prints `JOB=/…/runs/<timestamp>-<pid>`. That directory is the result:

| File | Content |
|---|---|
| `reference.yaml` | the planner's trajectory, as persisted polynomials |
| `steps.csv` | per-tick plant state, applied wrench, aerodynamic force, command |
| `nmpc_reference.csv` | the exact reference the solver was given, per horizon stage |
| `manifest.json` | status, config hashes, build revision, validation labels |
| `metrics.json` | RMSE and max error over the samples the run actually committed |
| `replay.rrd` + `verified.json` | the Rerun recording and its verification receipt |

Rerun is **offline only**. The scientific authority is the CSV/YAML/metrics;
viewing or exporting the recording never re-plans and never re-simulates.

### Two plant profiles

| Profile | Plant | Needs |
|---|---|---|
| `ideal` | rate-loop plant | nothing extra — this is what the quick start uses |
| `matlab-vtol` | four-motor plant with mixer, ESC, PID and propeller model | numeric parameters pre-extracted into `.artifacts/vtol-matlab/` |

### Figures

Plots re-read saved runs; they never re-simulate.

```bash
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" plot \
  --output "$AP_PNC_DIR/.artifacts/benchmark/MY_GROUP"
```

## Configuration

Every run snapshots its configuration into the job directory, so any result can
be traced back to the exact settings that produced it. These are the sources of
truth — read the files, not a table that goes stale:

| File | Owns |
|---|---|
| `core/bringup/config/planning.yaml` | corridor, seed curve, solver cost weights and limits, flatness parameters |
| `core/bringup/config/nmpc.yaml` | horizon and node spacing, objective weights, command bounds, estimator settings |
| `core/bringup/config/simple_sim.yaml` | plant mass and inertia, actuator model, aerodynamics model and wind |

**Before editing them, know two things:**

- **Weights, bounds and the grid are compiled into the solver.** Change them, then
  re-run `ap-pnc gen-nmpc-lib` and relink. A stale bundle is rejected by an
  explicit ABI check rather than silently producing wrong numbers.
- **The prediction uses ground speed**, so non-zero wind is rejected explicitly
  instead of being quietly mis-modelled.

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

`uv` is the only Python entry point, and the CLI resolves every path it needs
from `AP_PNC_DIR`.

## Repository layout

```
core/                      C++/ROS research code
  ros_packages/planner/      planner core (pure C++, pybind) + ROS node
  ros_packages/nmpc/         NMPC controller + solver wrapper
  ros_packages/interface/    the message contract
  bringup/                   planning.yaml, nmpc.yaml, launch files
  docker/                    multiarch ROS2 base image
infra/
  sim_infra/simple_sim/      offline stack: C++ simulator + Rust dora nodes
  sim_infra/aerodynamics/    aerodynamic models (C++ package + ML service)
  sim_infra/gazebo/          Gazebo SITL stack — roadmap
  nav_infra/                 px4ctrl / FAST-LIO / mavros — roadmap
tooling/                   Python: the ap-pnc CLI, acados codegen, paper layer
test/                      pytest
```

## Roadmap

None of these are needed by the quick start, and none of them are validated by it.

| Area | State | Note |
|---|---|---|
| Gazebo SITL | recipes, not accepted | stock PX4 v1.17 under Gazebo Harmonic with mavros2. The production PX4 rebuild and native AMD64 acceptance are pending. |
| QGroundControl, container desktop | optional | a noVNC desktop image; not required by any command above. |
| Real hardware | code present, unvalidated | `nav_infra` carries the px4ctrl tracking/INDI controller, mavros profiles, Livox drivers and FAST-LIO. No hardware or flight acceptance is claimed. |
| K3s orchestration | planned | ConfigMap-driven switching, running against cri-dockerd and the images you built yourself — scheduling over the same containers, not a second runtime. |
| Web reports and sweep aggregation | planned | browser views of trajectories and tracking error, and NMPC-vs-SE3 comparison across runs. |
