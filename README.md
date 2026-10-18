# AP-PnC

Aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV.

## Quick start

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and
[Docker](https://docs.docker.com/get-started/get-docker/).
Set your checkout location, build, and run:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
git clone https://github.com/SYSU-HILAB/AP-PnC.git "$AP_PNC_DIR"
uv sync --project "$AP_PNC_DIR"
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
uv run --project "$AP_PNC_DIR" ap-pnc benchmark --profile ideal --aero lyu --vmax 8
```

`uv` orchestrates Docker builds and simulation jobs. The benchmark compares
SE3 control with and without aerodynamic feedforward against NMPC, saves the
results, and prints a comparison table.

Use `--aero none,lyu,phi --vmax 8,10,12` for a larger sweep.
The `ideal` profile is ready to use; `practical` requires locally extracted
actuator parameters.

### Solver targets

The default target is the host CPU architecture. Select either target explicitly
for cross-architecture builds:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch x86_64
```

Docker builds for `linux/arm64` or `linux/amd64` and writes
`.artifacts/nmpc_solver/libnmpc_bundle_<arch>.a`.
The Docker builder/runtime must support the selected platform.

## ROS2 development scene

The interactive simulator combines a point-cloud scene, pose-dependent LiDAR,
Lyu aerodynamics and practical actuation, with pause, step and reset controls.
It requires the local practical actuator profile.

```bash
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim build
JOB=$(uv run --project "$AP_PNC_DIR" ap-pnc dev-sim prepare)
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim run "$JOB"
```

See the [development simulator guide](infra/sim_infra/lidar_sim/README.md) for
profile preparation, ROS2 topics, RViz and experimental scene options.

## Results

All generated files stay under `.artifacts/`:

- **Comparison summaries:** `.artifacts/benchmark/stack-sweep/`
- **Individual runs:** `.artifacts/simple-stack/linux-<arch>/runs/`

Run outputs include configurations, trajectories, simulation data, metrics,
logs and verified Rerun recordings.

## Configuration and documentation

- [planning.yaml](core/bringup/config/planning.yaml) — trajectory and planning.
- [nmpc.yaml](core/bringup/config/nmpc.yaml) — controller and solver.
- [simple_sim.yaml](core/bringup/config/simple_sim.yaml) — plant and simulation.

After changing solver settings, regenerate the NMPC bundle and rebuild the stack.

- [Simulation guide](infra/sim_infra/simple_sim/README.md)
- [Planner and NMPC package guide](core/ros_packages/README.md)
- [Linux development containers](core/docker/README.md)

For command options, run `uv run --project "$AP_PNC_DIR" ap-pnc --help`.

## Paper

**Aerodynamic Prior-Free Coordinated Trajectory Generation and Tracking Control
for a Tail-Sitter UAV**

[arXiv:2609.11698](https://arxiv.org/abs/2609.11698) ·
[IEEE/ASME Transactions on Mechatronics, Early Access](https://ieeexplore.ieee.org/document/11719954)

<details>
<summary>Cite this work (BibTeX)</summary>

```bibtex
@article{rong2026apPnc,
  title  = {Aerodynamic Prior-Free Coordinated Trajectory Generation and
            Tracking Control for a Tail-Sitter UAV},
  author = {Rong, Erchao and Liu, Zihao and Liang, Junning and Wang, Jianguo and
            Jie, Xiao and Fu, Haoran and Chen, Ziliang and Lyu, Ximin},
  journal = {arXiv preprint arXiv:2609.11698},
  note = {IEEE/ASME Transactions on Mechatronics, Early Access},
  year   = {2026},
  url    = {https://arxiv.org/abs/2609.11698}
}
```

</details>
