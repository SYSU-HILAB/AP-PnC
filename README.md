# AP-PnC

Aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV.

> **Work in progress:** Offline simulation is available. The complete usable
> release is delayed by approximately one month or more.

## Quick start

**Requirements:** Git, [uv](https://docs.astral.sh/uv/getting-started/installation/)
and Docker. This workflow does not need a host ROS or acados installation.
On Apple Silicon, it uses native Linux ARM64 containers.

### 1. Get the code

Replace `/absolute/path/to/AP-PnC` with your preferred clone location.

```bash
git clone https://github.com/SYSU-HILAB/AP-PnC.git /absolute/path/to/AP-PnC
cd /absolute/path/to/AP-PnC
```

### 2. Build

```bash
uv sync
uv run ap-pnc gen-nmpc-lib
uv run ap-pnc build simple-sim
```

The first build downloads and compiles the required dependencies in Docker.

### 3. Run an experiment

```bash
uv run ap-pnc benchmark --profile ideal --aero lyu --vmax 8
```

This compares SE3 control with and without aerodynamic feedforward against NMPC.
The CLI manages the simulations, saves the results, and prints a comparison table.

Use `--aero none,lyu,phi --vmax 8,10,12` for a larger sweep.
Keep `--profile ideal` for a fresh clone; `practical` needs locally extracted
actuator parameters.

## Results

All generated files stay under `.artifacts/`:

- **Comparison summaries:** `.artifacts/benchmark/stack-sweep/`
- **Individual runs:** `.artifacts/simple-stack/linux-<arch>/runs/`

Each run saves its configuration, trajectory, simulation data, metrics and logs.
Completed, verified runs also include an offline Rerun recording.
Check the reported status and tracking errors; a completed recording alone does
not establish good tracking performance.

## Configuration

- [planning.yaml](core/bringup/config/planning.yaml) — trajectory and planning settings.
- [nmpc.yaml](core/bringup/config/nmpc.yaml) — controller and solver settings.
- [simple_sim.yaml](core/bringup/config/simple_sim.yaml) — simulation and plant settings.

After changing solver settings, regenerate the NMPC bundle and rebuild the stack.
The current NMPC model supports zero wind only.

## Documentation

- [Simulation guide](infra/sim_infra/simple_sim/README.md)
- [Planner and NMPC package guide](core/ros_packages/README.md)
- [Linux development containers](core/docker/README.md)

For command options:

```bash
uv run ap-pnc --help
uv run ap-pnc benchmark --help
```

**Scope:** The quick start is an offline experiment, not a flight setup.
Complete flight and native AMD64 runtime acceptance remain pending.

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
