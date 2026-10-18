---
name: ap-pnc-bootstrap
description: Build the public AP-PnC checkout and run its offline ideal-profile benchmark through Docker. Use for fresh-clone setup, solver regeneration, architecture-tagged static libraries, and checking what is available without local/private data.
---

# Public checkout: build and benchmark

Read `$AP_PNC_DIR/AGENTS.md` and `$AP_PNC_DIR/README.md` first. The canonical
repository is `SYSU-HILAB/AP-PnC` (`official/main`); do not synchronize the retired
`origin` fork unless asked. Treat tracked code and CLI help as authoritative when
older documentation disagrees. Local images, datasets and saved results are not
fresh-clone prerequisites or evidence of public reproducibility.

## Fresh-clone path

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and
[Docker](https://docs.docker.com/get-started/get-docker/), then use an absolute
checkout root:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv sync --project "$AP_PNC_DIR"
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
uv run --project "$AP_PNC_DIR" ap-pnc benchmark --profile ideal --aero lyu --vmax 8
```

`uv` orchestrates the Docker jobs. The benchmark runs C++ simulation, Rust export
and independent Rerun verification sequentially. It is not a ROS flight setup or
live viewer. Explicitly select `ideal`: the benchmark CLI otherwise defaults to
`practical`, whose authorized numeric profile/provenance are not shipped.
Do not fetch private upstreams, fabricate a practical profile or silently change
profiles. The legacy `benchmark run` subcommand is an in-process Python path,
not this Docker benchmark; do not substitute it for the command above.

## Solver targets

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch aarch64
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib --arch x86_64
```

These select Docker `linux/arm64` and `linux/amd64`, respectively, including
explicit cross-architecture builds when Docker can execute the target platform.
The default selects the host CPU, not a host-OS compilation path. Outputs are
`$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_<solver_arch>.a`; there is no
untagged alias. Caches use `$AP_PNC_DIR/.artifacts/nmpc_build/<solver_arch>/`.
The toolchain pin lives in `$AP_PNC_DIR/tooling/acados_setup.py`; build recipes
live under `$AP_PNC_DIR/core/docker/`. Regenerate and rebuild statically linked
consumers after solver changes. Do not invent `--arch` for `build simple-sim`:
that command currently selects the host CPU and has no architecture option.

## Results and limits

- Summary: `$AP_PNC_DIR/.artifacts/benchmark/stack-sweep/`.
- Runs: `$AP_PNC_DIR/.artifacts/simple-stack/linux-<docker_arch>/runs/`.
- Inspect saved configs, reference, CSV, manifest, metrics and verification
  receipts. A successful export does not prove good tracking or flight safety.
- The NMPC model currently supports zero wind. Report observed tracking errors
  and speed peaks, not hard feasibility guarantees from soft planner limits.
- `dev-sim` is separate and conditional on local practical data; use the
  `ap-pnc-dev-sim` skill. Gazebo/nav/hardware are not fresh-clone acceptance.

For implementation details read `$AP_PNC_DIR/tooling/simple_stack.py` and
`$AP_PNC_DIR/tooling/paper/benchmark/stack_sweep.py`. The pipeline guide is
`$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/README.md`; verify its older helper
examples against the current code before using them.
