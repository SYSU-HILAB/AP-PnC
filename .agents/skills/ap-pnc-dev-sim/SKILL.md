---
name: ap-pnc-dev-sim
description: Work on AP-PnC's conditional ROS2 pillar/LiDAR development simulator, including pause/step/reset and experimental point-cloud/map-corridor options. Use only after checking the required local practical actuator profile; it is not the fresh-clone benchmark or a flight setup.
---

# ROS2 development scene

Read `$AP_PNC_DIR/AGENTS.md`, `$AP_PNC_DIR/tooling/dev_sim.py` and
`$AP_PNC_DIR/tooling/commands/dev_sim.py`. The sensor guide is
`$AP_PNC_DIR/infra/DEV_SIM.md`; confirm experimental behavior
against current code, not historical one-off results in that guide.

## Prerequisites: stop if missing

The public source includes the simulator, but does not include the practical
actuator data needed to run it. Require both:

- `$AP_PNC_DIR/.artifacts/vtol-matlab/profile.yaml`
- `$AP_PNC_DIR/.artifacts/vtol-matlab/provenance.json`

These come from an authorized local extraction, not `uv sync` or a public
sample. The import helper expects a separate private upstream; do not fetch it,
publish its contents or invent calibration values. If inputs are unavailable,
report the blocker and offer the explicit `--profile ideal` offline benchmark
from the bootstrap skill. Do not claim an ideal dev-sim mode exists.

## Conditional workflow

Once the authorized inputs are present and `AP_PNC_DIR` is absolute:

```bash
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
uv run --project "$AP_PNC_DIR" ap-pnc build simple-sim
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim build
JOB=$(uv run --project "$AP_PNC_DIR" ap-pnc dev-sim prepare)
uv run --project "$AP_PNC_DIR" ap-pnc dev-sim run "$JOB"
```

`build` uses the host CPU's Linux Docker target; no `--arch` option exists here.
`prepare` stages frozen practical + Lyu configs and prints an absolute job path.
Defaults are NMPC, pillars, `box_sfc` and `--v-max 4`; `--controller se3` is an
explicit alternative. Jobs live under
`$AP_PNC_DIR/.artifacts/dev-sim/linux-<docker_arch>/`.

The simulation starts paused. Read this job's `dev-sim.json` for its exact
`container_name`; never assume a global container name. Inside that container,
use `/entrypoint.sh ros2 service call /simple_sim/<operation>
std_srvs/srv/Trigger '{}'`, with `pause`, `resume`, `step` or `reset`.
Step requires pause; reset returns to paused time zero with frozen inputs.
Restart/reprepare to change configs. Ctrl-C stops only this job's container.

## Boundaries and checks

- Runtime uses `--network none`, host IPC, ROS domain 213 and localhost-only DDS.
  No PX4, MAVROS or aircraft command dispatch is involved.
- Check `/clock`, committed odometry, `/lidar_sim/map`, `/lidar_sim/points`,
  frames/stamps and pause/reset behavior. Use the isolated integration probe
  `$AP_PNC_DIR/test/integration/check_dev_sim.py` with absolute config/output
  paths inside the development image. It is not a flight or realism certificate.
- Default CLI operation is headless; do not promise a Mac GUI or live web viewer.
- Sensor geometry does not add plant collision/contact forces.
- `--scene garden-3dgs` additionally requires the externally obtained and
  converted `$AP_PNC_DIR/.artifacts/dev-sim/scenes/garden_points.ply`. It is not
  bundled. Check permission/license before acquisition; do not download it by
  default. Run conversion/numerical tools in Docker, not host Python.
- `--corridor-source map_sfc` is experimental; do not present it or a historical
  garden run as verified collision avoidance, tuned tracking or a public CI test.
- Keep original configs, reference, CSV, manifests and metrics; visualization
  alone is not evidence of scientific correctness or acceptable tracking.
