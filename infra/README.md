# Infrastructure

Simulation environments and deployment support for the [core](../core/README.md)
planner and controller. Start with the offline benchmark; interactive scenes and
PX4 integration have additional prerequisites.

## Choose an environment

| Environment | Use it for | Prerequisites and limits |
| --- | --- | --- |
| [Offline simple-sim](SIMPLE_SIM.md) | Lockstep trajectory tracking and controller comparisons. | The `ideal` actuator profile works without private data; no PX4 or ROS graph is needed for offline runs. |
| [ROS2 development scene](DEV_SIM.md) | Interactive pillar/LiDAR simulation with pause, step and reset. | Requires a locally extracted practical actuator profile. |
| [Live 3DGS demo](CITY_DEMO_START.md) | Closed-loop flight in a rendered city scene. | Requires licensed local scene assets and the practical actuator profile. |
| [Gazebo / PX4](GAZEBO.md) | SITL integration with the navigation sidecar and core. | Read its validation status and build restrictions before use; complete flight acceptance is not established. |

Follow the [repository quick start](../README.md#quick-start) to generate the
solver, build simple-sim and run an ideal-profile benchmark. The
[simulation quick start](QUICK_START.md) also covers interactive scenes.

All infrastructure guides live here at the `infra/` root. The
[offline pipeline guide](OFFLINE_STACK.md) covers export and verified replay;
[city-demo acceptance](CITY_DEMO.md) and [media attribution](CITY_DEMO_MEDIA.md)
record the demo's evidence and licensing boundaries.

## Layout

- [sim_infra](sim_infra/) — simulator implementations, plant models and Docker stacks.
  The [aerodynamics guide](AERODYNAMICS.md) covers aerodynamic models.
- [sim_k3s](sim_k3s/) — Kubernetes manifests and dispatcher. Its
  [deployment notes](K3S.md) include legacy CLI examples; the current
  `ap-pnc` CLI does not register a `k3s` command.
- [observability](observability/) — local telemetry Collector configuration and
  verification helper. Remote telemetry export requires explicit authorization.

PX4 integration also uses the local `nav_infra` sidecar for MAVROS and px4ctrl;
its sources are not included in the public checkout. See the Gazebo guide for
integration requirements and the verified scope.

## Runtime boundaries

Use an absolute `AP_PNC_DIR` for paths and keep generated outputs beneath
`$AP_PNC_DIR/.artifacts/`. Linux Docker uses the host CPU architecture by default;
keep images, solver archives and compiled outputs on the same target.
ROS2 services using FastDDS share host IPC and need consistent discovery/network
settings. Do not run synthetic control probes on an aircraft or production
control network.

Runtime planning and control configuration belongs to
[core/bringup/config](../core/bringup/config/), not simulator-specific copies.
Use [tooling](../tooling/README.md) for supported build and test entry points.
