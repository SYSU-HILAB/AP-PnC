---
name: ap-pnc-gazebo-readiness
description: Assess the experimental Gazebo/PX4 recipes and their external dependencies before build or deployment. Use for Gazebo, MAVROS, nav_infra or flight-readiness questions; the public checkout does not provide an accepted end-to-end flight runbook.
---

# Gazebo readiness, not a flight runbook

The public quick start is the offline ideal-profile benchmark. Gazebo Docker
recipes and an internal Compose API remain in this repository, but their
presence does not make a fresh clone a complete, accepted flight stack.
Do not treat historical local arming/takeoff/handover tests as a public flight
procedure.

## Inspect before acting

Read `$AP_PNC_DIR/AGENTS.md` and these tracked sources:

- `$AP_PNC_DIR/tooling/docker_runtime.py`
- `$AP_PNC_DIR/infra/sim_infra/gazebo/docker/docker-compose.yml`
- `$AP_PNC_DIR/infra/sim_infra/gazebo/scripts/verify_runtime.py`

The Compose nav service points to
`$AP_PNC_DIR/infra/nav_infra/docker/Dockerfile`. That external repository is not
tracked in AP-PnC. A local checkout or cached image does not establish its
availability to public users. Check dependency availability, revisions and
permission; report missing inputs instead of cloning private sources or
launching an incomplete stack. `ap-pnc docker` is not a CLI group.

`$AP_PNC_DIR/infra/GAZEBO.md` contains historical experiments
and stale examples. Verify each claim against current tracked code. There is
no tracked multiarch CI workflow to cite as acceptance evidence.

## Safe scope

- Do not run unqualified internal `build()`/`up()` as a bootstrap step: they
  select the whole experimental stack, including large PX4 builds.
- Honor the build/resource restrictions in AGENTS.md. Do not retry deferred
  production PX4 work or prune unrelated images to make it fit.
- Only inspect/render Compose unless deployment is explicitly requested and
  the external prerequisites are confirmed. Build/run platforms and tagged
  solver archives must agree; ARM64 and AMD64 recipes need separate acceptance.
- Preserve host IPC and consistent discovery for FastDDS SHM. A healthy
  container or UDP-only workaround does not prove ROS message delivery.
- Keep GUI/noVNC loopback-only. Do not connect synthetic ROS probes to aircraft
  networks or issue arming, takeoff, OFFBOARD or handover commands from a skill.
- The tracked runtime probe is read-only and expects an already running stack;
  even a successful probe does not prove flight-rate performance or flight
  acceptance. Save new evidence under `$AP_PNC_DIR/.artifacts/` and state its
  exact scope, including whether execution was native or emulated.

For public core builds use `ap-pnc-docker`; for conditional isolated sensor
work use `ap-pnc-dev-sim`. Full production startup and flight acceptance must
not be inferred from either workflow.
