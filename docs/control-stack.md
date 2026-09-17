# Control Stack & Module-Level Documentation

This document summarizes how the AP-PnC repository hangs together at runtime. It covers the main ROS 2 nodes, helper libraries, generated code, and the ROS topics that tie everything into a closed-loop autonomous flight stack.

## Layout

Core ROS 2 C++ packages live under `ros_packages/`:

```
ros_packages/
├── planner/                  # trajectory generation line
│   ├── tailsitter_df/        #   DF math kernel (autodiff phi-theory)
│   ├── basic_trajectories/   #   geometric seeds (circle, lemniscate, sin, line)
│   ├── trajectory_server/    #   planning server node
│   └── traj_buffer/          #   reference conditioning + bullet republisher
├── c_nmpc/                   # NMPC tracking node + nmpc_wrapper (bundle C API)
├── px4_interface/            # PX4 bridge node
├── interface_msgs/           # ros2_msgs + px4_msgs (submodule)
└── utils/                    # ROS2-Utils + external libraries (subtrees)
```

Aerodynamic ground-truth models live in `simulations/aerodynamics` (shared
by gazebo and simple_sim); the acados codegen toolchain lives in
`ap_pnc/nmpc` (invoked by `uv run ap-pnc gen-nmpc-lib`).

## Aero semantics

* **Prior-free mainline**: phi-theory flatness — `PhiAerodynamics`
  (`F = Φ v |v|`) plus the calibrated `phi[8]`/`mass` parameters in
  `bringup/config/planning.yaml` (`flatness` section). `tailsitter_df`
  provides the differentially-flat map and autodiff Jacobians on top of it.
* **Prior-based experiment path**: `lyu_alpha` (AeroAlphaSolver force
  balance + LyuAerodynamics data-fit model, configured by
  `benchmark/config/aero_alpha_solver.yaml`). Not part of the online
  prior-free pipeline.

## End-to-End Flow

1. **Global trajectory generation (`ros_packages/planner/trajectory_server`)** samples a basic curve (circle, lemniscate, line, etc.), builds safe flight corridors with GCOPTER, and publishes `interface/RefTrajectory` messages on `control_traj` along with visualization markers on `traj_viz`.
2. **Trajectory buffering (`ros_packages/planner/traj_buffer`)** listens to new `RefTrajectory`s plus mode/ready flags. It conditions the path (offset removal, thrust initialization, down-sampling) and continually publishes fixed-size `RefTrajectoryBullet` packets (`ref_trajectory_bullet`) that always cover one second of future motion. It also mirrors the path to RViz via `ref_vis`.
3. **State estimation & PX4 bridge (`ros_packages/px4_interface`)** receives raw PX4 `VehicleOdometry` and `VehicleStatus`, applies frame transforms, and republishes `interface/State` on `/cur_state` (and `/cur_state_odom` for visualization). The same node subscribes to `/mpc_controls`, converts FLU controls/rates into PX4 FRD commands, and streams them to `/fmu/in/vehicle_thrust_acc_setpoint`. Heartbeat and tracking info are also bridged to PX4 via `/fmu/in/offboard_control_mode` and `/fmu/in/tracking_info`.
4. **NMPC runtime (`ros_packages/c_nmpc`)** combines the latest `/cur_state`, `/ref_trajectory_bullet`, and `/fmu/out/vehicle_acceleration`. `stateCallback()` updates the acados NLP bounds and publishes `/ref_can_set` so the buffer knows the solver is live. `referenceCallback()` loads the whole reference horizon into acados, injects aerodynamic parameters, triggers `solve()`, and publishes `/mpc_controls` plus `/tracking_info`.
5. **Differential-flatness & aerodynamics layers** (`ros_packages/planner/tailsitter_df`, `simulations/aerodynamics`, `ros_packages/utils`) supply the planner and NMPC with aerodynamic coefficients, analytic derivatives, and config readers. Python tooling in `ap_pnc/nmpc` regenerates acados models and JSON artifacts when weights or dynamics change.

## Module Details

### ROS messages (`ros_packages/interface_msgs/ros2_msgs`)
* Defines the lingua franca between nodes.
* `State.msg` stores `[p, v, body rates, quaternion]` plus `cur_cz` (filtered lift coefficient estimate).
* `TrajectoryPoint.msg` packs pose, velocity, rates, thrust, and body-axis targets into a fixed-length array so the planner, the buffer, and the solver agree on indices.
* `RefTrajectory.msg` carries an arbitrarily long sequence, while `RefTrajectoryBullet.msg` fixes the array size (`SIZE=50`) for constant-time MPC loading.
* `Controls.msg` is the NMPC output that PX4 consumes.
* `TrackingInfo.msg` is used for logging, PX4 HUD feedback, and `px4_interface` bridging.

### Global planner (`ros_packages/planner/trajectory_server`)
* Node name `global_planning_node` (`GlobalPlanner` class).
* Depends on GCOPTER (`include/gcopter`) and `planner/basic_trajectories` helpers (`Circle`, `Lemniscate`, `Sin`, `Line`, `VerticalLoop`).
* Configuration comes from `bringup/config/planning.yaml` (PlannerCfg with nested CostCfg, MincoCfg, FlatnessCfg) — no ROS parameters.
* Publishes:
  * `control_traj` (`interface/RefTrajectory`) – consumed by `traj_buffer`.
  * `traj_viz` (`geometry_msgs/PoseArray`) – optional visualization.

### Trajectory buffer (`ros_packages/planner/traj_buffer`)
* Cuts the planner output into one-second `RefTrajectoryBullet` windows and keeps re-publishing them so the NMPC always has a live horizon.
* Also performs offset removal, thrust initialization, and down-sampling.

### NMPC node (`ros_packages/c_nmpc`)
* Wraps the self-contained acados solver archive `.artifacts/nmpc_wrapper/libnmpc_bundle.a` through `nmpc_wrapper` (C API: `nmpc_solver_create/solve/free`).
* Config from `bringup/config/c_nmpc.yaml` (`nmpc: {mass, cz_rate}`).
* The solver itself is generated by `ap_pnc/nmpc/create_ocp.py` via `uv run ap-pnc gen-nmpc-lib`.

### Differential-flatness kernel (`ros_packages/planner/tailsitter_df`)
* `tailsitter_df.cpp` implements the flatness map with autodiff. Model parameters (`phi` matrix, mass) arrive via `Model::setParameters` at node startup from the `flatness` section of `bringup/config/planning.yaml`, and exports helpers such as `df_forward`, `df_backward`, and aerodynamic Jacobians.
