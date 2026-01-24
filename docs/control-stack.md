# Control Stack & Module-Level Documentation

This document summarizes how the AP-PnC repository hangs together at runtime. It covers the main ROS 2 nodes, helper libraries, generated code, and the ROS topics that tie everything into a closed-loop autonomous flight stack.

## End-to-End Flow

1. **Global trajectory generation (`src/trajectory_server`)** samples a basic curve (circle, lemniscate, line, etc.), builds safe flight corridors with GCOPTER, and publishes `interface/RefTrajectory` messages on `control_traj` along with visualization markers on `traj_viz`.
2. **Trajectory buffering (`src/traj_buffer`)** listens to new `RefTrajectory`s plus mode/ready flags. It conditions the path (offset removal, thrust initialization, down-sampling) and continually publishes fixed-size `RefTrajectoryBullet` packets (`ref_trajectory_bullet`) that always cover one second of future motion. It also mirrors the path to RViz via `ref_vis`.
3. **State estimation & PX4 bridge (`src/px4_interface`)** receives raw PX4 `VehicleOdometry` and `VehicleStatus`, applies frame transforms, and republishes `interface/State` on `/cur_state` (and `/cur_state_odom` for visualization). The same node subscribes to `/mpc_controls`, converts FLU controls/rates into PX4 FRD commands, and streams them to `/fmu/in/vehicle_thrust_acc_setpoint`. Heartbeat and tracking info are also bridged to PX4 via `/fmu/in/offboard_control_mode` and `/fmu/in/tracking_info`.
4. **NMPC runtime (`src/c_nmpc`)** combines the latest `/cur_state`, `/ref_trajectory_bullet`, and `/fmu/out/vehicle_acceleration`. `stateCallback()` updates the acados NLP bounds and publishes `/ref_can_set` so the buffer knows the solver is live. `referenceCallback()` loads the whole reference horizon into acados, injects aerodynamic parameters, triggers `solve()`, and publishes `/mpc_controls` plus `/tracking_info`.
5. **Additional sensors/utilities** such as `src/mocap-px4/src/frame_transform` (converts `/mocap/odometry` into PX4 visual odometry).
6. **Differential-flatness & aerodynamics layers** (`src/tailsitter_df`, `src/aerodynamics`, `src/utils`) supply the planner and NMPC with aerodynamic coefficients, analytic derivatives, and config readers. Python tooling in `src/c_nmpc/nmpc` regenerates acados models and JSON artifacts when weights or dynamics change.

## Module Details

### ROS messages (`src/interface/msg`)
* Defines the lingua franca between nodes.
* `State.msg` stores `[p, v, body rates, quaternion]` plus `cur_cz` (filtered lift coefficient estimate).
* `TrajectoryPoint.msg` packs pose, velocity, rates, thrust, and body-axis targets into a fixed-length array so both GCOPTER, the buffer, and the solver agree on indices.
* `RefTrajectory.msg` carries an arbitrarily long sequence, while `RefTrajectoryBullet.msg` fixes the array size (`SIZE=50`) for constant-time MPC loading.
* `Controls.msg` is the NMPC output that PX4 consumes.
* `TrackingInfo.msg` is used for logging, PX4 HUD feedback, and `px4_interface` bridging.

### Global planner (`src/trajectory_server/src/global_planning.cpp`)
* Node name `global_planning_node` (`GlobalPlanner` class).
* Depends on GCOPTER (`src/external_libraries/gcopter`) and `src/basic_trajectories` helpers (`Circle`, `Lemniscate`, `Sin`, `Line`, `VerticalLoop`).
* Parameters (see `Config` struct) control box constraints, sampling resolution, and which base curve seeds the optimizer.
* Publishes:
  * `control_traj` (`interface/RefTrajectory`) – consumed by `traj_buffer`.
  * `traj_viz` (`geometry_msgs/PoseArray`) – optional visualization.
* Optionally subscribes to mission targets (placeholder `targetSub`), but the timer-based `plan()` is what drives periodic replanning.

### Trajectory buffer (`src/traj_buffer/src/traj_buffer.cpp`)
* Node name `trajectory_buffer`.
* Subscribes to:
  * `ref_can_set` (`std_msgs/Bool`) – tells it when the NMPC is ready to accept reference packets.
  * `mode` (`std_msgs/Bool`) – toggles between fixed hover points and streaming planner output.
  * `control_traj` (`interface/RefTrajectory`).
  * `cur_state` (`interface/State`) – currently only used to estimate aerodynamic coefficients.
* Publishes:
  * `ref_trajectory_bullet` (`interface/RefTrajectoryBullet`).
  * `ref_vis` (`visualization_msgs/MarkerArray`).
* Resets positions so every trajectory starts at the current pose, overwrites the thrust slice with hover thrust (`kThrustPerRotor`), and fills a rolling window of 50 points (with optional down-sampling). When the `circled` flag is set it loops; when not, it stops after the terminal pose and switches to fixed references.

### NMPC runtime (`src/c_nmpc/src/c_nmpc.cpp`)
* Node name `NMPC`.
* Wraps the generated acados solver (`c_generated_code/acados_solver_tailsitter_flu.*`) via `AcadosInterface`.
* Subscribes to `/cur_state`, `/ref_trajectory_bullet`, `/fmu/out/vehicle_acceleration`.
* Publishes `/mpc_controls`, `/ref_can_set`, and `/tracking_info`.
* The `stateCallback()` copies `[x,u,cz]` into `x_cur_`, updates both `lbx`/`ubx` for stage 0, estimates the velocity norm, and raises the `ref_can_set` flag.
* `referenceCallback()` packages the incoming bullet: `TrackingInfo` is populated with ref/actual values and derived angles of attack/sideslip (`alpha`, `beta`). Aerodynamic parameters (`cz`, `alpha`, `cz_rate`) are fed into acados via `setReference(..., p, i)`. After loading stages `0..N`, `solve()` runs, checks `status`, copies the predicted control `u_next`, and publishes `Controls` with solver latency.
* `accCallback()` filters PX4 accelerometer data to update `cz_filtered_`, enforcing a ±0.4 bound.
* Parameters `mass` and `cz_rate` come from ROS parameters (defaults in `global.toml` under `[c_nmpc]`).

### NMPC model builder (`src/c_nmpc/nmpc`)
* Python tooling used offline/CI to regenerate the solver artifacts.
* `model.py` defines `tailsitter_flu()` with augmented states `[p,v,r,q,u,cz_state]`, aerodynamic forces, and quaternion kinematics.
* `config.py` (`ControlConfig`, `ProposedMpcConfig`) centralizes horizon length, weight matrices, state bounds, and regularization terms.
* `create_ocp_or_sim.py` builds the `AcadosOcp`, sets cost expressions (`NONLINEAR_LS`), enforces actuator/state bounds, and emits JSON/c_generated code via `AcadosOcpSolver`.
* Run with `./pyp src/c_nmpc/nmpc/create_ocp_or_sim.py` after changing dynamics or weights; `c_generated_code/` is what the C++ node links against.

### PX4 bridge (`src/px4_interface/src/px4_interface.cpp`)
* Node name `px4_interface`.
* Publishes to PX4: `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_thrust_acc_setpoint`, `/fmu/in/tracking_info`, and `/fmu/in/vehicle_visual_odometry` (via the sibling mocap node).
* Publishes to the rest of the stack: `/cur_state` (`interface/State`) and `/cur_state_odom` (`nav_msgs/Odometry`).
* Subscribes to: `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`, `/mpc_controls`, `/tracking_info`, `/est_cmd` (estimation trigger).
* Implements a `Transcribe` helper that aligns PX4 FRD data with the internal FLU convention. A 100-sample bootstrap estimates initial position + yaw; only after `is_takeoff_complete` and the estimator is initialized does it publish state/accept controls.
* Converts `Controls.rates_sp` from FLU to FRD (`Transcribe::toFRD`) and republishes solver timing, thrust, and commanded pitch.

### Mocap bridge (`src/mocap-px4/src/frame_transform`)
* Provides two nodes:
  * `frame_transform` – library of quaternion/coordinate utilities.
  * `mocap_to_visual_odometry` – subscribes to `/mocap/odometry`, averages a window to remove bias, applies FLU→PX4 transforms, filters velocity with an alpha-beta filter, and publishes PX4 `VehicleOdometry` messages on `/fmu/in/vehicle_visual_odometry`.
* Tunable parameters: `time_unit`, `alpha`, `beta`, and bias window size.

### Differential-flatness & aerodynamics (`src/tailsitter_df`, `src/aerodynamics`, `src/utils`)
* `src/tailsitter_df/src/tailsitter_df.cpp` implements lift/drag modeling with autodiff. It loads aerodynamic matrices (`phi`) and mass from `global.toml`, computes body-frame bases `(xb,yb,zb)`, and exports helpers such as `df_forward`, `df_backward`, and aerodynamic Jacobians.
* `src/aerodynamics` contains various aerodynamic models (bspline fits, lookup tables) that can be swapped for benchmarking.
* `src/utils` provides config readers for spline/phi files (`phi_aero_config_reader.cpp`, `bspline_aero_config_reader.cpp`, etc.).

### External libraries (`src/external_libraries`)
* Includes GCOPTER, BSpline basis utilities, `tomlplusplus`, and `yamlcpp`. These are consumed by planner, utilities, and configuration readers.

## ROS Topic Reference

| Topic | Msg Type | Publishers | Subscribers | Purpose |
|-------|----------|------------|-------------|---------|
| `control_traj` | `interface/RefTrajectory` | `trajectory_server` | `traj_buffer` | Full-length optimized reference trajectory |
| `traj_viz` | `geometry_msgs/PoseArray` | `trajectory_server` | RViz | Visualize planned path |
| `ref_can_set` | `std_msgs/Bool` | `c_nmpc` | `traj_buffer` | Signals when NMPC is ready for new reference bullets |
| `mode` | `std_msgs/Bool` | ground station / user | `traj_buffer` | Switch between fixed-point vs planner references |
| `ref_trajectory_bullet` | `interface/RefTrajectoryBullet` | `traj_buffer` | `c_nmpc` | One-second window of references |
| `ref_vis` | `visualization_msgs/MarkerArray` | `traj_buffer` | RViz | Render buffered reference |
| `/cur_state` | `interface/State` | `px4_interface` | `c_nmpc`, `traj_buffer` | Current pose/velocity in FLU frame |
| `/cur_state_odom` | `nav_msgs/Odometry` | `px4_interface` | RViz | Visualization-only odometry |
| `/mpc_controls` | `interface/Controls` | `c_nmpc` | `px4_interface` | Solver output (per-rotor thrust & rates) |
| `/tracking_info` | `interface/TrackingInfo` | `c_nmpc` | `px4_interface` | Logging & PX4 telemetry overlay |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | `px4_interface` | Raw estimator pose from PX4 |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4 | `px4_interface` | Arm/mode state gating |
| `/fmu/out/vehicle_acceleration` | `px4_msgs/VehicleAcceleration` | PX4 | `c_nmpc` | Used to update aerodynamic `cz` filter |
| `/fmu/in/vehicle_thrust_acc_setpoint` | `px4_msgs/VehicleThrustAccSetpoint` | `px4_interface` | PX4 | Sends thrust/rate commands |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | `px4_interface` | PX4 | Heartbeat to keep PX4 in offboard mode |
| `/fmu/in/tracking_info` | `px4_msgs/TrackingInfo` | `px4_interface` | PX4 UI | Mirrors solver stats |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | `mocap_to_visual_odometry` | PX4 | Replaces onboard VIO with mocap |
| `/mocap/odometry` | `geometry_msgs/PoseStamped` | motion capture bridge | `mocap_to_visual_odometry` | Raw mocap pose |
| `mock_map` | `sensor_msgs/PointCloud2` | `mockamap` | planners/visualizers | Procedural obstacle cloud |
| `/est_cmd` | `std_msgs/Bool` | pilot/tooling | `px4_interface` | Indicates takeoff complete – enables control publishing |

## Calling Procedure Checklist

1. **Bring-up**
   * Launch `px4_interface` first so it can monitor `/fmu/out/vehicle_status`. It waits until PX4 is armed + in POSCTL and until `/est_cmd` is true before publishing `/cur_state`.
   * Start `mocap_to_visual_odometry` (if using mocap) so PX4 receives `/fmu/in/vehicle_visual_odometry`.

2. **Planning**
   * Launch `global_planning_node`. It continuously publishes `control_traj`. Parameters for curvature, radius, and safe corridor lengths live in `src/trajectory_server/config` and `global.toml`.

3. **Buffering**
   * Start `trajectory_buffer`. Once `NMPC` raises `/ref_can_set`, it converts the RefTrajectory stream into bullets and optionally locks onto the last point when the planner stops publishing.

4. **Control**
   * Launch `c_nmpc`. After it receives the first `/cur_state` it pushes `/ref_can_set=true`. Each bullet triggers `referenceCallback()`, which updates aerodynamic parameters, logs `TrackingInfo`, and solves acados.
   * `solve()` publishes `/mpc_controls`. The PX4 interface immediately forwards those commands.

5. **PX4 execution**
   * `px4_interface` keeps PX4 in offboard mode via the 10 Hz heartbeat, translates commands, and mirrors telemetry back to PX4 HUD through `/fmu/in/tracking_info`.

## Extending or Modifying the Stack

* **Changing dynamics/weights** – update `src/c_nmpc/nmpc/model.py` or `src/c_nmpc/nmpc/config.py`, re-run `./pyp src/c_nmpc/nmpc/create_ocp_or_sim.py`, then rebuild the ROS 2 workspace so `c_nmpc` links against the refreshed `c_generated_code`.
* **Adding new trajectories** – implement the shape inside `src/basic_trajectories` (follow `Circle` or `Lemniscate`), expose it in `GlobalPlanner::init_basic_trajectory_type`, and add parameters in `global.toml` or the relevant ROS 2 launch file.
* **Integrating new sensors** – publish your measurement on a fresh topic and tap into `px4_interface` or `trajectory_buffer` as needed. Both nodes rely on `interface/msg/*.msg`, so reuse or extend those definitions.
* **Tuning aerodynamic logic** – edit `global.toml` (mass, `phi`) and the filters inside `src/tailsitter_df` or `src/c_nmpc`. Re-run the NMPC builder after large changes so solver bounds stay consistent.
