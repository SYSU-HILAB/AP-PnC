**TDD Assessment**

**Why.** Feature modules host mission-critical control/sim code, yet tight coupling to ACADOS/ROS and manual scripts prevents Red-Green-Refactor, hiding regressions until late and eroding confidence.

**What (key gaps).**
- **Controller (untested).** `ProposedNmpcController.compute_control()` (`feature/lu_mpc/controller/ctrl_proposed_nmpc.py:49`) runs without any pytest coverage and immediately instantiates the real `AcadosOcpSolver` (`feature/lu_mpc/controller/ctrl_proposed_nmpc.py:53-124`), making a Red step impossible.
- **Trajectories (unverified).** `load_trajectory_csv` and `TrajectoryReference` (`feature/lu_mpc/controller/trajectory_loader.py:62-215`) validate planner CSVs via printouts, but no tests exercise `_find_planning_csv`, error messaging, or quaternion normalization; ROS nodes discover failures only at runtime.
- **Simulator (untested).** The GD simulator (`feature/lu_mpc/model/sim/sim_gd_model.py:1-210`) still lacks pytest coverage or any supported entry point now that the ad-hoc plotting script was removed, so regressions in the GD model remain invisible.
- **ROS (mixed layers).** `ReferenceVisualizer` blends ROS plumbing with playback math (`feature/lu_mpc/ros/reference_visualizer.py:1-150`); automated coverage is limited to `advance_playback_time()` while other “user_test_*” scripts remain manual entry points (`tests/lu_mpc/ros/user_test_ref_traj_viz.py:1-120`).

**How (next steps).**
1. **Decouple.** Inject solvers, trajectory providers, and publishers so controllers/simulators can be instantiated in pytest without ACADOS/ROS stacks.
2. **Test data.** Add fixtures for planner CSVs and GD states to exercise validation errors, extrapolation edges, aerodynamic outputs, and parameter sweeps.
3. **Rehome scripts.** Move “user_test” launchers to `scripts/tools` and replace them with real unit/integration suites under `tests/lu_mpc`, covering ROS message/QoS helpers.
