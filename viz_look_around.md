# Visualization Lookup Notes

- feature/lu_mpc/controller/trajectory_loader.py — Parses planner CSV outputs using the new quaternion and alpha column names (`df_phi_quat_*`, `df_lyu_quat_*`, `df_phi_alpha`, `df_lyu_alpha`) and raises immediately if any required field is missing before downstream visualization consumes it.
- feature/lu_mpc/ros/reference_visualizer.py — Consumes `df_phi_quat` and `df_lyu_quat` states when publishing the RViz reference paths/odometry and now enforces that the df_lyu quaternion fields exist.
- feature/lu_mpc/plotting/trajectory_plots.py — Offline visualization helper that reads the new alpha columns to plot DF phi vs DF Lyu attack commands over time.
- feature/lu_mpc/ros/config/nmpc_viz.rviz — Placeholder RViz config retained for tooling compatibility but intentionally left empty.
- tests/trajectory_server/user_test_viz_alpha_time_series.py — CLI helper that plots `df_phi_alpha`/`df_lyu_alpha` individually over time plus their delta, useful for sanity-checking new planner outputs.
