# Artifacts Layout

All generated results now live under `artifacts/` so they stay organized and self-describing. This tree is safe to wipe, but keep the structure when sharing artifacts or creating automation targets.

## Structure
- `benchmarks/ulgs/`: Logged `.ulg` files and CSV exports for timing/benchmark analysis. `scripts/tools/sulg`, `csv_trans`, and `csv_plots` read/write here, and Docker mounts this folder for QGroundControl captures.
- `simulations/temp_data/`: Intermediate planning CSV files and simulation dumps (e.g., `planning_*.csv`, `routes_*.csv`, `sim_results.csv`). Generators such as `run_sim_viz.py`, the ROS trajectory server, and `scripts/tools/plan_traj_vis.py` deposit/read data here.
- `visualization/figs/`: Default destination for published visual assets.
- `visualization/test_figs/`: Scratch space for QA plots or experiment-specific renders; point ad-hoc scripts here instead of cluttering the repo root.
- `tests/results/`: Placeholder for any persisted test outputs or coverage reports that must be shared; keep per-suite subfolders if needed.

Each directory contains a `.gitkeep` (or versioned data) so the hierarchy remains visible even when empty.
