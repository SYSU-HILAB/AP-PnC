# NMPC short-horizon reference, built on the NMPC grid (see nmpc.yaml):
#   N = round(horizon_s / traj_res_s);  points.size() == N + 1
#   point k is evaluated exactly at t0 + k * traj_resolution
builtin_interfaces/Time timestamp
float64 traj_resolution
bool tracking_valid
TrajectoryPoint[] points
