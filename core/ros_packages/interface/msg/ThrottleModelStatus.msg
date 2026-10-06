# Online RLS throttle model status (ROS-FLU domain: u in [0,1], accel +z up).
uint64 timestamp
uint64 timestamp_sample
# Specific force per unit throttle [m/s^2 per throttle unit]
float32 eta
# Estimator variance of eta [(m/s^2)^2 per throttle unit^2]
float32 eta_variance
# a_meas - eta * u of last update [m/s^2]
float32 innovation
# innovation^2 / predicted variance [-]
float32 innovation_test_ratio
# True when variance has converged below the validity threshold
bool estimate_valid
# Last (filtered) throttle fed to the estimator [0,1]
float32 throttle_input
# Last (filtered) measured specific force on the thrust axis [m/s^2]
float32 measured_specific_force
