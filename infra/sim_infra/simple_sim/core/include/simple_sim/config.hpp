#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace simple_sim
{

  struct PlantConfig
  {
    double          mass    = 2.0;
    double          gravity = 9.81;
    Eigen::Vector3d inertia =
        Eigen::Vector3d(0.01715066, 0.01818037, 0.01987597);
    void validate() const;
  };

  struct RateLoopConfig
  {
    // First-order inner loop. angular rate channels: gain = 1/tau [1/s]; the
    // collective specific force: tau = thrust_tau_s [s]. Together they are the
    // whole "ideal" actuation model, so a run on this profile isolates the
    // guidance and control law from the motor, mixer and PID chain.
    Eigen::Vector3d gain       = Eigen::Vector3d(100.0, 100.0, 100.0);  // 1/s
    double          thrust_tau_s = 0.1;                                 // s
    Eigen::Vector3d max_rate   = Eigen::Vector3d(8.0, 8.0, 8.0);        // rad/s
    Eigen::Vector3d max_moment = Eigen::Vector3d(2.0, 2.0, 1.0);        // N m
    double          max_specific_force = 30.0;                          // m/s^2
    void            validate() const;
  };

  struct RunnerConfig
  {
    std::int64_t physics_dt_ns = 1000000;
    std::int64_t control_dt_ns = 20000000;
    std::int64_t steps         = 1;
    void         validate() const;
  };

}  // namespace simple_sim
