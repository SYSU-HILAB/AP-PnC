#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace simple_sim
{

  // World ENU; body FLU; attitude rotates body vectors into world.
  struct State
  {
    Eigen::Vector3d    position         = Eigen::Vector3d::Zero();
    Eigen::Vector3d    velocity         = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude         = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    angular_velocity = Eigen::Vector3d::Zero();
  };

  struct Wrench
  {
    Eigen::Vector3d force  = Eigen::Vector3d::Zero();  // body FLU, N
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();  // body FLU, N m
  };

  struct Command
  {
    double          specific_force = 0.0;             // along body +Z, m/s^2
    Eigen::Vector3d rates = Eigen::Vector3d::Zero();  // body FLU, rad/s
  };

  struct Feedback
  {
    Eigen::Vector3d specific_force =
        Eigen::Vector3d::Zero();  // ideal IMU, no gravity
    Wrench aerodynamic;
  };

  struct StepContext
  {
    std::int64_t index         = 0;
    std::int64_t time_ns       = 0;
    std::int64_t control_dt_ns = 0;
    double       time_s() const { return static_cast<double>(time_ns) * 1e-9; }
  };

  struct ControlResult
  {
    Command command;
    int     solver_status = 0;
  };

  struct StepRecord
  {
    StepContext   context;
    State         before;
    State         after;
    Feedback      feedback_before;
    Feedback      feedback_after;
    ControlResult control;
    double        solve_time_ms = 0.0;  // diagnostic wall time only
  };

  void validate_state(const State &state);
  void validate_command(const Command &command);

}  // namespace simple_sim
