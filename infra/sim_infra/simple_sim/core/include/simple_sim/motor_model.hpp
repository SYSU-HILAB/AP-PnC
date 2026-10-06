#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>

#include "simple_sim/types.hpp"

namespace simple_sim
{
  // Parameters are loaded from the private upstream's numeric .mat snapshot,
  // not guessed defaults. See stack/import_matlab.py and local provenance.
  struct MotorModelConfig
  {
    Eigen::Vector4d arms         = Eigen::Vector4d::Zero();
    Eigen::Vector4d arm_angles   = Eigen::Vector4d::Zero();
    Eigen::Vector4d tilts        = Eigen::Vector4d::Zero();
    Eigen::Matrix4d mixer        = Eigen::Matrix4d::Zero();
    Eigen::Vector3d thrust_poly  = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_poly  = Eigen::Vector3d::Zero();
    Eigen::Vector3d esc_poly     = Eigen::Vector3d::Zero();
    Eigen::Vector3d damping_wing = Eigen::Vector3d::Zero();
    Eigen::Vector3d kp           = Eigen::Vector3d::Zero();
    Eigen::Vector3d ki           = Eigen::Vector3d::Zero();
    Eigen::Vector3d kd           = Eigen::Vector3d::Zero();
    Eigen::Vector3d wind_world   = Eigen::Vector3d::Zero();
    double          tau = 0.0, diameter = 0.0, ct = 0.0, cq = 0.0, jm = 0.0;
    double          pwm_min = 0.0, pwm_max = 0.0;
    std::int64_t    inner_dt_ns = 0;
    void            validate() const;
  };

  struct MixerResult
  {
    Eigen::Vector4d fraction = Eigen::Vector4d::Zero();
    bool            lower = false, upper = false;
  };

  // Pure component operations; no RK4-stage mutation or global PID/motor state.
  class MotorModel
  {
   public:
    MotorModel(MotorModelConfig config, double mass, double max_specific_force,
               Eigen::Vector3d max_rate, std::int64_t physics_dt_ns);
    MixerResult     mix(const Eigen::Vector4d &normalized) const;
    double          esc_rpm(double pwm) const;
    Eigen::Vector2d propeller(double rpm, double axial_speed) const;
    Wrench          wrench(const State &state) const;
    Eigen::Vector4d derivative(const State &state) const;
    State initialize(const State &state, const Command &command) const;
    State prepare(const State &state, const Command &command) const;

    /**
     * @brief The model's own static inverse: the throttle it would use for a
     * thrust-axis specific force. Exposed so the effectiveness seed of the
     * online realization starts from the modelled hover quotient instead of an
     * assumed hover throttle.
     */
    double static_throttle(double specific_force) const
    {
      return throttle(specific_force);
    }

    /**
     * @brief Use a throttle realized by the inner loop instead of inverting the
     * modelled curve here. Const because the plant holds a const motor model;
     * the realized value is runtime state, not configuration.
     */
    void set_realized_throttle(double throttle) const
    {
      realized_throttle_ = throttle;
    }
    void clear_realized_throttle() const { realized_throttle_.reset(); }
    bool has_realized_throttle() const
    {
      return realized_throttle_.has_value();
    }

   private:
    mutable std::optional<double>  realized_throttle_;
    MotorModelConfig               cfg_;
    double                         mass_, max_force_;
    Eigen::Vector3d                max_rate_;
    std::int64_t                   inner_stride_;
    std::array<Eigen::Matrix3d, 4> rotations_;
    double                         throttle(double specific_force) const;
  };
}  // namespace simple_sim
