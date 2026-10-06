#include <cmath>
#include <stdexcept>
#include <utility>

#include "simple_sim/adapters/controllers.hpp"

namespace simple_sim
{
  namespace
  {
    Eigen::Matrix3d desired_attitude(const Eigen::Vector3d &force,
                                     const Eigen::Vector3d &yb)
    {
      if (!force.allFinite() || force.norm() < 1e-9 || !yb.allFinite())
        throw std::runtime_error("degenerate SE3 reference");
      const Eigen::Vector3d z = force.normalized();
      Eigen::Vector3d       y = yb - z * yb.dot(z);
      if (y.norm() < 1e-9)
        throw std::runtime_error("reference yb parallel to thrust axis");
      y.normalize();
      Eigen::Matrix3d r;
      r.col(0) = y.cross(z);
      r.col(1) = y;
      r.col(2) = z;
      return r;
    }
  }  // namespace
  State initial_state(const planner::core::ReferencePoint &ref, double gravity)
  {
    State s;
    s.position = ref.p;
    s.velocity = ref.v;
    s.attitude = Eigen::Quaterniond(
        desired_attitude(ref.a + gravity * Eigen::Vector3d::UnitZ(), ref.yb));
    s.angular_velocity = ref.omega;
    validate_state(s);
    return s;
  }
  Se3Controller::Se3Controller(
      std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
      double gravity, Se3Gains gains)
      : trajectory_(std::move(trajectory)),
        gravity_(gravity),
        gains_(std::move(gains))
  {
    if (!trajectory_ || trajectory_->empty() || !std::isfinite(gravity_) ||
        gravity_ <= 0.0)
      throw std::invalid_argument("invalid SE3 trajectory or gravity");
    for (const auto *v : {&gains_.position, &gains_.velocity, &gains_.attitude})
      if (!v->allFinite() || (v->array() <= 0.0).any())
        throw std::invalid_argument("SE3 gains must be finite and positive");
  }
  void Se3Controller::reset(const State &initial)
  {
    validate_state(initial);
  }
  ControlResult Se3Controller::compute(const StepContext &ctx, const State &s,
                                       const Feedback &)
  {
    if (ctx.time_s() < 0.0 || ctx.time_s() > trajectory_->duration())
      throw std::runtime_error("SE3 reference time outside trajectory");
    const auto            ref = trajectory_->sample(ctx.time_s());
    const Eigen::Vector3d force =
        ref.a + gravity_ * Eigen::Vector3d::UnitZ() +
        gains_.position.cwiseProduct(ref.p - s.position) +
        gains_.velocity.cwiseProduct(ref.v - s.velocity);
    const Eigen::Matrix3d   desired = desired_attitude(force, ref.yb);
    const Eigen::Matrix3d   current = s.attitude.toRotationMatrix();
    const Eigen::AngleAxisd error(desired * current.transpose());
    ControlResult           result;
    result.command.specific_force = force.dot(current.col(2));
    result.command.rates =
        gains_.attitude.cwiseProduct(current.transpose() *
                                     (error.axis() * error.angle())) +
        current.transpose() * desired * ref.omega;
    return result;
  }
}  // namespace simple_sim
