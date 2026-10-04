#include "simple_sim/adapters/nmpc_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace simple_sim
{
  NmpcController::NmpcController(
      std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
      nmpc::TrackingConfig                                      config)
      : trajectory_(std::move(trajectory)), controller_(config)
  {
    if (!trajectory_ || trajectory_->empty())
      throw std::invalid_argument("empty NMPC trajectory");
  }
  void NmpcController::reset(const State &initial)
  {
    validate_state(initial);
    controller_.reset();
  }
  ControlResult NmpcController::compute(const StepContext &ctx, const State &s,
                                        const Feedback &feedback)
  {
    const auto &cfg = controller_.config();
    if (std::abs(static_cast<double>(ctx.control_dt_ns) * 1e-9 -
                 1.0 / cfg.ctrl_frq) > 1e-9)
      throw std::runtime_error("runner control period differs from nmpc.yaml");
    const auto n =
        static_cast<int>(std::llround(cfg.horizon_s / cfg.traj_res_s));
    std::vector<nmpc::CostReference> horizon;
    horizon.reserve(n + 1);
    for (int i = 0; i <= n; ++i)
    {
      const double t = ctx.time_s() + i * cfg.traj_res_s;
      if (t < 0.0 || t > trajectory_->duration())
        throw std::runtime_error("NMPC horizon outside planned trajectory");
      const auto ref = trajectory_->sample(t);
      horizon.push_back(nmpc::cost_reference(
          {ref.p.x(), ref.p.y(), ref.p.z()}, {ref.v.x(), ref.v.y(), ref.v.z()},
          {ref.yb.x(), ref.yb.y(), ref.yb.z()}, cfg.u_hover));
    }
    const std::array<double, 13> x = {
        s.position.x(),         s.position.y(),         s.position.z(),
        s.velocity.x(),         s.velocity.y(),         s.velocity.z(),
        s.angular_velocity.x(), s.angular_velocity.y(), s.angular_velocity.z(),
        s.attitude.w(),         s.attitude.x(),         s.attitude.y(),
        s.attitude.z()};
    const auto result =
        controller_.compute(x, feedback.specific_force.x(), horizon);
    return {
        {result.specific_force,
         Eigen::Vector3d(result.rates[0], result.rates[1], result.rates[2])},
        result.status};
  }
}  // namespace simple_sim
