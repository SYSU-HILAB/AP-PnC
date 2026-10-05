#include "nmpc_controller/tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nmpc
{
  void TrackingConfig::validate() const
  {
    if (!std::isfinite(horizon_s) || horizon_s <= 0.0 ||
        !std::isfinite(traj_res_s) || traj_res_s <= 0.0 ||
        !std::isfinite(ctrl_frq) || ctrl_frq <= 0.0 ||
        !std::isfinite(u_hover) || u_hover <= 0.0 || !std::isfinite(cz_rate) ||
        cz_rate < 0.0 || !std::isfinite(thrust_tau_s) || thrust_tau_s <= 0.0 ||
        std::abs(horizon_s / traj_res_s - std::round(horizon_s / traj_res_s)) >
            1e-9)
      throw std::invalid_argument("invalid NMPC horizon/control configuration");
    for (double tau : rate_tau_s)
      if (!std::isfinite(tau) || tau <= 0.0)
        throw std::invalid_argument("invalid NMPC rate time constant");
  }
  CostReference cost_reference(const std::array<double, 3> &p,
                               const std::array<double, 3> &v,
                               const std::array<double, 3> &yb, double hover)
  {
    CostReference y{};
    std::copy(p.begin(), p.end(), y.begin());
    std::copy(v.begin(), v.end(), y.begin() + 3);
    std::copy(yb.begin(), yb.end(), y.begin() + 9);
    y[12] = hover;  // collective command reference; rate setpoints default zero
    return y;
  }
  TrackingController::TrackingController(TrackingConfig config)
      : config_(config)
  {
    config_.validate();
    if (nmpc_solver_abi_version() != NMPC_SOLVER_ABI)
      throw std::runtime_error("stale NMPC bundle ABI; regenerate and relink");
    reset();
  }
  void TrackingController::reset()
  {
    solver_.reset(nmpc_solver_create());
    if (!solver_)
      throw std::runtime_error("cannot create NMPC solver");
    const int n = nmpc_solver_horizon(solver_.get());
    if (std::abs(n * config_.traj_res_s - config_.horizon_s) > 1e-9)
      throw std::runtime_error(
          "NMPC bundle horizon differs from nmpc.yaml; regenerate bundle");
    for (int i = 0; i < n; ++i)
      if (std::abs(nmpc_solver_stage_dt(solver_.get(), i) -
                   config_.traj_res_s) > 1e-9)
        throw std::runtime_error(
            "NMPC bundle grid differs from nmpc.yaml; regenerate bundle");
    specific_force_estimate_    = config_.u_hover;
    previous_specific_force_sp_ = config_.u_hover;
    cz_                         = 0.0;
    trace_                      = {};
  }
  TrackingResult TrackingController::compute(
      const std::array<double, 13> &state, double specific_force_x,
      const std::vector<CostReference> &horizon)
  {
    trace_      = {};
    const int n = nmpc_solver_horizon(solver_.get());
    if (horizon.size() != static_cast<std::size_t>(n + 1))
      throw std::invalid_argument("NMPC requires exactly N+1 reference points");
    for (double value : state)
      if (!std::isfinite(value))
        throw std::runtime_error("non-finite NMPC state");
    for (const auto &y : horizon)
      for (double value : y)
        if (!std::isfinite(value))
          throw std::runtime_error("non-finite NMPC reference");
    if (!std::isfinite(specific_force_x))
      throw std::runtime_error("invalid IMU feedback");
    const double speed2 =
        state[3] * state[3] + state[4] * state[4] + state[5] * state[5];
    cz_ = std::clamp(specific_force_x / (speed2 + 0.1), -0.4, 0.4);
    // Body velocity x/z from inverse Hamilton quaternion, state quaternion
    // order wxyz.
    const double qw = state[9], qx = state[10], qy = state[11], qz = state[12];
    if (std::abs(qw * qw + qx * qx + qy * qy + qz * qz - 1.0) > 1e-6)
      throw std::runtime_error("NMPC requires a unit quaternion");
    const double vx = (1 - 2 * (qy * qy + qz * qz)) * state[3] +
                      2 * (qx * qy + qw * qz) * state[4] +
                      2 * (qx * qz - qw * qy) * state[5];
    const double vz = 2 * (qx * qz + qw * qy) * state[3] +
                      2 * (qy * qz - qw * qx) * state[4] +
                      (1 - 2 * (qx * qx + qy * qy)) * state[5];
    const double alpha = std::atan2(vx, vz);
    const double pi    = std::acos(-1.0);
    const double rate =
        alpha > 0.0 && alpha < pi / 2
            ? config_.cz_rate
            : (alpha >= pi / 2 && alpha < pi ? -config_.cz_rate : 0.0);
    double                      parameters[NMPC_NP] = {cz_,
                                                       alpha,
                                                       rate,
                                                       config_.thrust_tau_s,
                                                       config_.rate_tau_s[0],
                                                       config_.rate_tau_s[1],
                                                       config_.rate_tau_s[2]};
    std::array<double, NMPC_NX> x{};
    std::copy(state.begin(), state.end(), x.begin());
    // Estimate the lagged collective over the actual controller period,
    // never use x_1 at the solver's (potentially different) horizon spacing.
    const double estimate =
        previous_specific_force_sp_ +
        (specific_force_estimate_ - previous_specific_force_sp_) *
            std::exp(-1.0 / (config_.ctrl_frq * config_.thrust_tau_s));
    x[13] = estimate;
    x[14] = cz_;
    nmpc_solver_set_state(solver_.get(), x.data());
    for (int i = 0; i < n; ++i)
      nmpc_solver_set_ref_stage(solver_.get(), i, horizon[i].data(),
                                parameters);
    std::array<double, NMPC_NYN> terminal{};
    std::copy_n(horizon.back().begin(), NMPC_NYN, terminal.begin());
    nmpc_solver_set_ref_terminal(solver_.get(), terminal.data());
    trace_.references = horizon;
    trace_.terminal   = terminal;
    trace_.state      = x;
    std::copy_n(parameters, NMPC_NP, trace_.parameters.begin());
    trace_.submitted = true;
    std::array<double, NMPC_NU> command{};
    std::array<double, NMPC_NX> predicted{};
    const int                   status =
        nmpc_solver_solve(solver_.get(), command.data(), predicted.data());
    trace_.status = status;
    // Status 2 is the existing controller's accepted max-iteration result;
    // record it explicitly.
    if (status != 0 && status != 2)
      throw std::runtime_error("NMPC solve failed, status=" +
                               std::to_string(status));
    for (double value : predicted)
      if (!std::isfinite(value))
        throw std::runtime_error("non-finite NMPC prediction");
    for (double value : command)
      if (!std::isfinite(value))
        throw std::runtime_error("non-finite NMPC command");
    specific_force_estimate_    = estimate;
    previous_specific_force_sp_ = command[0];
    TrackingResult result;
    // Publish u_0 itself: desired collective and body rates, not future x_1.
    result.rates          = {command[1], command[2], command[3]};
    result.specific_force = command[0];
    result.status         = status;
    return result;
  }
}  // namespace nmpc
