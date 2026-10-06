#include "nmpc_controller/tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nmpc
{
  void CxSlopeEstimator::reset(double initial_slope, double slope_limit,
                               double covariance_limit)
  {
    slope_limit_       = slope_limit;
    covariance_limit_  = covariance_limit;
    intercept_         = 0.0;
    slope_             = initial_slope;
    speed_coefficient_ = 0.0;
    // Moderate initial covariance: the first few samples move the estimate
    // quickly, but one noisy sample cannot own it outright.
    p00_     = 1.0;
    p01_     = 0.0;
    p02_     = 0.0;
    p11_     = 1.0;
    p12_     = 0.0;
    p22_     = 1.0;
    updates_ = 0;
    skipped_ = 0;
  }

  void CxSlopeEstimator::update(double alpha, double coefficient,
                                double speed_squared)
  {
    if (!std::isfinite(alpha) || !std::isfinite(coefficient) ||
        !std::isfinite(speed_squared))
      return;
    // Regressor phi = [1, alpha, nu] with nu the normalised speed squared. The
    // normalisation keeps the third column the same order of magnitude as the
    // second, so the 3x3 solve stays well conditioned.
    const double nu = speed_squared / 25.0;  // (|v| / 5 m/s)^2
    const double a0 = p00_ + alpha * p01_ + nu * p02_;
    const double a1 = p01_ + alpha * p11_ + nu * p12_;
    const double a2 = p02_ + alpha * p12_ + nu * p22_;
    const double denominator =
        forgetting_ + a0 + alpha * a1 + nu * a2;  // lambda + phi^T P phi
    if (!(denominator > 0.0) || !std::isfinite(denominator))
      return;
    const double gain0 = a0 / denominator;
    const double gain1 = a1 / denominator;
    const double gain2 = a2 / denominator;
    const double residual =
        coefficient - (intercept_ + slope_ * alpha + speed_coefficient_ * nu);
    intercept_ += gain0 * residual;
    slope_ += gain1 * residual;
    speed_coefficient_ += gain2 * residual;

    p00_ =
        std::clamp((p00_ - gain0 * a0) / forgetting_, 0.0, covariance_limit_);
    p01_ = std::clamp((p01_ - gain0 * a1) / forgetting_, -covariance_limit_,
                      covariance_limit_);
    p02_ = std::clamp((p02_ - gain0 * a2) / forgetting_, -covariance_limit_,
                      covariance_limit_);
    p11_ =
        std::clamp((p11_ - gain1 * a1) / forgetting_, 1e-9, covariance_limit_);
    p12_ = std::clamp((p12_ - gain1 * a2) / forgetting_, -covariance_limit_,
                      covariance_limit_);
    p22_ =
        std::clamp((p22_ - gain2 * a2) / forgetting_, 1e-9, covariance_limit_);
    // Keep the recursion symmetric and the parameter inside the physical range.
    slope_ = std::clamp(slope_, -slope_limit_, slope_limit_);
    ++updates_;
  }

  void TrackingConfig::validate() const
  {
    if (!std::isfinite(horizon_s) || horizon_s <= 0.0 ||
        !std::isfinite(traj_res_s) || traj_res_s <= 0.0 ||
        !std::isfinite(ctrl_frq) || ctrl_frq <= 0.0 ||
        1.0 / ctrl_frq > traj_res_s + 1e-9 ||
        !std::isfinite(initial_specific_thrust) ||
        initial_specific_thrust <= 0.0 || !std::isfinite(cx_alpha_slope) ||
        !std::isfinite(cx_slope_forgetting) || cx_slope_forgetting <= 0.0 ||
        cx_slope_forgetting > 1.0 || !std::isfinite(cx_slope_limit) ||
        cx_slope_limit <= 0.0 || !std::isfinite(cx_slope_covariance_limit) ||
        cx_slope_covariance_limit <= 0.0 ||
        !std::isfinite(cx_slope_min_excitation) ||
        cx_slope_min_excitation < 0.0 || !std::isfinite(cx_slope_lag_s) ||
        cx_slope_lag_s < 0.0 ||
        cx_slope_lag_s >= static_cast<double>(kAlphaHistory) / ctrl_frq ||
        std::abs(cx_alpha_slope) > cx_slope_limit ||
        std::abs(horizon_s / traj_res_s - std::round(horizon_s / traj_res_s)) >
            1e-9)
      throw std::invalid_argument("invalid NMPC horizon/control configuration");
  }
  CostReference cost_reference(const std::array<double, 3> &p,
                               const std::array<double, 3> &v,
                               const std::array<double, 3> &yb)
  {
    CostReference y{};
    std::copy(p.begin(), p.end(), y.begin());
    std::copy(v.begin(), v.end(), y.begin() + 3);
    std::copy(yb.begin(), yb.end(), y.begin() + 6);
    // y[9:13] = 0: command-derivative regularization, no absolute command
    // target.
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
    reset({config_.initial_specific_thrust, 0.0, 0.0, 0.0});
  }
  void TrackingController::reset(
      const std::array<double, NMPC_NU> &initial_command)
  {
    for (double value : initial_command)
      if (!std::isfinite(value))
        throw std::invalid_argument("invalid initial NMPC command");
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
    previous_command_   = initial_command;
    aero_coefficient_x_ = 0.0;
    cx_slope_estimator_.reset(config_.cx_alpha_slope, config_.cx_slope_limit,
                              config_.cx_slope_covariance_limit);
    cx_alpha_slope_active_ = config_.cx_alpha_slope;
    alpha_history_.fill(0.0);
    alpha_write_         = 0;
    alpha_history_count_ = 0;
    previous_alpha_      = 0.0;
    has_previous_alpha_  = false;
    excitation_skips_    = 0;
    trace_               = {};
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
    {
      for (double value : y)
        if (!std::isfinite(value))
          throw std::runtime_error("non-finite NMPC reference");
      for (int i = 9; i < NMPC_NY; ++i)
        if (y[i] != 0.0)
          throw std::invalid_argument(
              "command derivative reference must be zero");
    }
    if (!std::isfinite(specific_force_x))
      throw std::runtime_error("invalid IMU feedback");
    const double speed2 =
        state[3] * state[3] + state[4] * state[4] + state[5] * state[5];
    // Lumped body-X specific-force coefficient, calibrated from the IMU. It
    // absorbs any actuator body-X component on purpose. Ground speed only:
    // the caller must guarantee zero wind.
    aero_coefficient_x_ =
        std::clamp(specific_force_x / (speed2 + 0.1), -0.4, 0.4);
    const double qw = state[9], qx = state[10], qy = state[11], qz = state[12];
    if (std::abs(qw * qw + qx * qx + qy * qy + qz * qz - 1.0) > 1e-6)
      throw std::runtime_error("NMPC requires a unit quaternion");
    // The estimator regresses the coefficient level on alpha, so it must use
    // the SAME regularised alpha as the prediction model's derivative term.
    const double vx = (1 - 2 * (qy * qy + qz * qz)) * state[3] +
                      2 * (qx * qy + qw * qz) * state[4] +
                      2 * (qx * qz - qw * qy) * state[5];
    const double vz = 2 * (qx * qz + qw * qy) * state[3] +
                      2 * (qy * qz - qw * qx) * state[4] +
                      (1 - 2 * (qx * qx + qy * qy)) * state[5];
    const double alpha = std::atan2(vx, vz + 0.1);
    // The estimator's own current value is the fallback; never fall back to the
    // configured seed, otherwise every low-speed tick would discard the
    // estimate.
    cx_alpha_slope_active_ = config_.cx_slope_estimation
                                 ? cx_slope_estimator_.slope()
                                 : config_.cx_alpha_slope;
    double alpha_delayed   = alpha;
    if (config_.cx_slope_estimation)
    {
      // Delay the regressor by the motor time constant when requested: cx lags
      // alpha, so a zero-lag fit absorbs that phase error into the slope.
      alpha_history_[alpha_write_] = alpha;
      alpha_write_                 = (alpha_write_ + 1) % kAlphaHistory;
      ++alpha_history_count_;
      const int available = static_cast<int>(
          std::min<std::int64_t>(alpha_history_count_, kAlphaHistory));
      const int lag = std::min(static_cast<int>(std::lround(
                                   config_.cx_slope_lag_s * config_.ctrl_frq)),
                               available - 1);
      alpha_delayed = alpha_history_[(alpha_write_ - 1 - lag + kAlphaHistory) %
                                     kAlphaHistory];
      // Measured excitation of the delayed regressor, at the control period.
      const double alpha_rate =
          has_previous_alpha_
              ? (alpha_delayed - previous_alpha_) * config_.ctrl_frq
              : 0.0;
      const bool excited =
          std::abs(alpha_rate) >= config_.cx_slope_min_excitation;
      if (speed2 > 1.0 && excited)
      {
        // Only an excited window informs the fit: the model's own speed guard,
        // plus enough d(alpha)/dt for the slope to be identifiable.
        cx_slope_estimator_.update(alpha_delayed, aero_coefficient_x_, speed2);
      }
      else if (speed2 > 1.0)
      {
        ++excitation_skips_;
      }
      cx_alpha_slope_active_ = cx_slope_estimator_.slope();
    }
    previous_alpha_                                 = alpha_delayed;
    has_previous_alpha_                             = true;
    const double                parameters[NMPC_NP] = {cx_alpha_slope_active_};
    std::array<double, NMPC_NX> x{};
    std::copy(state.begin(), state.end(), x.begin());
    // These augmented states are COMMANDS, not measured angular rates or lag
    // estimates. Never overwrite their memory with the plant's inner-loop
    // response.
    std::copy_n(previous_command_.begin() + 1, 3, x.begin() + 6);
    x[13] = previous_command_[0];
    x[14] = aero_coefficient_x_;
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
    std::array<double, NMPC_NU> command_derivative{};
    std::array<double, NMPC_NX> predicted{};
    const int                   status = nmpc_solver_solve(
        solver_.get(), command_derivative.data(), predicted.data());
    trace_.status             = status;
    trace_.command_derivative = command_derivative;
    if (status != 0 && status != 2)
      throw std::runtime_error("NMPC solve failed, status=" +
                               std::to_string(status));
    for (double value : predicted)
      if (!std::isfinite(value))
        throw std::runtime_error("non-finite NMPC prediction");
    std::array<double, NMPC_NU> next_command{};
    for (int i = 0; i < NMPC_NU; ++i)
    {
      next_command[i] =
          previous_command_[i] + command_derivative[i] / config_.ctrl_frq;
      if (!std::isfinite(command_derivative[i]) ||
          !std::isfinite(next_command[i]))
        throw std::runtime_error("non-finite NMPC command integration");
    }
    // Commit only an accepted finite solve. Integrate at the actual control dt,
    // not the OCP's 100 ms x_1, and do not publish du/dt as an absolute
    // command.
    previous_command_         = next_command;
    trace_.integrated_command = next_command;
    trace_.accepted           = true;
    return {{next_command[1], next_command[2], next_command[3]},
            next_command[0],
            status};
  }
}  // namespace nmpc
