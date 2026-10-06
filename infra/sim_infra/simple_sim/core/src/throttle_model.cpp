#include "simple_sim/throttle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simple_sim
{
  void ThrottleModelConfig::validate() const
  {
    if (realization != "ideal_invert" && realization != "indi")
      throw std::invalid_argument(
          "thrust realization must be ideal_invert or indi, got '" +
          realization + "'");
    // eta_seed == 0 means "derive the seed from the hover assumption"; it is
    // filled in by the constructor, so it is not an error here.
    if (!std::isfinite(eta_seed) || eta_seed < 0.0 || !std::isfinite(eta_min) ||
        !std::isfinite(eta_max) || eta_min <= 0.0 || eta_max < eta_min)
      throw std::invalid_argument("invalid throttle effectiveness bounds");
    if (!std::isfinite(forgetting_factor) || forgetting_factor <= 0.0 ||
        forgetting_factor > 1.0 || !std::isfinite(p_init) || p_init <= 0.0 ||
        !std::isfinite(measurement_variance) || measurement_variance <= 0.0)
      throw std::invalid_argument("invalid RLS throttle configuration");
    if (!std::isfinite(innovation_gate) || innovation_gate < 1.0 ||
        !std::isfinite(var_valid_threshold) || var_valid_threshold <= 0.0 ||
        !std::isfinite(min_throttle_update) || min_throttle_update < 0.0 ||
        min_throttle_update > 1.0)
      throw std::invalid_argument("invalid RLS gating configuration");
    if (!std::isfinite(specific_force_p) || specific_force_p <= 0.0 ||
        !std::isfinite(throttle_min) || !std::isfinite(throttle_max) ||
        throttle_min < 0.0 || throttle_max <= throttle_min ||
        throttle_max > 1.0)
      throw std::invalid_argument("invalid INDI throttle configuration");
    if (!std::isfinite(frq) || frq <= 0.0)
      throw std::invalid_argument("invalid INDI tick rate");
    if (!std::isfinite(lag_s) || lag_s < 0.0 ||
        !std::isfinite(throttle_tau_s) || throttle_tau_s < 0.0)
      throw std::invalid_argument("invalid throttle pairing configuration");
    if (!std::isfinite(measurement_tau_s) || measurement_tau_s < 0.0 ||
        !std::isfinite(setpoint_tau_s) || setpoint_tau_s < 0.0)
      throw std::invalid_argument("invalid throttle filter time constant");
  }

  ThrottleModel::ThrottleModel(const ThrottleModelConfig &config,
                               double                     seed_effectiveness)
      : config_(config)
  {
    config_.validate();
    if (!std::isfinite(seed_effectiveness) || seed_effectiveness <= 0.0)
      throw std::invalid_argument(
          "throttle effectiveness seed must be positive");
    // The vehicle seeds the estimator from grav / hover_throttle; the simulator
    // knows the same quotient, so the two start from the same place.
    if (config_.eta_seed <= 0.0 || config_.realization == "ideal_invert")
      config_.eta_seed = seed_effectiveness;
    reset();
  }

  void ThrottleModel::reset()
  {
    eta_ = std::clamp(config_.eta_seed, config_.eta_min, config_.eta_max);
    covariance_            = config_.p_init;
    innovation_            = 0.0;
    innovation_test_ratio_ = 0.0;
    valid_                 = false;
    throttle_              = 0.0;
    filters_initialised_   = false;
  }

  double ThrottleModel::low_pass(double value, double previous, double tau_s,
                                 double dt_s) const
  {
    if (tau_s <= 0.0 || dt_s >= tau_s)
      return value;
    const double alpha = dt_s / (tau_s + dt_s);
    return previous + alpha * (value - previous);
  }

  void ThrottleModel::step_estimator(double throttle, double specific_force)
  {
    if (throttle < config_.min_throttle_update)
      return;  // low signal-to-noise: no update below the update gate
    // Scalar RLS of a = eta * u with a forgetting factor. The sample is
    // (u = throttle, a = measured thrust-axis specific force).
    const double p_prior = covariance_ / config_.forgetting_factor;
    const double s =
        config_.measurement_variance + throttle * p_prior * throttle;
    const double gain      = p_prior * throttle / s;
    const double residual  = specific_force - eta_ * throttle;
    innovation_test_ratio_ = std::abs(residual) / std::sqrt(s);
    innovation_            = residual;
    if (innovation_test_ratio_ > config_.innovation_gate)
      return;  // rejected: keep eta, keep the covariance
    eta_ = std::clamp(eta_ + gain * residual, config_.eta_min, config_.eta_max);
    covariance_ = std::max((1.0 - gain * throttle) * p_prior, 1.0e-9);
    valid_      = covariance_ < config_.var_valid_threshold;
  }

  double ThrottleModel::command(double specific_force_sp,
                                double specific_force_meas, double dt_s)
  {
    if (!std::isfinite(specific_force_sp) ||
        !std::isfinite(specific_force_meas) || !std::isfinite(dt_s) ||
        dt_s <= 0.0)
      throw std::invalid_argument("invalid throttle command inputs");

    if (!filters_initialised_)
    {
      measurement_lpf_ = specific_force_meas;
      setpoint_lpf_    = specific_force_sp;
      // Seed the throttle from the static inverse so the first increment starts
      // from a usable point instead of zero.
      throttle_ = std::clamp(specific_force_sp / eta_, config_.throttle_min,
                             config_.throttle_max);
      filters_initialised_ = true;
      return throttle_;
    }
    measurement_lpf_ = low_pass(specific_force_meas, measurement_lpf_,
                                config_.measurement_tau_s, dt_s);
    setpoint_lpf_    = low_pass(specific_force_sp, setpoint_lpf_,
                                config_.setpoint_tau_s, dt_s);

    if (config_.realization == "ideal_invert")
    {
      // Exact inverse of a = eta * u with the modelled effectiveness. No
      // estimate is updated, so any plant/model mismatch shows up as a
      // specific-force error.
      throttle_ = std::clamp(setpoint_lpf_ / eta_, config_.throttle_min,
                             config_.throttle_max);
      return throttle_;
    }

    // INDI: estimate the effectiveness on the applied throttle, then take one
    // increment from the previous throttle proportional to the specific-force
    // error. This is px4ctrl's computeIndiThrottle.
    // Record the throttle applied during the last tick and pair the estimator
    // with the command from `lag_s` ago, filtered. Pairing the held command
    // with a force that lags through the motor and ESC biases the effectiveness
    // estimate; lag 0 with the filter is px4ctrl's INDI path.
    command_ring_[command_write_] = throttle_;
    command_write_                = (command_write_ + 1) % command_ring_.size();
    const std::size_t lag_ticks =
        static_cast<std::size_t>(std::llround(config_.lag_s / dt_s)) %
        command_ring_.size();
    const std::size_t index =
        (command_write_ + command_ring_.size() - 1 - lag_ticks) %
        command_ring_.size();
    throttle_lpf_ = low_pass(command_ring_[index], throttle_lpf_,
                             config_.throttle_tau_s, dt_s);
    step_estimator(throttle_lpf_, measurement_lpf_);
    const double error = setpoint_lpf_ - measurement_lpf_;
    throttle_ =
        std::clamp(throttle_ + config_.specific_force_p * error / eta_ * dt_s,
                   config_.throttle_min, config_.throttle_max);
    return throttle_;
  }

}  // namespace simple_sim
