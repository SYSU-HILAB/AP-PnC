/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Ported from PX4-Neupilot src/modules/acc_rates_control (AccRatesControl)
 * to ROS2/mavros: single-parameter RLS throttle model, first-order LPF,
 * and INDI-style incremental thrust command.
 *
 * Lives in px4ctrl: the throttle estimator selected by
 * `throttle_estimator` (tracking) and the NMPC controller mode both use
 * this model.
 */

#pragma once

#include <algorithm>
#include <cmath>

/**
 * Scalar recursive least-squares estimator of throttle effectiveness.
 *
 * Model (ROS-FLU positive domain): a = eta * u with u in [0,1] the
 * normalized collective throttle and a the thrust-axis specific force
 * (hover ~ +CONSTANTS_ONE_G). Follows the NeuPilot ThrottleModelRls
 * semantics: forgetting factor, eta bounds, minimum-thrust update gate,
 * variance-based validity, and innovation test ratio.
 */
class RlsThrottleModel
{
 public:
  RlsThrottleModel() = default;

  /**
   * Set the exponential forgetting factor.
   *
   * @param[in] lambda Forgetting factor in (0, 1] [-]
   */
  void setForgettingFactor(double lambda)
  {
    lambda_ = std::clamp(lambda, 0.9, 1.0);
  }

  /**
   * Bound the admissible throttle effectiveness.
   *
   * @param[in] eta_min Lower bound [m/s^2], clamped to at least 1 g
   * @param[in] eta_max Upper bound [m/s^2]
   */
  void setEtaBounds(double eta_min, double eta_max)
  {
    eta_min_ = std::max(eta_min, kOneG);
    eta_max_ = std::max(eta_max, eta_min_);
  }

  /**
   * Set the throttle below which no RLS update happens (low signal-to-noise).
   *
   * @param[in] u_min Minimum throttle for update [0,1]
   */
  void setMinThrustForUpdate(double u_min)
  {
    u_min_ = std::clamp(u_min, 0.0, 1.0);
  }

  /**
   * Set the initial estimator covariance.
   *
   * @param[in] p_init Initial variance [-]
   */
  void setPInit(double p_init) { p_init_ = std::max(p_init, 1.0e-6); }

  /**
   * Set the variance threshold below which the estimate is valid.
   *
   * @param[in] var_thr Validity variance threshold [-]
   */
  void setVarValidThreshold(double var_thr)
  {
    var_valid_thr_ = std::max(var_thr, 1.0e-9);
  }

  /**
   * Set the innovation test ratio gate; updates above it are rejected.
   *
   * @param[in] gate Test ratio gate [-]
   */
  void setInnovationGate(double gate)
  {
    innovation_gate_ = std::max(gate, 1.0);
  }

  /**
   * Set the assumed measurement noise variance of the specific force.
   *
   * @param[in] r Measurement variance [(m/s^2)^2]
   */
  void setMeasurementVariance(double r) { meas_var_ = std::max(r, 1.0e-6); }

  /**
   * Reset the estimator to a seeded effectiveness value.
   *
   * @param[in] eta0 Initial effectiveness [m/s^2]
   */
  void reset(double eta0)
  {
    eta_                   = std::clamp(eta0, eta_min_, eta_max_);
    P_                     = p_init_;
    innovation_            = 0.0;
    innovation_test_ratio_ = 0.0;
    valid_                 = false;
  }

  /**
   * Run one RLS recursion on a (throttle, specific force) sample pair.
   *
   * @param[in] u Normalized throttle command [0,1]
   * @param[in] a_meas Thrust-axis specific force measurement [m/s^2]
   */
  void step(double u, double a_meas)
  {
    if (!std::isfinite(u) || !std::isfinite(a_meas))
    {
      return;
    }

    const double prediction = eta_ * u;
    innovation_             = a_meas - prediction;

    // Test statistic: innovation^2 over predicted measurement variance.
    const double pred_var  = u * u * P_ + meas_var_;
    innovation_test_ratio_ = innovation_ * innovation_ / pred_var;

    if (u < u_min_ || innovation_test_ratio_ > innovation_gate_)
    {
      P_ /= lambda_;
      return;
    }

    const double gain = P_ * u / (lambda_ + u * u * P_);
    eta_ += gain * innovation_;
    P_     = (P_ - gain * u * P_) / lambda_;
    eta_   = std::clamp(eta_, eta_min_, eta_max_);
    valid_ = P_ < var_valid_thr_;
  }

  /** @return Effectiveness estimate [m/s^2] */
  double getEta() const { return eta_; }

  /** @return Estimator variance [-] */
  double getEtaVar() const { return P_; }

  /** @return Last innovation [m/s^2] */
  double getInnovation() const { return innovation_; }

  /** @return Last innovation test ratio [-] */
  double getInnovationTestRatio() const { return innovation_test_ratio_; }

  /** @return True once the variance converged below the validity threshold */
  bool isEstimateValid() const { return valid_; }

  static constexpr double kOneG = 9.80665;

 private:
  double lambda_          = 0.999;
  double eta_min_         = kOneG;
  double eta_max_         = 40.0;
  double u_min_           = 0.1;
  double p_init_          = 10.0;
  double var_valid_thr_   = 0.01;
  double innovation_gate_ = 5.0;
  double meas_var_        = 4.0;

  double eta_                   = kOneG / 0.5;
  double P_                     = 10.0;
  double innovation_            = 0.0;
  double innovation_test_ratio_ = 0.0;
  bool   valid_                 = false;
};

/**
 * Timestamp-driven first-order low-pass filter with per-sample dt.
 */
class FirstOrderLpf
{
 public:
  FirstOrderLpf() = default;

  /**
   * Configure the filter cutoff.
   *
   * @param[in] cutoff_hz Cutoff frequency [Hz]
   */
  void setCutoff(double cutoff_hz) { cutoff_hz_ = std::max(cutoff_hz, 0.1); }

  /** Drop the filter state; the next sample re-initializes the filter. */
  void reset()
  {
    state_       = 0.0;
    initialized_ = false;
  }

  /**
   * Apply the filter; initializes to the first sample.
   *
   * @param[in] x Raw sample [-]
   * @param[in] dt Time since the previous sample [s]
   * @return Filtered value [-]
   */
  double apply(double x, double dt)
  {
    if (!initialized_ || !std::isfinite(x))
    {
      state_       = std::isfinite(x) ? x : 0.0;
      initialized_ = std::isfinite(x);
      return state_;
    }
    const double alpha =
        1.0 - std::exp(-2.0 * M_PI * cutoff_hz_ * std::clamp(dt, 1.0e-5, 1.0));
    state_ += alpha * (x - state_);
    return state_;
  }

  /** @return Current filtered value [-] */
  double value() const { return state_; }

 private:
  double cutoff_hz_   = 20.0;
  double state_       = 0.0;
  bool   initialized_ = false;
};

/**
 * INDI-style incremental throttle command (reference: AccRatesControl
 * computeThrustAxisCommand, ROS-FLU positive domain).
 *
 * @param[in] u_current Last applied throttle [0,1]
 * @param[in] eta Throttle effectiveness estimate [m/s^2]
 * @param[in] specific_force_sp Thrust-axis specific force setpoint [m/s^2]
 * @param[in] specific_force Measured thrust-axis specific force [m/s^2]
 * @param[in] kp Specific-force feedback gain [1/s]
 * @param[in] dt Time step [s]
 * @param[in] u_min Throttle floor [0,1]
 * @param[in] u_max Throttle ceiling [0,1]
 * @return Throttle command clamped to [u_min, u_max] [0,1]
 */
inline double computeIndiThrottle(double u_current, double eta,
                                  double specific_force_sp,
                                  double specific_force, double kp, double dt,
                                  double u_min = 0.05, double u_max = 1.0)
{
  const double eta_safe = std::max(eta, 1.0e-3);
  const double error    = specific_force_sp - specific_force;
  const double u_cmd    = u_current + kp * error / eta_safe * dt;
  return std::clamp(u_cmd, u_min, u_max);
}
