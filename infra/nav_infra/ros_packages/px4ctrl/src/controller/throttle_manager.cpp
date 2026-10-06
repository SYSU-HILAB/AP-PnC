#include "throttle_manager.h"

#include <algorithm>
#include <cmath>

ThrottleManager::ThrottleManager(Parameters &param) : param_(param)
{
  rls_.setForgettingFactor(param_.rls.lambda);
  rls_.setEtaBounds(param_.rls.eta_min, param_.rls.eta_max);
  rls_.setMinThrustForUpdate(param_.rls.u_min);
  rls_.setPInit(param_.rls.p_init);
  rls_.setVarValidThreshold(param_.rls.var_valid_thr);
  rls_.setInnovationGate(param_.rls.innovation_gate);

  setpoint_lpf_.setCutoff(param_.indi.setpoint_lpf_cutoff_hz);
  meas_lpf_.setCutoff(param_.indi.meas_lpf_cutoff_hz);
  throttle_lpf_.setCutoff(param_.indi.throttle_lpf_cutoff_hz);

  resetThrustMapping();
}

double ThrottleManager::computeDesiredCollectiveThrustSignal(
    const Eigen::Quaterniond &est_q, const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc, double voltage,
    quadrotor_msgs::msg::Px4ctrlDebug &dbg)
{
  double                normalized_thrust;
  const Eigen::Vector3d body_z_axis  = est_q * Eigen::Vector3d::UnitZ();
  double                des_acc_norm = des_acc.dot(body_z_axis);
  if (des_acc_norm < kMinNormalizedCollectiveAcc)
  {
    des_acc_norm = kMinNormalizedCollectiveAcc;
  }

  // This compensates for an acceleration component in thrust direction due
  // to the square of the body-horizontal velocity.
  des_acc_norm -= param_.rt_drag.k_thrust_horz *
                  (pow(est_v.x(), 2.0) + pow(est_v.y(), 2.0));

  dbg.des_thr = des_acc_norm;

  if (param_.throttle_estimator == 1)
  {
    normalized_thrust = computeIndiThrustSignal(des_acc_norm);
  }
  else if (param_.thr_map.accurate_thrust_model)
  {
    normalized_thrust =
        thr_scale_compensate * accurateThrustAccMapping(des_acc_norm, voltage);
  }
  else
  {
    normalized_thrust = des_acc_norm / thr2acc;
  }

  // Keep the last commanded throttle for the INDI state and telemetry.
  throttle_memory_ = normalized_thrust;
  pushTimedThrust(normalized_thrust);

  return normalized_thrust;
}

double ThrottleManager::accurateThrustAccMapping(double des_acc_z,
                                                 double voltage) const
{
  if (voltage < param_.low_voltage)
  {
    voltage = param_.low_voltage;
    RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"), "Low voltage!");
  }
  if (voltage > 1.5 * param_.low_voltage)
  {
    voltage = 1.5 * param_.low_voltage;
  }

  // F=K1*Voltage^K2*(K3*u^2+(1-K3)*u)
  double a      = param_.thr_map.K3;
  double b      = 1 - param_.thr_map.K3;
  double c      = -(param_.mass * des_acc_z) /
                  (param_.thr_map.K1 * pow(voltage, param_.thr_map.K2));
  double b2_4ac = pow(b, 2) - 4 * a * c;
  if (b2_4ac <= 0)
    b2_4ac = 0;
  double thrust = (-b + sqrt(b2_4ac)) / (2 * a);
  // if (thrust <= 0) thrust = 0; // This should be avoided before calling this
  // function
  return thrust;
}

void ThrottleManager::pushTimedThrust(double thrust)
{
  timed_thrust_.push(
      std::pair<rclcpp::Time, double>(rclcpp::Clock().now(), thrust));
  while (timed_thrust_.size() > 100)
    timed_thrust_.pop();
}

bool ThrottleManager::estimateThrustModel(
    const Eigen::Vector3d &est_a, double voltage, const Eigen::Vector3d &est_v,
    quadrotor_msgs::msg::Px4ctrlDebug &dbg)
{
  rclcpp::Time t_now = rclcpp::Clock().now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<rclcpp::Time, double> t_t         = timed_thrust_.front();
    double                          time_passed = (t_now - t_t.first).seconds();
    if (time_passed > 0.045)  // 45ms
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035)  // 35ms
    {
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    if (param_.thr_map.accurate_thrust_model)
    {
      /**************************************************************************/
      /* Model: thr = thr_scale_compensate * AccurateThrustAccMapping(est_a(2))
       */
      /**************************************************************************/
      double thr_fb = accurateThrustAccMapping(est_a(2), voltage);
      double gamma  = 1 / (rho2 + thr_fb * P * thr_fb);
      double K      = gamma * P * thr_fb;
      thr_scale_compensate =
          thr_scale_compensate + K * (thr - thr_fb * thr_scale_compensate);
      P = (1 - K * thr_fb) * P / rho2;

      if (thr_scale_compensate > 1.15 || thr_scale_compensate < 0.85)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("px4ctrl"),
            "Thrust scale = %f, which should be around 1. It means the thrust model is no longer accurate. \
                  Re-calibrate the thrust model!",
            thr_scale_compensate);
        thr_scale_compensate =
            thr_scale_compensate > 1.15 ? 1.15 : thr_scale_compensate;
        thr_scale_compensate =
            thr_scale_compensate < 0.85 ? 0.85 : thr_scale_compensate;
      }

      dbg.thr_scale_compensate = thr_scale_compensate;
      dbg.voltage              = voltage;
      if (param_.thr_map.print_val)
      {
        RCLCPP_WARN(rclcpp::get_logger("px4ctrl"), "thr_scale_compensate = %f",
                    thr_scale_compensate);
      }
    }
    else
    {
      /***********************************/
      /* Model: est_a(2) = thr2acc * thr */
      /***********************************/
      if (!param_.thr_map.noisy_imu)
      {
        double gamma = 1 / (rho2 + thr * P * thr);
        double K     = gamma * P * thr;
        thr2acc      = thr2acc + K * (est_a(2) - thr * thr2acc);
        P            = (1 - K * thr) * P / rho2;
      }
      else  // Strongly not recommended to use!!!
      {
        double K =
            10 / param_.ctrl_freq_max;  // thr2acc changes 10 every second when
                                        // est_v(2) - des_v(2) = 1 m/s
        thr2acc = thr2acc + K * (est_v(2) - dbg.des_v_z);
      }
      const double hover_percentage = param_.gra / thr2acc;
      if (hover_percentage > 0.8 || hover_percentage < 0.1)
      {
        RCLCPP_ERROR(rclcpp::get_logger("px4ctrl"),
                     "Estimated hover_percentage >0.8 or <0.1! Perhaps the "
                     "accel vibration is too high!");
        thr2acc = hover_percentage > 0.8 ? param_.gra / 0.8 : thr2acc;
        thr2acc = hover_percentage < 0.1 ? param_.gra / 0.1 : thr2acc;
      }
      dbg.hover_percentage = hover_percentage;
      if (param_.thr_map.print_val)
      {
        RCLCPP_WARN(rclcpp::get_logger("px4ctrl"), "hover_percentage = %f",
                    dbg.hover_percentage);
      }
    }

    return true;
  }

  return false;
}

void ThrottleManager::resetThrustMapping()
{
  thr2acc              = param_.gra / param_.thr_map.hover_percentage;
  thr_scale_compensate = 1.0;
  P                    = 1e6;

  // RLS eta model and INDI state share the hover seed so both estimators
  // start from the configured hover throttle.
  rls_.reset(param_.gra / param_.thr_map.hover_percentage);
  throttle_memory_ = std::clamp(param_.thr_map.hover_percentage, 0.0, 1.0);
  setpoint_lpf_.reset();
  meas_lpf_.reset();
  throttle_lpf_.reset();
}

void ThrottleManager::setControlDt(double dt)
{
  control_dt_ = std::clamp(dt, 1.0e-4, 0.1);
}

bool ThrottleManager::updateIndiThrottleModel(double specific_force)
{
  const double a_lpf = meas_lpf_.apply(specific_force, control_dt_);
  const double u_lpf = throttle_lpf_.apply(throttle_memory_, control_dt_);

  rls_.step(u_lpf, a_lpf);

  return rls_.isEstimateValid();
}

double ThrottleManager::computeNmpcThrottle(double specific_force,
                                            double specific_force_sp)
{
  updateIndiThrottleModel(specific_force);

  const double a_sp_lpf = setpoint_lpf_.apply(specific_force_sp, control_dt_);
  throttle_memory_      = computeIndiThrottle(
      throttle_memory_, rls_.getEta(), a_sp_lpf, meas_lpf_.value(),
      param_.indi.specific_force_p, control_dt_, param_.indi.throttle_min,
      param_.indi.throttle_max);
  return throttle_memory_;
}

double ThrottleManager::computeIndiThrustSignal(double specific_force_sp)
{
  const double a_sp_lpf = setpoint_lpf_.apply(specific_force_sp, control_dt_);
  return computeIndiThrottle(throttle_memory_, rls_.getEta(), a_sp_lpf,
                             meas_lpf_.value(), param_.indi.specific_force_p,
                             control_dt_, param_.indi.throttle_min,
                             param_.indi.throttle_max);
}

void ThrottleManager::fillThrottleStatus(
    interface::msg::ThrottleModelStatus &status) const
{
  status.eta                     = rls_.getEta();
  status.eta_variance            = rls_.getEtaVar();
  status.innovation              = rls_.getInnovation();
  status.innovation_test_ratio   = rls_.getInnovationTestRatio();
  status.estimate_valid          = rls_.isEstimateValid();
  status.throttle_input          = throttle_lpf_.value();
  status.measured_specific_force = meas_lpf_.value();
}
