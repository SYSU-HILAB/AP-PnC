#include "controller.h"

#include <stdexcept>

#include "controller/alg0_differential_flatness.h"
#include "controller/alg1_geometric.h"
#include "controller/alg2_rotor_drag.h"

using namespace std;

Controller::Controller(Parameters &param)
    : param_(param),
      pid_(param),
      att_fb_(param),
      limiter_(param.max_angle, param.gra),
      throttle_(param),
      yaw_target_(param)
{
}

std::unique_ptr<TrackingControllerBase> Controller::makeSolver()
{
  switch (param_.pose_solver)
  {
    case 0:
      return std::make_unique<Alg0DifferentialFlatness>(param_, pid_, att_fb_,
                                                        limiter_, throttle_);
    case 1:
      return std::make_unique<Alg1Geometric>(param_, pid_, att_fb_, limiter_,
                                             throttle_);
    case 2:
      return std::make_unique<Alg2RotorDrag>(param_, pid_, att_fb_, limiter_,
                                             throttle_);
    default:
      throw std::runtime_error("Illegal pose_solver selection: " +
                               std::to_string(param_.pose_solver));
  }
}

quadrotor_msgs::msg::Px4ctrlDebug Controller::update(const DesiredState &des,
                                                     const OdomData     &odom,
                                                     const ImuData      &imu,
                                                     ControllerOutput   &u,
                                                     double voltage)
{
  if (!solver_ || solver_id_ != param_.pose_solver)
  {
    solver_    = makeSolver();
    solver_id_ = param_.pose_solver;
  }
  solver_->update(des, odom, imu, u, voltage, debug_);
  // hover_percentage is owned by the throttle estimator path and only
  // refreshed there; backfill the cached RLS value for solvers that do
  // not touch the debug field (alg2), mirroring updateIndiThrottleModel.
  if (param_.throttle_estimator == 1)
  {
    debug_.hover_percentage = param_.gra / throttle_.getEta();
  }
  return debug_;
}

double Controller::computeYawTargetBodyRate(const double target_yaw,
                                            const double current_yaw) const
{
  return yaw_target_.compute(target_yaw, current_yaw);
}

Eigen::Vector3d Controller::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond &des_q, const Eigen::Quaterniond &est_q)
{
  return att_fb_.compute(des_q, est_q, debug_);
}

void Controller::resetThrustMapping(void)
{
  throttle_.resetThrustMapping();
  limiter_.reset();
  if (solver_)
    solver_->reset();
}

void Controller::setControlDt(double dt)
{
  throttle_.setControlDt(dt);
}

bool Controller::updateIndiThrottleModel(double specific_force)
{
  const bool valid        = throttle_.updateIndiThrottleModel(specific_force);
  debug_.hover_percentage = param_.gra / throttle_.getEta();
  return valid;
}

double Controller::computeNmpcThrottle(double specific_force,
                                       double specific_force_sp)
{
  return throttle_.computeNmpcThrottle(specific_force, specific_force_sp);
}

void Controller::fillThrottleStatus(
    interface::msg::ThrottleModelStatus &status) const
{
  throttle_.fillThrottleStatus(status);
}

bool Controller::estimateThrustModel(const Eigen::Vector3d &est_a,
                                     double                 voltage,
                                     const Eigen::Vector3d &est_v)
{
  return throttle_.estimateThrustModel(est_a, voltage, est_v, debug_);
}
