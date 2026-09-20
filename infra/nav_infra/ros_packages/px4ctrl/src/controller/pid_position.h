#ifndef __CONTROLLER_PID_POSITION_H
#define __CONTROLLER_PID_POSITION_H

#include <Eigen/Dense>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include "PX4CtrlParam.h"
#include "types.h"

/**
 * Cascaded P/PD position-to-acceleration law (I/D terms reserved).
 *
 * Holds its own Kp/Kv copies; the legacy Kvi/Kvd gains and int_e_v
 * were write-only in the old Controller and are not carried over.
 */
class PidPosition
{
 public:
  explicit PidPosition(const Parameters &param);

  /**
   * Desired acceleration from position/velocity errors [m/s^2].
   *
   * @param[in] odom Current odometry (world frame) [-]
   * @param[in] des Desired state (position/velocity feedforward) [-]
   * @param[out] dbg Telemetry (des_v_*) to publish [-]
   * @return Feedback acceleration in world [m/s^2]
   */
  Eigen::Vector3d compute(const OdomData &odom, const DesiredState &des,
                          quadrotor_msgs::msg::Px4ctrlDebug &dbg) const;

 private:
  Eigen::Vector3d Kp_;
  Eigen::Vector3d Kv_;
};

#endif
