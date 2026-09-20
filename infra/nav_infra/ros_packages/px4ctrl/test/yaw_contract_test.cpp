#include <gtest/gtest.h>

#include <cmath>

#include "controller.h"

namespace
{

  constexpr double kPi = 3.14159265358979323846;

  double rad(const double degrees)
  {
    return degrees * kPi / 180.0;
  }

  double shortestAngle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  Eigen::Quaterniond yawQuaternion(const double yaw)
  {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  double physicalQuaternionAngle(const Eigen::Quaterniond &lhs,
                                 const Eigen::Quaterniond &rhs)
  {
    const double dot = std::min(1.0, std::max(-1.0, std::abs(lhs.dot(rhs))));
    return 2.0 * std::acos(dot);
  }

  Parameters makeParameters()
  {
    Parameters parameter;
    parameter.mass                          = 1.0;
    parameter.gra                           = 9.81;
    parameter.pose_solver                   = 0;
    parameter.max_angle                     = rad(80.0);
    parameter.ctrl_freq_max                 = 200.0;
    parameter.max_yaw_target_mode_rate_max  = 1.0;
    parameter.yaw_target_kp                 = 1.0;
    parameter.yaw_target_deadband           = rad(2.0);
    parameter.gain.Kp0                      = 2.5;
    parameter.gain.Kp1                      = 2.5;
    parameter.gain.Kp2                      = 2.5;
    parameter.gain.Kv0                      = 3.0;
    parameter.gain.Kv1                      = 3.0;
    parameter.gain.Kv2                      = 3.0;
    parameter.gain.Kvi0                     = 0.0;
    parameter.gain.Kvi1                     = 0.0;
    parameter.gain.Kvi2                     = 0.0;
    parameter.gain.Kvd0                     = 0.0;
    parameter.gain.Kvd1                     = 0.0;
    parameter.gain.Kvd2                     = 0.0;
    parameter.gain.KAngR                    = 20.0;
    parameter.gain.KAngP                    = 20.0;
    parameter.gain.KAngY                    = 4.0;
    parameter.thr_map.accurate_thrust_model = false;
    parameter.thr_map.hover_percentage      = 0.5;
    return parameter;
  }

  ControllerOutput solveYaw(const double odom_yaw, const double command_yaw,
                            const double command_yaw_rate = 0.0)
  {
    static Parameters parameter = makeParameters();
    Controller        controller(parameter);
    OdomData          odometry;
    odometry.p.setZero();
    odometry.v.setZero();
    odometry.w.setZero();
    odometry.q = yawQuaternion(odom_yaw);
    ImuData imu;
    imu.q = odometry.q;
    imu.w.setZero();
    imu.a = Eigen::Vector3d(0.0, 0.0, parameter.gra);
    DesiredState desired(odometry);
    desired.yaw      = command_yaw;
    desired.yaw_rate = command_yaw_rate;
    ControllerOutput output;
    controller.update(desired, odometry, imu, output, 24.0);
    return output;
  }

  void expectShortestYaw(const double current_yaw, const double command_yaw)
  {
    const ControllerOutput output = solveYaw(current_yaw, command_yaw);
    const double expected = std::abs(shortestAngle(command_yaw - current_yaw));
    const double actual =
        physicalQuaternionAngle(yawQuaternion(current_yaw), output.q);
    EXPECT_NEAR(actual, expected, 1.0e-6);

    const Eigen::Vector3d body_z = output.q * Eigen::Vector3d::UnitZ();
    EXPECT_NEAR(body_z.x(), 0.0, 1.0e-6);
    EXPECT_NEAR(body_z.y(), 0.0, 1.0e-6);
    EXPECT_NEAR(body_z.z(), 1.0, 1.0e-6);
  }

  TEST(Px4ctrlYawContract, CrossesPositivePiByTwoDegrees)
  {
    expectShortestYaw(rad(179.0), rad(-179.0));
  }

  TEST(Px4ctrlYawContract, CrossesNegativePiByTwoDegrees)
  {
    expectShortestYaw(rad(-179.0), rad(179.0));
  }

  TEST(Px4ctrlYawContract, PreservesShortestPhysicalAttitudeUnderLag)
  {
    expectShortestYaw(rad(150.0), rad(-170.0));
    expectShortestYaw(rad(-150.0), rad(170.0));
  }

  TEST(Px4ctrlYawContract, FeedbackBodyRateUsesShortestQuaternionHemisphere)
  {
    Parameters               parameter = makeParameters();
    Controller               controller(parameter);
    const Eigen::Quaterniond current = yawQuaternion(rad(179.0));
    const Eigen::Quaterniond desired = yawQuaternion(rad(-179.0));
    const Eigen::Vector3d    body_rate =
        controller.computeFeedBackControlBodyrates(desired, current);
    EXPECT_GT(body_rate.z(), 0.0);
    EXPECT_LT(std::abs(body_rate.z()), rad(20.0));
  }

  TEST(Px4ctrlYawContract, FixedTargetBodyRateReversesAcrossPiByShortestPath)
  {
    Parameters               parameter = makeParameters();
    Controller               controller(parameter);
    const Eigen::Quaterniond target = yawQuaternion(rad(-179.0));

    const Eigen::Vector3d before_wrap =
        controller.computeFeedBackControlBodyrates(target,
                                                   yawQuaternion(rad(179.0)));
    const Eigen::Vector3d after_wrap =
        controller.computeFeedBackControlBodyrates(target,
                                                   yawQuaternion(rad(-178.0)));

    EXPECT_GT(before_wrap.z(), 0.0);
    EXPECT_LT(after_wrap.z(), 0.0);
    EXPECT_LT(std::abs(before_wrap.z()), rad(20.0));
    EXPECT_LT(std::abs(after_wrap.z()), rad(20.0));
  }

  TEST(Px4ctrlYawContract, StaticMockOdomKeepsFixedTargetOnShortestPath)
  {
    Parameters parameter = makeParameters();
    Controller controller(parameter);
    OdomData   odometry;
    odometry.p = Eigen::Vector3d(1.0, -2.0, 1.5);
    odometry.v.setZero();
    odometry.w.setZero();
    odometry.q = yawQuaternion(rad(179.0));
    ImuData imu;
    imu.q = odometry.q;
    imu.w.setZero();
    imu.a = Eigen::Vector3d(0.0, 0.0, parameter.gra);
    DesiredState desired(odometry);
    desired.yaw      = rad(-179.0);
    desired.yaw_rate = 0.0;

    for (int i = 0; i < 20; ++i)
    {
      ControllerOutput output;
      controller.update(desired, odometry, imu, output, 24.0);
      EXPECT_GT(output.bodyrates.z(), 0.0);
      EXPECT_LT(std::abs(output.bodyrates.z()), rad(20.0));
      const Eigen::Vector3d body_z = output.q * Eigen::Vector3d::UnitZ();
      EXPECT_NEAR(body_z.x(), 0.0, 1.0e-6);
      EXPECT_NEAR(body_z.y(), 0.0, 1.0e-6);
      EXPECT_NEAR(body_z.z(), 1.0, 1.0e-6);
    }
  }

  TEST(Px4ctrlYawContract, YawTargetRateSaturatesAtConfiguredMagnitude)
  {
    Parameters parameter = makeParameters();
    Controller controller(parameter);
    EXPECT_DOUBLE_EQ(controller.computeYawTargetBodyRate(rad(120.0), 0.0), 1.0);
    EXPECT_DOUBLE_EQ(controller.computeYawTargetBodyRate(rad(-120.0), 0.0),
                     -1.0);
  }

  TEST(Px4ctrlYawContract, YawTargetRateSlowsNearTargetAndStopsInDeadband)
  {
    Parameters parameter = makeParameters();
    Controller controller(parameter);
    EXPECT_NEAR(controller.computeYawTargetBodyRate(rad(10.0), 0.0), rad(10.0),
                1.0e-9);
    EXPECT_DOUBLE_EQ(controller.computeYawTargetBodyRate(rad(1.0), 0.0), 0.0);
  }

  TEST(Px4ctrlYawContract, YawTargetRateUsesShortestPathAcrossPi)
  {
    Parameters parameter = makeParameters();
    Controller controller(parameter);
    EXPECT_NEAR(controller.computeYawTargetBodyRate(rad(-178.0), rad(179.0)),
                rad(3.0), 1.0e-9);
    EXPECT_NEAR(controller.computeYawTargetBodyRate(rad(178.0), rad(-179.0)),
                rad(-3.0), 1.0e-9);
  }

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
