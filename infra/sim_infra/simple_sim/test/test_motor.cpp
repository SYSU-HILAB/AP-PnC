#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "simple_sim/motor_model.hpp"
#include "simple_sim/runner.hpp"

namespace
{
  void check(bool ok, const char *message)
  {
    if (!ok)
      throw std::runtime_error(message);
  }
  simple_sim::MotorModelConfig synthetic()
  {
    simple_sim::MotorModelConfig c;
    constexpr double             pi = 3.14159265358979323846;
    c.arms.setConstant(0.25);
    c.arm_angles << pi / 4, 3 * pi / 4, 5 * pi / 4, 7 * pi / 4;
    c.tilts << 0.2, -0.2, 0.2, -0.2;
    c.mixer << 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1;
    c.thrust_poly << -1.0, -0.5, 1.0;
    c.torque_poly << -2.0, 0.5, 1.0;
    c.esc_poly << 0, 10, -10000;
    c.tau         = 0.1;
    c.diameter    = 0.25;
    c.ct          = 1e-7;
    c.cq          = 1e-9;
    c.jm          = 1e-6;
    c.pwm_min     = 1000;
    c.pwm_max     = 2000;
    c.inner_dt_ns = 4000000;
    c.kp.setConstant(0.02);
    c.ki.setConstant(0.01);
    c.kd.setConstant(0.001);
    c.damping_wing << -0.02, -0.02, -0.02;
    return c;
  }
  class Fixed : public simple_sim::Controller
  {
   public:
    simple_sim::Command       command{12.0, Eigen::Vector3d::Zero()};
    void                      reset(const simple_sim::State &) override {}
    simple_sim::ControlResult compute(const simple_sim::StepContext &,
                                      const simple_sim::State &,
                                      const simple_sim::Feedback &) override
    {
      return {command, 0};
    }
  };
}  // namespace
int main()
{
  try
  {
    using namespace simple_sim;
    const MotorModel model(synthetic(), 2.0, 30.0, Eigen::Vector3d::Constant(8),
                           1000000);
    // The frame mapping now has one definition; this test uses the same helper
    // as the plant, so it verifies the real conversion, not a private copy.
    Eigen::Matrix3d frame;
    for (int axis = 0; axis < 3; ++axis)
      frame.col(axis) = wing_from_flu(Eigen::Vector3d::Unit(axis));
    check(
        frame.determinant() == 1.0 && (frame.transpose() * frame).isIdentity(),
        "frame must be a proper rotation");
    check((wing_from_flu(Eigen::Vector3d::UnitZ()))
                  .isApprox(Eigen::Vector3d::UnitX()) &&
              (flu_from_wing(wing_from_flu(Eigen::Vector3d(1, 2, 3)))
                   .isApprox(Eigen::Vector3d(1, 2, 3))),
          "thrust axis mapping / inverse");
    const auto trim =
        model.initialize(State{}, {9.81, Eigen::Vector3d::Zero()});
    const auto wrench = model.wrench(trim);
    check(std::abs(wrench.force.z() - 2 * 9.81) < 1e-10 &&
              wrench.moment.norm() < 1e-10,
          "static trim/mass");
    for (int i = 0; i < 100; ++i)
      check(model.wrench(trim).force.isApprox(wrench.force),
            "feedback mutated motor state");
    const auto mixed = model.mix({0.5, 0, 0, 0});
    check(mixed.fraction.isApprox(Eigen::Vector4d::Constant(0.5)),
          "balanced mixer");
    const auto saturated = model.mix({0.9, 1, 1, 1});
    check(saturated.lower && saturated.upper && saturated.fraction.allFinite(),
          "saturation flags");
    State lag = trim;
    lag.motors.rpm_sp.array() += 1000;
    const auto      initial_rpm = lag.motors.rpm;
    Plant           plant(PlantConfig{}, {},
                          [&model](const State &s) { return model.derivative(s); });
    const Actuation zero = [](const State &, const Command &)
    { return Wrench{}; };
    for (int k = 0; k < 10; ++k)
      lag = plant.advance(lag, {}, 0.001, zero);
    const double expected = 1000 * (1 - std::exp(-0.01 / 0.1));
    check(((lag.motors.rpm - initial_rpm).array() - expected).abs().maxCoeff() <
              1e-6,
          "RK4 first-order motor lag");
    Fixed controller;
    Plant failing(
        PlantConfig{},
        [](const State &s)
        {
          Wrench w;
          if (s.position.x() > 0.005)
            w.force.x() = std::numeric_limits<double>::quiet_NaN();
          return w;
        },
        [&model](const State &s) { return model.derivative(s); });
    Runner runner(
        {1000000, 20000000, 2}, std::move(failing),
        [&model](const State &s, const Command &) { return model.wrench(s); },
        controller, [&model](const State &s, const Command &u)
        { return model.initialize(s, u); },
        [&model](const State &s, const Command &u)
        { return model.prepare(s, u); });
    State initial;
    initial.velocity.x() = 1;
    runner.reset(initial, controller.command);
    const auto before = runner.state();
    bool       failed = false;
    try
    {
      runner.step();
    }
    catch (const std::exception &)
    {
      failed = true;
    }
    check(failed && runner.index() == 0 && runner.time_ns() == 0,
          "failure must not commit time");
    check(runner.state().motors.physics_ticks == before.motors.physics_ticks &&
              runner.state().motors.rpm.isApprox(before.motors.rpm) &&
              runner.state().motors.integral.isApprox(before.motors.integral),
          "motor/PID rollback failed");
    runner.reset(initial, controller.command);
    check(runner.state().motors.physics_ticks == 0, "actuator reset failed");
    std::cout
        << "PASS "
           "frame/trim/mixer/pure-feedback/lag/transactional-rollback/reset\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
