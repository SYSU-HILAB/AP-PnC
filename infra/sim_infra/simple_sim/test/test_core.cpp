#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include "simple_sim/rate_loop.hpp"
#include "simple_sim/runner.hpp"

using namespace simple_sim;
namespace
{
  void check(bool condition, const char *message)
  {
    if (!condition)
      throw std::runtime_error(message);
  }
  void near(double value, double expected, double tolerance,
            const char *message)
  {
    check(std::abs(value - expected) <= tolerance, message);
  }
  template <class F>
  void throws(F f)
  {
    bool caught = false;
    try
    {
      f();
    }
    catch (const std::exception &)
    {
      caught = true;
    }
    check(caught, "expected failure was not raised");
  }
  class FixedController : public Controller
  {
   public:
    Command       command{9.81, Eigen::Vector3d::Zero()};
    bool          fail  = false;
    bool          slow  = false;
    int           calls = 0;
    void          reset(const State &) override { calls = 0; }
    ControlResult compute(const StepContext &ctx, const State &,
                          const Feedback &) override
    {
      check(ctx.index == calls, "controller order differs from tick order");
      ++calls;
      if (slow)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (fail)
        throw std::runtime_error("injected controller failure");
      return {command, 0};
    }
  };
  Actuation direct()
  {
    return [](const State &, const Command &c)
    { return Wrench{{0.0, 0.0, 2.0 * c.specific_force}, c.rates}; };
  }
  void dynamics()
  {
    Plant      plant(PlantConfig{});
    State      s;
    const auto zero = [](const State &, const Command &) { return Wrench{}; };
    const auto falling = plant.advance(s, {}, 0.1, zero);
    near(falling.velocity.z(), -0.981, 1e-12, "free-fall velocity");
    near(falling.position.z(), -0.04905, 1e-12, "free-fall position");
    const auto hovering =
        plant.advance(s, {9.81, Eigen::Vector3d::Zero()}, 0.1, direct());
    check(hovering.position.norm() < 1e-12, "hover equilibrium");
    const auto imu =
        plant.feedback(s, {9.81, Eigen::Vector3d::Zero()}, direct());
    near(imu.specific_force.z(), 9.81, 1e-12, "IMU must exclude gravity");
    near(plant.feedback(s, {}, zero).specific_force.norm(), 0.0, 1e-12,
         "free-fall IMU");
    // FLU body +Z rotated into world +X.
    s.attitude = Eigen::Quaterniond(
        Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitY()));
    const auto tilted =
        plant.advance(s, {10.0, Eigen::Vector3d::Zero()}, 0.01, direct());
    near(tilted.velocity.x(), 0.1, 1e-12, "body-to-world thrust rotation");
    near(tilted.velocity.z(), -0.0981, 1e-12, "gravity world axis");
    s                      = State{};
    s.angular_velocity.z() = 1.0;
    for (int i = 0; i < 100; ++i)
      s = plant.advance(s, {}, 0.01, zero);
    near(s.attitude.w(), std::cos(0.5), 1e-10,
         "quaternion Hamilton convention");
    near(s.attitude.z(), std::sin(0.5), 1e-10,
         "body-rate quaternion integration");
    // No ground/collision correction.
    check(falling.position.z() < 0.0,
          "plant unexpectedly clamps ground position");
  }
  void convergence()
  {
    PlantConfig cfg;
    cfg.inertia = Eigen::Vector3d::Ones();
    Plant      plant(cfg);
    const auto act = [](const State &, const Command &)
    { return Wrench{{0, 0, 0}, {0, 0, 1}}; };
    auto integrate = [&](double dt, int n)
    {
      State s;
      for (int i = 0; i < n; ++i)
        s = plant.advance(s, {}, dt, act);
      return s;
    };
    const auto a = integrate(0.2, 5), b = integrate(0.1, 10),
               fine = integrate(0.001, 1000);
    const double ea = a.attitude.angularDistance(fine.attitude);
    const double eb = b.attitude.angularDistance(fine.attitude);
    check(eb < ea / 8, "RK4 step-halving did not converge");
    near(fine.angular_velocity.z(), 1.0, 1e-12, "constant torque angular rate");
  }
  void rate_loop()
  {
    PlantConfig    p;
    RateLoopConfig c;
    c.max_specific_force = 15.0;
    c.max_moment         = Eigen::Vector3d::Constant(0.1);
    RateLoop   loop(p, c);
    const auto w = loop(State{}, {100.0, Eigen::Vector3d::Constant(100.0)});
    near(w.force.z(), 30.0, 1e-12,
         "specific force converted to N exactly once");
    near(w.moment.x(), 0.1, 1e-12, "moment saturation");
    near(loop(State{}, {-1.0, Eigen::Vector3d::Zero()}).force.z(), 0.0, 1e-12,
         "negative thrust saturation");
  }
  std::vector<State> run(bool slow)
  {
    FixedController controller;
    controller.slow    = slow;
    controller.command = {10.0, Eigen::Vector3d::Zero()};
    Runner runner({1000000, 10000000, 10}, Plant(PlantConfig{}), direct(),
                  controller);
    runner.reset(State{}, controller.command);
    std::vector<State> states;
    for (int i = 0; i < 10; ++i)
    {
      const auto r = runner.step();
      check(r.context.time_ns == i * 10000000LL, "integer tick time");
      check(r.context.index == i, "step index");
      near(r.before.position.z(), i == 0 ? 0.0 : states.back().position.z(),
           0.0, "before/after alignment");
      check(runner.time_ns() == (i + 1) * 10000000LL, "commit time");
      states.push_back(r.after);
    }
    check(runner.status() == RunStatus::Finished, "missing finished status");
    throws([&] { runner.step(); });
    return states;
  }
  void lockstep()
  {
    const auto fast = run(false), slow = run(true);
    for (std::size_t i = 0; i < fast.size(); ++i)
    {
      check(fast[i].position == slow[i].position,
            "wall time changed physical position");
      check(fast[i].attitude.coeffs() == slow[i].attitude.coeffs(),
            "wall time changed attitude");
    }
  }
  void lifecycle()
  {
    FixedController c;
    Runner runner({1000000, 10000000, 3}, Plant(PlantConfig{}), direct(), c);
    runner.reset(State{}, c.command);
    runner.pause();
    check(runner.time_ns() == 0, "pause advanced time");
    const auto first = runner.step();
    check(runner.status() == RunStatus::Paused, "single-step lost pause");
    runner.resume();
    runner.step();
    c.fail          = true;
    const auto last = runner.state();
    throws([&] { runner.step(); });
    check(runner.status() == RunStatus::Failed && runner.index() == 2,
          "failed solve committed tick");
    check(runner.state().position == last.position,
          "failed solve committed state");
    check(!runner.failure().empty(), "missing failure reason");
    c.fail = false;
    runner.reset(State{}, c.command);
    check(c.calls == 0 && runner.time_ns() == 0,
          "reset did not clear controller/time");
    const auto repeated = runner.step();
    check(first.after.position == repeated.after.position,
          "reset did not reproduce state");
    // Physics failure after multiple substeps also rolls back the whole control
    // step.
    ForceModel unstable = [](const State &s)
    {
      if (s.position.z() < -0.00001)
        throw std::runtime_error("injected force failure");
      return Wrench{};
    };
    c.command.specific_force = 0.0;
    Runner bad({1000000, 10000000, 1}, Plant(PlantConfig{}, unstable), direct(),
               c);
    bad.reset(State{}, c.command);
    throws([&] { bad.step(); });
    check(bad.time_ns() == 0 && bad.state().position.isZero(),
          "partial physics step was committed");
  }
  void validation()
  {
    throws([] { RunnerConfig{3000000, 10000000, 1}.validate(); });
    throws([] { RunnerConfig{0, 10000000, 1}.validate(); });
    throws(
        []
        {
          RunnerConfig{1, 10000000, std::numeric_limits<std::int64_t>::max()}
              .validate();
        });
    throws(
        []
        {
          PlantConfig c;
          c.mass = -1;
          c.validate();
        });
    throws(
        []
        {
          PlantConfig c;
          c.inertia.x() = 0;
          c.validate();
        });
    throws(
        []
        {
          State s;
          s.attitude.coeffs().setZero();
          validate_state(s);
        });
    throws(
        []
        {
          validate_command({std::numeric_limits<double>::quiet_NaN(),
                            Eigen::Vector3d::Zero()});
        });
    throws(
        []
        {
          Plant plant(PlantConfig{},
                      [](const State &)
                      {
                        Wrench w;
                        w.force.x() = std::numeric_limits<double>::quiet_NaN();
                        return w;
                      });
          plant.advance(State{}, {}, 0.01, direct());
        });
  }
}  // namespace
int main()
{
  const std::vector<std::pair<const char *, std::function<void()>>> tests = {
      {"dynamics/frames/IMU", dynamics},
      {"RK4 convergence", convergence},
      {"rate-loop limits", rate_loop},
      {"lockstep wall-time invariance", lockstep},
      {"pause/reset/transactional failure", lifecycle},
      {"configuration/NaN rejection", validation}};
  try
  {
    for (const auto &test : tests)
    {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
  return 0;
}
