#include <iostream>
#include <limits>
#include <stdexcept>
#include <tailsitter_df/flu_flatness.hpp>
#include <tailsitter_df/flu_reference.hpp>

#include "simple_sim/adapters/aero_model.hpp"
#include "simple_sim/adapters/controllers.hpp"
#include "simple_sim/rate_loop.hpp"
#include "simple_sim/runner.hpp"

namespace
{
  void check(bool c, const char *message)
  {
    if (!c)
      throw std::runtime_error(message);
  }
  class FakeAero : public aerodynamics::AerodynamicsInterface
  {
   public:
    Eigen::Vector3d input   = Eigen::Vector3d::Zero();
    bool            success = true;
    bool getAeroWrench(const Eigen::Vector3d &v, Eigen::Vector3d &f,
                       Eigen::Vector3d &m, double &alpha, double &beta) override
    {
      input = v;
      f     = {1, 2, 3};
      m     = {4, 5, 6};
      alpha = 0.2;
      beta  = 0.1;
      return success;
    }
  };
  void flu_flatness_jacobian()
  {
    // The analytic Jacobian must agree with central differences of the same
    // value map, otherwise the MINCO cost gradient is wrong.
    tailsitter_df::FluFlatnessParams params;
    const auto                       value = [&params](const Eigen::Vector3d &v,
                                                       const Eigen::Vector3d &a,
                                                       const Eigen::Vector3d &j)
    {
      const auto r = tailsitter_df::flu_flatness(v, a, j, params);
      Eigen::Matrix<double, 4, 1> out;
      out << r.specific_thrust, r.omega;
      return out;
    };
    const std::array<Eigen::Vector3d, 4> velocities{
        Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(6.0, -3.0, 1.0),
        Eigen::Vector3d(0.0, 10.0, 0.0), Eigen::Vector3d(5.0, 5.0, 2.0)};
    const Eigen::Vector3d acc(0.0, -5.0, 0.0);
    const Eigen::Vector3d jerk(1.0, 0.5, -0.5);
    double                worst = 0.0;
    for (const auto &v : velocities)
    {
      const auto flatness = tailsitter_df::flu_flatness(v, acc, jerk, params);
      check(flatness.nominal, "expected the nominal branch");
      const std::array<Eigen::Vector3d, 3> inputs{v, acc, jerk};
      const double                         step = 1e-6;
      for (int block = 0; block < 3; ++block)
        for (int axis = 0; axis < 3; ++axis)
        {
          Eigen::Vector3d plus = inputs[block], minus = inputs[block];
          plus[axis] += step;
          minus[axis] -= step;
          Eigen::Vector3d p[3] = {inputs[0], inputs[1], inputs[2]};
          Eigen::Vector3d m[3] = {inputs[0], inputs[1], inputs[2]};
          p[block]             = plus;
          m[block]             = minus;
          const Eigen::Matrix<double, 4, 1> central =
              (value(p[0], p[1], p[2]) - value(m[0], m[1], m[2])) / (2 * step);
          const double error =
              (central - flatness.jacobian.col(block * 3 + axis))
                  .cwiseAbs()
                  .maxCoeff();
          worst = std::max(worst, error);
          check(error < 1e-6,
                "FLU flatness Jacobian disagrees with finite differences");
        }
    }
    // Body-X residual of the aerodynamic force must be the only component.
    const Eigen::Vector3d v(10.0, 0.0, 0.0);
    const Eigen::Vector3d circle_acc(0.0, -5.0, 0.0);
    const auto r = tailsitter_df::flu_flatness(v, circle_acc,
                                               Eigen::Vector3d::Zero(), params);
    const Eigen::Vector3d eta =
        circle_acc + params.gravity * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d aero_body =
        r.rotation.transpose() * (eta - r.specific_thrust * r.rotation.col(2));
    check(aero_body(0) < 0.0 && std::abs(aero_body(1)) < 1e-9 &&
              std::abs(aero_body(2)) < 1e-9,
          "10 m/s circle must give a negative body-X aerodynamic force");
    std::cout << "FLU flatness: worst Jacobian/FD error = " << worst
              << ", circle body-X aero = " << aero_body(0) << "\n";
  }
  void aero_frames()
  {
    auto                   model = std::make_shared<FakeAero>();
    simple_sim::AeroConfig cfg;
    cfg.wind_world          = {1, 2, 3};
    auto              force = simple_sim::tailsitter_aero(model, cfg);
    simple_sim::State s;
    s.velocity   = {11, 22, 33};
    const auto w = force(s);
    check(model->input.isApprox(Eigen::Vector3d(10, 20, 30)),
          "air-relative velocity must remain body FLU");
    check(w.observation.valid && w.observation.alpha_rad == 0.2 &&
              w.observation.beta_rad == 0.1 &&
              w.observation.velocity_body.isApprox(model->input),
          "actual model angles/air velocity not preserved");
    check(w.force.isApprox(Eigen::Vector3d(1, 2, 3)),
          "model FLU force altered");
    check(w.moment.isApprox(Eigen::Vector3d(4, 5, 6)),
          "model FLU moment altered");
    s.attitude = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
    const auto expected_body =
        (s.attitude.conjugate() * (s.velocity - cfg.wind_world)).eval();
    force(s);
    check(model->input.isApprox(expected_body),
          "rotated body/world wind frame");
    s.attitude.coeffs() *= -1;
    force(s);
    check(model->input.isApprox(expected_body),
          "quaternion double cover changed physical frame");
    model->success = false;
    bool caught    = false;
    try
    {
      force(s);
    }
    catch (const std::exception &)
    {
      caught = true;
    }
    check(caught, "failed aero model silently accepted");
  }
  void se3_flu_force_balance()
  {
    class AxialDependentAero : public aerodynamics::AerodynamicsInterface
    {
     public:
      bool getAeroWrench(const Eigen::Vector3d &v, Eigen::Vector3d &f,
                         Eigen::Vector3d &m, double &alpha,
                         double &beta) override
      {
        std::tie(alpha, beta) = aerodynamics::air_angles(v);
        f                     = {-2.0 - 0.1 * v.z(), 0, 0};
        m.setZero();
        return true;
      }
    };
    const double          yaw = 0.7, gravity = 9.81, mass = 2.0, airspeed = 5.0;
    const Eigen::Vector3d direction(std::cos(yaw), std::sin(yaw), 0);
    Eigen::Matrix<double, 3, 6> polynomial =
        Eigen::Matrix<double, 3, 6>::Zero();
    polynomial.col(4) = 6.0 * direction;  // Airspeed 5 plus a 1 m/s tailwind.
    polynomial(2, 5)  = 2.0;
    TrajectoryRepresentation<5> representation;
    representation.emplace_back(2.0, polynomial);
    auto reference = std::make_shared<const planner::core::ReferenceTrajectory>(
        std::move(representation));
    auto                 model = std::make_shared<AxialDependentAero>();
    simple_sim::Se3Gains gains;
    gains.aero_flatness_feedforward = true;
    simple_sim::AeroConfig aero;
    aero.wind_world = direction;
    simple_sim::Se3Controller controller(reference, gravity, gains, model, mass,
                                         true, aero);
    simple_sim::State         state;
    state.position = reference->sample(0).p;
    state.velocity = 6.0 * direction;
    // -m*g*sin(theta) = -2 - .1*V*sin(theta), so the native FLU
    // force balance requires this positive body-Y rotation into forward tilt.
    const double tilt = std::asin(2.0 / (mass * gravity - .1 * airspeed));
    state.attitude    = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY());
    controller.reset(state);
    const auto command =
        controller.compute({0, 0, 20000000}, state, {}).command;
    check(command.rates.norm() < 1e-8,
          "SE3 model angle disagrees with its candidate FLU attitude");
    check(std::abs(command.specific_force - gravity * std::cos(tilt)) < 1e-9,
          "SE3 thrust-axis projection disagrees with FLU force balance");
  }
  void tracking()
  {
    // Use planner-owned continuous trajectory; no sim-owned reference
    // representation.
    Eigen::Matrix<double, 3, 6> polynomial =
        Eigen::Matrix<double, 3, 6>::Zero();
    polynomial(0, 4) = 1.0;  // p_x = t
    polynomial(2, 5) = 2.0;
    TrajectoryRepresentation<5> representation;
    representation.emplace_back(2.0, polynomial);
    auto reference = std::make_shared<const planner::core::ReferenceTrajectory>(
        std::move(representation));
    const auto endpoint = reference->sample(reference->duration());
    for (double t : {reference->duration(), reference->duration() + 0.1,
                     reference->duration() + 100.0})
    {
      const auto held = reference->sample(t);
      check(held.t == endpoint.t && held.p == endpoint.p &&
                held.v == endpoint.v && held.a == endpoint.a &&
                held.yb == endpoint.yb && held.omega == endpoint.omega &&
                held.rotation == endpoint.rotation &&
                held.thrust == endpoint.thrust &&
                held.flatness_fallback == endpoint.flatness_fallback,
            "terminal reference was extrapolated or modified");
    }
    for (double t : {-1.0, std::numeric_limits<double>::quiet_NaN()})
    {
      bool caught = false;
      try
      {
        reference->sample(t);
      }
      catch (const std::invalid_argument &)
      {
        caught = true;
      }
      check(caught, "invalid reference time accepted");
    }
    simple_sim::PlantConfig   plant;
    simple_sim::Se3Controller controller(reference, plant.gravity, {});
    simple_sim::Runner runner({1000000, 20000000, 50}, simple_sim::Plant(plant),
                              simple_sim::RateLoop(plant, {}), controller);
    const auto         first = reference->sample(0.0);
    const auto initial       = simple_sim::initial_state(first, plant.gravity);
    runner.reset(initial, {plant.gravity, Eigen::Vector3d::Zero()});
    for (int i = 0; i < 50; ++i)
    {
      const auto r   = runner.step();
      const auto ref = reference->sample(r.context.time_s() + 0.02);
      check((r.after.position - ref.p).norm() < 1e-6,
            "line tracking regression");
    }
    check(runner.status() == simple_sim::RunStatus::Finished,
          "tracking did not finish");
  }
}  // namespace
int main()
{
  try
  {
    const auto hover = tailsitter_df::flu_reference(Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());
    check(hover.rotation.isApprox(Eigen::Matrix3d::Identity()) &&
              std::abs(hover.specific_thrust - 9.81) < 1e-12,
          "+Z FLU hover reference");
    for (double speed : {0.0, 0.499999, 0.500001, 1.0, 6.0})
    {
      const auto r =
          tailsitter_df::flu_reference({speed, 0, 0}, {0, 0, 0}, {0, 0, 0});
      check(r.rotation.col(1).isApprox(Eigen::Vector3d::UnitY()),
            "yb flipped across speed guard");
      check(std::abs(r.rotation.determinant() - 1) < 1e-12,
            "reference reflection");
    }
    aero_frames();
    std::cout << "PASS aerodynamic frame mapping/failure\n";
    std::cout << "PASS single wing/FLU frame mapping\n";
    flu_flatness_jacobian();
    std::cout << "PASS FLU flatness Jacobian + circle aerodynamics\n";
    se3_flu_force_balance();
    tracking();
    std::cout << "PASS planner reference + SE3 line tracking\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
