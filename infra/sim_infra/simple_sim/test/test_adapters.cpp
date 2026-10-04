#include <iostream>
#include <stdexcept>
#include <tailsitter_df/tailsitter_df.hpp>

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
    bool            getAeroWrench(const Eigen::Vector3d &v, Eigen::Vector3d &f,
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
  void aero_frames()
  {
    auto                   model = std::make_shared<FakeAero>();
    simple_sim::AeroConfig cfg;
    cfg.wind_world          = {1, 2, 3};
    auto              force = simple_sim::tailsitter_aero(model, cfg);
    simple_sim::State s;
    s.velocity   = {11, 22, 33};
    const auto w = force(s);
    check(model->input.isApprox(Eigen::Vector3d(30, -20, 10)),
          "FLU -> wing FRD velocity");
    check(w.force.isApprox(Eigen::Vector3d(3, -2, 1)), "wing FRD -> FLU force");
    check(w.moment.isApprox(Eigen::Vector3d(6, -5, 4)),
          "wing FRD -> FLU moment");
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
    tailsitter_df::Model::getInstance().setParameters(
        std::vector<double>(9, 0.0), 2.0);
    auto reference = std::make_shared<const planner::core::ReferenceTrajectory>(
        std::move(representation));
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
    aero_frames();
    std::cout << "PASS aerodynamic frame mapping/failure\n";
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
