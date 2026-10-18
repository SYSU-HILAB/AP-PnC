// Docker integration probe; caller supplies the authorized staged
// configuration.
#include <iostream>
#include <stdexcept>

#include "simple_sim/io/experiment.hpp"

namespace
{
  planner::core::ReferenceTrajectory hold(double x, double duration)
  {
    Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
    c.col(5)                      = Eigen::Vector3d(x, -4, 6);
    TrajectoryRepresentation<5> pieces;
    pieces.emplace_back(duration, c);
    return planner::core::ReferenceTrajectory(std::move(pieces));
  }
  void require(bool value, const char *message)
  {
    if (!value)
      throw std::runtime_error(message);
  }
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc != 2)
      throw std::invalid_argument("absolute staged config required");
    auto                   config = simple_sim::load_config(argv[1], "nmpc");
    simple_sim::Experiment online(
        config, std::make_shared<const planner::core::ReferenceTrajectory>(
                    hold(-1, 8)));
    simple_sim::Experiment fixed(
        config, std::make_shared<const planner::core::ReferenceTrajectory>(
                    hold(-1, 13)));
    for (int i = 0; i < 100; ++i)
    {
      if (i == 25)
      {
        const auto tick     = online.runner().index();
        const auto before   = online.runner().state();
        bool       rejected = false;
        try
        {
          online.append_reference(hold(3, 5));
        }
        catch (const std::invalid_argument &)
        {
          rejected = true;
        }
        require(rejected && online.trajectory().duration() == 8,
                "discontinuous append must not commit");
        online.append_reference(hold(-1, 5));
        require(online.runner().index() == tick,
                "append changed simulation time");
        require(
            (online.runner().state().motors.rpm - before.motors.rpm).norm() ==
                0,
            "append reset motors");
      }
      const auto a = online.step();
      const auto b = fixed.step();
      require((a.after.position - b.after.position).norm() == 0,
              "reference append changed hover position");
      require((a.after.velocity - b.after.velocity).norm() == 0,
              "reference append changed hover velocity");
      require((a.after.motors.rpm - b.after.motors.rpm).norm() == 0,
              "reference append reset actuator state");
      require(
          a.control.command.specific_force == b.control.command.specific_force,
          "reference append changed NMPC thrust state");
      require((a.control.command.rates - b.control.command.rates).norm() == 0,
              "reference append changed NMPC rate state");
    }
    online.finish("finished");
    fixed.finish("finished");
    bool rejected = false;
    try
    {
      online.append_reference(hold(-1, 5));
    }
    catch (const std::logic_error &)
    {
      rejected = true;
    }
    require(rejected, "finalized experiment accepted an append");
    std::cout << "reference append: identical plant/motor/NMPC state; "
                 "discontinuity and finalization rejection passed\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
