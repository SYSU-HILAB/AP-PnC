#include <iostream>
#include <planner_core/planner.hpp>
#include <stdexcept>

namespace
{
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
      throw std::invalid_argument("absolute planning config required");
    planner::core::ProblemConfig config;
    config.load(argv[1]);
    config.problem.map.speed_controlled = true;
    config.problem.map.sfc_progress_m   = 18;
    config.problem.map.sfc_range_m      = 9;
    config.problem.cost.v_max           = 10;
    config.problem.cost.acc_max         = 12;
    planner::core::Planner planner(config);
    voxel_map::VoxelMap    map(Eigen::Vector3i(160, 160, 60),
                               Eigen::Vector3d(-80.5, -80.5, -10.5), 1.0);
    // Nonempty global map with an empty local road: exercise the full curved
    // FIRI cover. Shortcutting the cover used to discard the arc's outside.
    map.setOccupied(Eigen::Vector3d(-70, -70, 18));
    map.dilate(1);
    Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), goal = start;
    start.col(0) = Eigen::Vector3d(0, -20, 18);
    start.col(1) = Eigen::Vector3d(10, 0, 0);
    goal.col(0)  = Eigen::Vector3d(20, 0, 18);
    goal.col(1)  = Eigen::Vector3d(0, 10, 0);
    auto arc     = planner.plan_corridor(start, goal, map);
    require(arc.has_value(), "constant-speed corner failed");
    double lo = 100, hi = 0;
    for (int i = 0; i <= 1000; ++i)
    {
      const auto rp = arc->sample(arc->duration() * i / 1000);
      lo            = std::min(lo, rp.v.norm());
      hi            = std::max(hi, rp.v.norm());
      require(std::abs(rp.v.norm() - 10) < 0.03,
              "corner brakes or accelerates away from cruise speed");
    }
    require((arc->sample(0).v - start.col(1)).norm() < 1e-9,
            "initial velocity changed");
    require((arc->sample(arc->duration()).v - goal.col(1)).norm() < 1e-9,
            "terminal velocity changed");
    require(arc->sample(0).a.norm() < 1e-9 &&
                arc->sample(arc->duration()).a.norm() < 1e-9,
            "PVA splice is not C2");
    const auto obstacle = arc->sample(arc->duration() / 2).p;
    map.setOccupied(obstacle);
    map.dilate(1);
    require(!planner.plan_corridor(start, goal, map),
            "colliding fly-through connector accepted");
    voxel_map::VoxelMap empty(Eigen::Vector3i(160, 160, 60),
                              Eigen::Vector3d(-80.5, -80.5, -10.5), 1.0);
    start.setZero();
    goal.setZero();
    start.col(0)      = Eigen::Vector3d(-56, 0, 18);
    goal.col(0)       = Eigen::Vector3d(0, 0, 18);
    goal.col(1)       = Eigen::Vector3d(10, 0, 0);
    auto acceleration = planner.plan_corridor(start, goal, empty);
    require(acceleration.has_value(), "hover-to-cruise failed");
    double previous = 0;
    for (int i = 0; i <= 100; ++i)
    {
      const double speed =
          acceleration->sample(acceleration->duration() * i / 100).v.norm();
      require(speed >= previous - 1e-9 && speed <= 10 + 1e-9,
              "launch speed is not monotone");
      previous = speed;
    }
    // A ring must retain centripetal acceleration through every splice.
    // Resetting it to zero creates curvature pulses even at constant speed.
    const double radius = 45.0, angle = M_PI / 8;
    start.setZero();
    goal.setZero();
    start.col(0) = Eigen::Vector3d(0, -radius, 18);
    start.col(1) = Eigen::Vector3d(10, 0, 0);
    start.col(2) = Eigen::Vector3d(0, 100 / radius, 0);
    goal.col(0)  = Eigen::Vector3d(radius * std::sin(angle),
                                   -radius * std::cos(angle), 18);
    goal.col(1) =
        Eigen::Vector3d(10 * std::cos(angle), 10 * std::sin(angle), 0);
    goal.col(2) = Eigen::Vector3d(-100 / radius * std::sin(angle),
                                  100 / radius * std::cos(angle), 0);
    auto ring   = planner.plan_corridor(start, goal, empty);
    require(ring.has_value(), "centripetal ring connection failed");
    require((ring->sample(0).a - start.col(2)).norm() < 1e-9 &&
                (ring->sample(ring->duration()).a - goal.col(2)).norm() < 1e-9,
            "ring acceleration was reset at a splice");
    for (int i = 0; i <= 1000; ++i)
    {
      const auto rp = ring->sample(ring->duration() * i / 1000);
      require(std::abs(rp.v.norm() - 10) < 0.001,
              "ring reference has speed ripple");
      require(std::abs(rp.p.head<2>().norm() - radius) < 0.001,
              "ring reference leaves the intended circle");
    }
    std::cout << "fly-through corner speed " << lo << " .. " << hi
              << " m/s; PVA, launch and collision refusal passed\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
