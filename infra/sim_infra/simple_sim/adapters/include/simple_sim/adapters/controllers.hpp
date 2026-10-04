#pragma once

#include <memory>
#include <planner_core/trajectory.hpp>

#include "simple_sim/controller.hpp"

namespace simple_sim
{

  struct Se3Gains
  {
    Eigen::Vector3d position = Eigen::Vector3d(4.0, 4.0, 4.0);
    Eigen::Vector3d velocity = Eigen::Vector3d(3.0, 3.0, 3.0);
    Eigen::Vector3d attitude = Eigen::Vector3d(6.0, 6.0, 6.0);
  };

  class Se3Controller final : public Controller
  {
   public:
    Se3Controller(
        std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
        double gravity, Se3Gains gains);
    void          reset(const State &initial) override;
    ControlResult compute(const StepContext &, const State &,
                          const Feedback &) override;

   private:
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    double                                                    gravity_;
    Se3Gains                                                  gains_;
  };

  State initial_state(const planner::core::ReferencePoint &reference,
                      double                               gravity);

}  // namespace simple_sim
