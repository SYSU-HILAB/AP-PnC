#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <nmpc_controller/tracking_controller.hpp>
#include <planner_core/trajectory.hpp>

#include "simple_sim/controller.hpp"

namespace simple_sim
{

  class NmpcController final : public Controller
  {
   public:
    NmpcController(
        std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
        nmpc::TrackingConfig config, std::filesystem::path trace_file = {});
    void          reset(const State &initial) override;
    ControlResult compute(const StepContext &, const State &,
                          const Feedback &) override;

   private:
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    nmpc::TrackingController                                  controller_;
    std::ofstream                                             trace_file_;
    void write_trace(const StepContext &,
                     const std::vector<planner::core::ReferencePoint> &);
  };

}  // namespace simple_sim
