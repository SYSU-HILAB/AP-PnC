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

    /**
     * @brief Swap in a replanned leg without discontinuity.
     *
     * `trajectory`'s own time axis starts at 0; `time_offset` is the sim
     * time at which that 0 occurred, so `sample(t)` calls become
     * `sample(t - time_offset)`. Unused by the default (box_sfc) path, which
     * never calls this and keeps `time_offset_ == 0` for its whole run.
     */
    void set_trajectory(
        std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
        double                                                    time_offset);

   private:
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    double                   time_offset_ = 0.0;
    nmpc::TrackingController controller_;
    std::ofstream            trace_file_;
    void write_trace(const StepContext &,
                     const std::vector<planner::core::ReferencePoint> &);
  };

}  // namespace simple_sim
