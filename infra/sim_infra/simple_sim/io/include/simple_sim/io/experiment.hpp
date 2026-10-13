#pragma once

#include <fstream>
#include <functional>
#include <gcopter/voxel_map.hpp>
#include <memory>
#include <optional>
#include <planner_core/planner.hpp>
#include <vector>

#include "simple_sim/io/config_loader.hpp"
#include "simple_sim/runner.hpp"

namespace simple_sim
{

  // Application composition and recording, shared by headless and ROS entry
  // points.
  class Experiment
  {
   public:
    explicit Experiment(
        RuntimeConfig                                             config,
        std::shared_ptr<const planner::core::ReferenceTrajectory> reference =
            {});
    ~Experiment();
    Experiment(const Experiment &)            = delete;
    Experiment &operator=(const Experiment &) = delete;
    StepRecord  step();
    void    finish(const std::string &status, const std::string &reason = {});
    Runner &runner() { return *runner_; }
    const planner::core::ReferenceTrajectory &trajectory() const
    {
      return *trajectory_;
    }
    const std::filesystem::path &output_dir() const { return output_dir_; }
    const RuntimeConfig         &config() const { return config_; }
    std::shared_ptr<const planner::core::ReferenceTrajectory> reference_handle()
        const
    {
      return trajectory_;
    }
    bool finalized() const { return finished_; }

    /**
     * @brief Ingest the ground-truth map once, for `corridor_source: map_sfc`.
     *
     * No-op (logged) unless `problem_formulation.corridor_source == map_sfc`
     * in the planning config this experiment loaded; the default (box_sfc)
     * path never consults a map. Only the first call builds the occupancy
     * grid (the ground-truth map is static per scene); later calls are
     * ignored.
     *
     * @param points World-frame map points (e.g. from `/lidar_sim/map`)
     */
    void ingest_map(const std::vector<Eigen::Vector3d> &points);

   private:
    RuntimeConfig                                             config_;
    std::filesystem::path                                     output_dir_;
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    std::unique_ptr<Controller>                               controller_;
    std::unique_ptr<Runner>                                   runner_;
    std::ofstream                                             log_;

    // --- map_sfc continuous replanning (unused on the box_sfc path) ---
    planner::core::ProblemConfig          planning_;
    std::optional<planner::core::Planner> planner_;
    std::function<void(
        std::shared_ptr<const planner::core::ReferenceTrajectory>, double)>
                                       apply_replan_;
    std::optional<voxel_map::VoxelMap> map_;
    bool                               map_degenerate_logged_ = false;
    // Set by ingest_map(); forces one replan on the next step() regardless
    // of the current leg's remaining duration, since the map arrived mid-leg.
    bool force_replan_ = false;
    // Sim time at which `trajectory_`'s own t=0 occurred; subtracted before
    // every sample() so a replanned leg splices without a discontinuity.
    double trajectory_time_offset_ = 0.0;
    // Sim time at which the current leg's terminal state is reached; replan
    // once the remaining time drops below
    // `planning_.problem.map.replan_margin_s`.
    double current_leg_end_time_s_ = 0.0;
    // Cooldown so a degenerate corridor (no path/empty hPolys) cannot be
    // retried every control step.
    double next_replan_attempt_s_ = 0.0;
    // Index into mission_route() of the waypoint `trajectory_` ends at.
    std::size_t mission_waypoint_index_ = 0;
    // Divergence guard only (logged, see replan_if_due): never fed into the
    // next leg's iniState, which always comes from the predicted terminal
    // state, not raw odometry.
    bool   drift_warning_logged_ = false;
    void   replan_if_due(double                 sim_time_s,
                         const Eigen::Vector3d &actual_position);
    double squared_position_error_ = 0.0;
    double squared_velocity_error_ = 0.0;
    double max_position_error_     = 0.0;
    // Sideslip over the committed window: the body-frame lateral velocity
    // relative to the flight path, max |beta| and its RMS.
    double max_sideslip_rad_ = 0.0;
    // Sideslip samples, gated at the end on 0.8 of the run's own maximum
    // airspeed: the transition and hover part of a trajectory is not what this
    // metric is for.
    std::vector<double> sideslip_rad_samples_;
    std::vector<double> sideslip_speed_samples_;
    double              squared_sideslip_rad_ = 0.0;
    std::int64_t        samples_              = 0;
    std::int64_t        status2_count_        = 0;
    bool                finished_             = false;
    void                metrics_sample(const State &state, double t);
  };

}  // namespace simple_sim
