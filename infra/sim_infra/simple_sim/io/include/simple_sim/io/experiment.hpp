#pragma once

#include <fstream>
#include <memory>

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

   private:
    RuntimeConfig                                             config_;
    std::filesystem::path                                     output_dir_;
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    std::unique_ptr<Controller>                               controller_;
    std::unique_ptr<Runner>                                   runner_;
    std::ofstream                                             log_;
    double       squared_position_error_ = 0.0;
    double       squared_velocity_error_ = 0.0;
    double       max_position_error_     = 0.0;
    std::int64_t samples_                = 0;
    std::int64_t status2_count_          = 0;
    bool         finished_               = false;
    void         metrics_sample(const State &state, double t);
  };

}  // namespace simple_sim
