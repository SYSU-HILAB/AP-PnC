#pragma once

#include <string>

#include "simple_sim/controller.hpp"
#include "simple_sim/plant.hpp"

namespace simple_sim
{

  enum class RunStatus
  {
    Ready,
    Running,
    Paused,
    Finished,
    Failed
  };

  using ActuatorTransition =
      std::function<State(const State &, const Command &)>;

  // Single-thread-owned. ROS/headless callers schedule step(), never integrate
  // themselves.
  class Runner
  {
   public:
    Runner(RunnerConfig config, Plant plant, Actuation actuation,
           Controller &controller, ActuatorTransition initialize = {},
           ActuatorTransition prepare = {});
    void reset(const State &initial, const Command &initial_command);
    void pause();
    void resume();
    StepRecord
    step();  // one entire control period; paused single-step stays paused
    const State    &state() const { return state_; }
    const Feedback &feedback() const { return feedback_; }
    std::int64_t    time_ns() const { return index_ * config_.control_dt_ns; }
    std::int64_t    index() const { return index_; }
    std::int64_t    control_dt_ns() const { return config_.control_dt_ns; }
    RunStatus       status() const { return status_; }
    const std::string &failure() const { return failure_; }

   private:
    RunnerConfig       config_;
    Plant              plant_;
    Actuation          actuation_;
    Controller        &controller_;
    ActuatorTransition initialize_, prepare_;
    State              state_;
    Feedback           feedback_;
    std::int64_t       index_  = 0;
    RunStatus          status_ = RunStatus::Failed;  // reset is mandatory
    std::string        failure_;
  };

}  // namespace simple_sim
