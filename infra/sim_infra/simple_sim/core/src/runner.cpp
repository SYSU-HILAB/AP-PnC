#include "simple_sim/runner.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace simple_sim
{
  Runner::Runner(RunnerConfig config, Plant plant, Actuation actuation,
                 Controller &controller, ActuatorTransition initialize,
                 ActuatorTransition prepare)
      : config_(config),
        plant_(std::move(plant)),
        actuation_(std::move(actuation)),
        controller_(controller),
        initialize_(std::move(initialize)),
        prepare_(std::move(prepare))
  {
    config_.validate();
    if (!actuation_)
      throw std::invalid_argument("missing actuation");
  }
  void Runner::reset(const State &initial, const Command &initial_command)
  {
    status_ = RunStatus::Failed;
    try
    {
      validate_state(initial);
      validate_command(initial_command);
      const State prepared =
          initialize_ ? initialize_(initial, initial_command) : initial;
      validate_state(prepared);
      controller_.reset(prepared);
      const Feedback feedback =
          plant_.feedback(prepared, initial_command, actuation_);
      state_    = prepared;
      feedback_ = feedback;
      index_    = 0;
      failure_.clear();
      status_ = RunStatus::Ready;
    }
    catch (const std::exception &e)
    {
      failure_ = e.what();
      throw;
    }
  }
  void Runner::pause()
  {
    if (status_ == RunStatus::Ready || status_ == RunStatus::Running)
      status_ = RunStatus::Paused;
  }
  void Runner::resume()
  {
    if (status_ == RunStatus::Paused || status_ == RunStatus::Ready)
      status_ = RunStatus::Running;
  }
  StepRecord Runner::step()
  {
    if (status_ == RunStatus::Finished || status_ == RunStatus::Failed)
      throw std::logic_error(
          "cannot advance a finished/failed runner; reset required");
    const bool paused = status_ == RunStatus::Paused;
    try
    {
      StepRecord r;
      r.context         = {index_, time_ns(), config_.control_dt_ns};
      r.before          = state_;
      r.feedback_before = feedback_;
      const auto start  = std::chrono::steady_clock::now();
      r.control         = controller_.compute(r.context, state_, feedback_);
      r.solve_time_ms   = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
      validate_command(r.control.command);
      if (r.control.solver_status != 0 && r.control.solver_status != 2)
        throw std::runtime_error(
            "controller returned unaccepted solver status=" +
            std::to_string(r.control.solver_status));
      // Advance a local candidate. Any failure leaves the last committed
      // state/time intact.
      State        candidate = state_;
      const auto   substeps  = config_.control_dt_ns / config_.physics_dt_ns;
      const double dt = static_cast<double>(config_.physics_dt_ns) * 1e-9;
      for (std::int64_t i = 0; i < substeps; ++i)
      {
        if (prepare_)
          candidate = prepare_(candidate, r.control.command);
        candidate =
            plant_.advance(candidate, r.control.command, dt, actuation_);
      }
      r.after = candidate;
      r.feedback_after =
          plant_.feedback(candidate, r.control.command, actuation_);
      state_    = candidate;
      feedback_ = r.feedback_after;
      ++index_;
      status_ = index_ == config_.steps
                    ? RunStatus::Finished
                    : (paused ? RunStatus::Paused : RunStatus::Running);
      return r;
    }
    catch (const std::exception &e)
    {
      status_  = RunStatus::Failed;
      failure_ = e.what();
      throw;
    }
  }
}  // namespace simple_sim
