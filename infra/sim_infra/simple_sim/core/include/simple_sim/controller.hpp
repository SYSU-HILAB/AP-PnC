#pragma once

#include "simple_sim/types.hpp"

namespace simple_sim
{

  class Controller
  {
   public:
    virtual ~Controller()                    = default;
    virtual void reset(const State &initial) = 0;
    // A controller owns access to the planner's trajectory. Sim defines no
    // reference type.
    virtual ControlResult compute(const StepContext &context,
                                  const State       &state,
                                  const Feedback    &feedback) = 0;
  };

}  // namespace simple_sim
