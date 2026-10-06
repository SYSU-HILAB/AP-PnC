#pragma once

#include "simple_sim/config.hpp"
#include "simple_sim/types.hpp"

namespace simple_sim
{

  // Stateless P rate loop with gyroscopic compensation and explicit saturation.
  // Updated throughout each physics step; never overwrites the body's rates.
  class RateLoop
  {
   public:
    RateLoop(PlantConfig plant, RateLoopConfig config);
    Wrench operator()(const State &state, const Command &command) const;

   private:
    PlantConfig    plant_;
    RateLoopConfig config_;
  };

}  // namespace simple_sim
