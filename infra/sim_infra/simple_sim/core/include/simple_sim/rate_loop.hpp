#pragma once

#include "simple_sim/config.hpp"
#include "simple_sim/types.hpp"

namespace simple_sim
{

  // Ideal inner loop: a P rate loop with gyroscopic compensation and explicit
  // saturation, plus a first-order lag on the collective specific force. The
  // rate loop is stateless; the thrust lag lives in the plant state, so the
  // actuation is evaluated the same way at every RK4 stage.
  //
  // Updated throughout each physics step; never overwrites the body's rates.
  class RateLoop
  {
   public:
    // `filtered_thrust` selects the first-order specific-force loop: when true
    // the applied thrust is the plant state (drive it through derivative()),
    // otherwise the command is applied directly as before.
    RateLoop(PlantConfig plant, RateLoopConfig config,
             bool filtered_thrust = false);
    Wrench operator()(const State &state, const Command &command) const;
    // d(collective specific force)/dt for the plant state.
    double derivative(const State &state, const Command &command) const;

   private:
    PlantConfig    plant_;
    RateLoopConfig config_;
    bool           filtered_thrust_ = false;
  };

}  // namespace simple_sim
