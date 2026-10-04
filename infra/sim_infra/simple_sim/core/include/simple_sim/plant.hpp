#pragma once

#include <functional>

#include "simple_sim/config.hpp"
#include "simple_sim/types.hpp"

namespace simple_sim
{

  // Evaluated at every RK4 stage. Must be deterministic and side-effect free.
  using ForceModel = std::function<Wrench(const State &)>;
  using Actuation  = std::function<Wrench(const State &, const Command &)>;

  class Plant
  {
   public:
    explicit Plant(PlantConfig config, ForceModel aerodynamic = {});
    State    advance(const State &state, const Command &command, double dt,
                     const Actuation &actuation) const;
    Feedback feedback(const State &state, const Command &command,
                      const Actuation &actuation) const;
    const PlantConfig &config() const { return config_; }

   private:
    PlantConfig config_;
    ForceModel  aerodynamic_;
    Wrench      aero(const State &state) const;
  };

}  // namespace simple_sim
