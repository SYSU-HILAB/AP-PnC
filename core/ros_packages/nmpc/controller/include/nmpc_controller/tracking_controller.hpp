#pragma once

#include <array>
#include <memory>
#include <vector>

#include "nmpc_solver.h"

namespace nmpc
{

  struct TrackingConfig
  {
    double                horizon_s  = 1.0;
    double                traj_res_s = 0.1;
    double                ctrl_frq   = 50.0;
    double                u_hover    = 9.81;  // collective N/kg, not per rotor
    double                cz_rate    = 0.1;
    double                thrust_tau_s = 0.076;
    std::array<double, 3> rate_tau_s{0.1, 0.1, 0.1};
    void                  validate() const;
  };

  using CostReference = std::array<double, NMPC_NY>;  // NMPC objective layout,
                                                      // not a trajectory type
  CostReference cost_reference(const std::array<double, 3> &position,
                               const std::array<double, 3> &velocity,
                               const std::array<double, 3> &yb, double u_hover);

  struct TrackingResult
  {
    std::array<double, 3> rates{};
    double                specific_force = 0.0;
    int                   status         = 0;
  };

  struct TrackingTrace
  {
    std::vector<CostReference>   references;
    std::array<double, NMPC_NYN> terminal{};
    std::array<double, NMPC_NX>  state{};
    std::array<double, NMPC_NP>  parameters{};
    int                          status    = -1;
    bool                         submitted = false;
  };

  // ROS-free controller shared by the flight node and simple_sim.
  class TrackingController
  {
   public:
    explicit TrackingController(TrackingConfig config);
    void
    reset();  // destroys/recreates the solver, including all warm-start state
    TrackingResult        compute(const std::array<double, 13>     &state,
                                  double                            specific_force_x,
                                  const std::vector<CostReference> &horizon);
    const TrackingConfig &config() const { return config_; }
    double                cz() const { return cz_; }
    const TrackingTrace  &trace() const { return trace_; }

   private:
    TrackingConfig                                                 config_;
    std::unique_ptr<nmpc_solver_t, decltype(&nmpc_solver_destroy)> solver_{
        nullptr, nmpc_solver_destroy};
    TrackingTrace trace_;
    double        specific_force_estimate_    = 9.81;
    double        previous_specific_force_sp_ = 9.81;
    double        cz_                         = 0.0;
  };

}  // namespace nmpc
