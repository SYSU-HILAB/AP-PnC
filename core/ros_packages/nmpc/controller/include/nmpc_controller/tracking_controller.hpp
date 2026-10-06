#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "nmpc_solver.h"

namespace nmpc
{
  // Ring-buffer length for the delayed regressor; shared by config validation.
  inline constexpr int kAlphaHistory = 32;

  // Online estimate of the NMPC's kappa = d(cx)/d(alpha).
  //
  // The paper treats kappa as a fixed gain "tuned for performance"; this fits
  // it instead. It regresses the measured *level* cx on alpha, so the noisy
  // IMU-derived coefficient is never differentiated, and the fitted slope is
  // exactly the quantity the prediction model wants. The forgetting factor
  // keeps the fit local, matching the assumption that the simplified model only
  // has to hold over the short prediction horizon.
  //
  // The 2x2 normal equations use plain doubles: this target has no Eigen.
  class CxSlopeEstimator
  {
   public:
    // Restarts the recursion; `initial_slope` is the fallback used until the
    // measurements constrain it.
    void reset(double initial_slope, double slope_limit,
               double covariance_limit);
    // One (alpha, cx, speed^2) triple. Non-finite input is ignored.
    void         update(double alpha, double coefficient, double speed_squared);
    double       slope() const { return slope_; }
    double       intercept() const { return intercept_; }
    double       speed_coefficient() const { return speed_coefficient_; }
    std::int64_t updates() const { return updates_; }

   private:
    // Symmetric 3x3 inverse-covariance times gain, stored as six entries.
    double p00_ = 0.0, p01_ = 0.0, p02_ = 0.0;
    double p11_ = 0.0, p12_ = 0.0, p22_ = 0.0;
    double intercept_ = 0.0, slope_ = 0.0, speed_coefficient_ = 0.0;
    double forgetting_ = 0.995, slope_limit_ = 0.0, covariance_limit_ = 0.0;
    std::int64_t updates_ = 0, skipped_ = 0;
  };

  struct TrackingConfig
  {
    double horizon_s               = 1.0;
    double traj_res_s              = 0.1;
    double ctrl_frq                = 50.0;
    double initial_specific_thrust = 9.81;  // initialization only
    // SIGNED d(cx)/d(alpha): the initial value, and the fixed value when online
    // estimation is disabled.
    double cx_alpha_slope      = -0.044;
    bool   cx_slope_estimation = true;
    double cx_slope_forgetting = 0.995;      // ~200 samples of memory at 50 Hz
    double cx_slope_limit      = 0.2;        // |kappa| cap, 1/rad
    double cx_slope_covariance_limit = 1.0;  // bounds the RLS gain, no wind-up
    // High-frequency gate: the slope is only identifiable while alpha actually
    // moves. Quasi-static segments let the fit absorb lag and non-alpha
    // effects, which is what biased the estimate; samples below this
    // |d(alpha)/dt| are simply not fed to the recursion. Units: rad/s.
    double cx_slope_min_excitation = 0.05;
    // Motor time constant compensation: regress cx(t) on alpha(t - lag) instead
    // of alpha(t). The actuator body-X contribution lags the angle of attack by
    // the motor time constant, which biases a zero-lag fit. 0 disables the
    // delay.
    double cx_slope_lag_s = 0.0;
    void   validate() const;
  };

  using CostReference = std::array<double, NMPC_NY>;  // NMPC objective layout,
                                                      // not a trajectory type
  CostReference cost_reference(const std::array<double, 3> &position,
                               const std::array<double, 3> &velocity,
                               const std::array<double, 3> &yb);

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
    std::array<double, NMPC_NU>  command_derivative{};
    std::array<double, NMPC_NU>  integrated_command{};
    bool                         accepted = false;
  };

  // ROS-free controller shared by the flight node and simple_sim.
  class TrackingController
  {
   public:
    explicit TrackingController(TrackingConfig config);
    void
    reset();  // destroys/recreates the solver, including all warm-start state
    void           reset(const std::array<double, NMPC_NU> &initial_command);
    TrackingResult compute(const std::array<double, 13>     &state,
                           double                            specific_force_x,
                           const std::vector<CostReference> &horizon);
    const TrackingConfig &config() const { return config_; }
    double aero_coefficient_x() const { return aero_coefficient_x_; }
    // Slope actually handed to the solver this tick (estimated or configured).
    double cx_alpha_slope_active() const { return cx_alpha_slope_active_; }
    const CxSlopeEstimator &slope_estimator() const
    {
      return cx_slope_estimator_;
    }
    const TrackingTrace &trace() const { return trace_; }

   private:
    TrackingConfig                                                 config_;
    std::unique_ptr<nmpc_solver_t, decltype(&nmpc_solver_destroy)> solver_{
        nullptr, nmpc_solver_destroy};
    TrackingTrace               trace_;
    std::array<double, NMPC_NU> previous_command_{};
    double                      aero_coefficient_x_ = 0.0;
    CxSlopeEstimator            cx_slope_estimator_;
    double                      cx_alpha_slope_active_ = -0.044;
    // Ring buffer of alpha so the regressor can be delayed by cx_slope_lag_s.
    static constexpr int              kAlphaHistory = 32;
    std::array<double, kAlphaHistory> alpha_history_{};
    int                               alpha_write_         = 0;
    std::int64_t                      alpha_history_count_ = 0;
    double                            previous_alpha_      = 0.0;
    bool                              has_previous_alpha_  = false;
    std::int64_t                      excitation_skips_    = 0;
  };

}  // namespace nmpc
