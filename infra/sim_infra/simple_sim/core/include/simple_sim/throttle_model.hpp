#pragma once

#include <string>
#include <vector>

namespace simple_sim
{

  /**
   * @brief How a specific-force setpoint is turned into a normalized throttle.
   *
   * The plant consumes a thrust-axis specific force (N/kg) and a real flight
   * controller sends a normalized collective throttle in [0,1]. Two
   * realizations:
   *
   * - ``ideal_invert``: the exact static inverse of the modelled thrust curve.
   *   Perfect model knowledge, no online estimation, no correction of battery
   * sag, inflow or curve error.
   * - ``indi``: the incremental throttle law used on the vehicle
   *   (px4ctrl's ThrottleManager: single-parameter RLS of ``a = eta * u`` plus
   * an INDI-style increment). The effectiveness is estimated online from the
   *   measured thrust-axis specific force, so model error is absorbed.
   *
   * Both are stepped at the inner-loop rate (400 Hz), not at the physics rate
   * and not at the guidance rate: the physics runs at 2000 Hz and the guidance
   * at 50 Hz, and mixing those rates into one tick is how the estimand stops
   * being well defined.
   */
  struct ThrottleModelConfig
  {
    std::string realization = "ideal_invert";  // "ideal_invert" | "indi"

    /// Tick rate of the incremental law [Hz]. The INDI layer runs at 100 Hz by
    /// design: the actuator inner loop (throttle/mixer/ESC and the rate loop)
    /// runs at 400 Hz, the physics at 2000 Hz and the guidance at 50 Hz.
    double frq = 100.0;

    /// Exact thrust-axis specific force per unit throttle at hover [m/s^2].
    /// Used as the static inverse in ``ideal_invert`` and as the RLS seed in
    /// ``indi`` (the vehicle seeds from grav / hover_throttle).
    double eta_seed = 19.62;

    /// Admissible effectiveness bounds [m/s^2]. The lower bound is at least 1
    /// g.
    double eta_min = 9.81;
    double eta_max = 60.0;

    /// RLS: forgetting factor, initial variance, measurement variance.
    double forgetting_factor    = 0.999;
    double p_init               = 100.0;
    double measurement_variance = 0.2;

    /// RLS gating: innovation test ratio, variance validity, minimum throttle.
    double innovation_gate     = 3.0;
    double var_valid_threshold = 0.01;
    double min_throttle_update = 0.05;

    /// INDI gain on the specific-force error [1/s] and its bounds.
    double specific_force_p = 2.0;
    double throttle_min     = 0.05;
    double throttle_max     = 1.0;

    /// First-order low-pass time constants [s]; 0 disables the filter.
    double measurement_tau_s = 0.02;
    double setpoint_tau_s    = 0.02;

    /// Command/measurement pairing delay [s] and the low-pass on the applied
    /// command [s]. The measured force lags the command through the motor and
    /// ESC, so pairing simultaneous samples biases the effectiveness estimate.
    /// px4ctrl's INDI path uses the current command through this filter (lag 0)
    /// and its legacy path uses a delayed command queue; the lag is
    /// configurable so the two can be compared instead of assumed.
    double lag_s          = 0.0;
    double throttle_tau_s = 0.02;

    void validate() const;
  };

  /**
   * @brief Throttle realization and its online effectiveness estimate.
   *
   * Stateful and not thread-safe: one instance per plant. ``command`` is meant
   * to be called once per inner-loop tick (400 Hz); the caller owns the tick
   * and the rate, this class only owns the law.
   */
  /// Ring capacity for the delayed-command pairing: 256 ticks covers 2.5 s at
  /// the 100 Hz INDI rate, far more lag than any actuator.
  inline constexpr std::size_t kCommandRing = 256;

  class ThrottleModel
  {
   public:
    ThrottleModel(const ThrottleModelConfig &config, double seed_effectiveness);

    /**
     * @brief One inner-loop tick.
     *
     * @param specific_force_sp   Commanded thrust-axis specific force [m/s^2]
     * @param specific_force_meas Measured thrust-axis specific force [m/s^2]
     * @param dt_s               Inner-loop period [s]
     * @return Normalized throttle in [throttle_min, throttle_max]
     */
    double command(double specific_force_sp, double specific_force_meas,
                   double dt_s);

    /// Reset to the seeded effectiveness (takeoff, mode switch).
    void reset();

    double effectiveness() const { return eta_; }
    double throttle() const { return throttle_; }
    bool   estimate_valid() const { return valid_; }
    double innovation() const { return innovation_; }
    double innovation_test_ratio() const { return innovation_test_ratio_; }
    double variance() const { return covariance_; }
    const ThrottleModelConfig &config() const { return config_; }

   private:
    /// One scalar RLS recursion on the sample pair (throttle, specific force).
    void step_estimator(double throttle, double specific_force);

    double low_pass(double value, double previous, double tau_s,
                    double dt_s) const;

    ThrottleModelConfig config_;
    double              eta_;
    double              covariance_;
    double              innovation_            = 0.0;
    double              innovation_test_ratio_ = 0.0;
    bool                valid_                 = false;
    double              throttle_              = 0.0;
    double              measurement_lpf_       = 0.0;
    double              throttle_lpf_          = 0.0;
    std::vector<double> command_ring_  = std::vector<double>(kCommandRing, 0.0);
    std::size_t         command_write_ = 0;
    double              setpoint_lpf_  = 0.0;
    bool                filters_initialised_ = false;
  };

}  // namespace simple_sim
