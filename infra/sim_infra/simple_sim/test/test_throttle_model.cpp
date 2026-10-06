// Tests for the throttle realization: the exact static inverse ("ideal_invert")
// and the online incremental law ("indi", px4ctrl's ThrottleManager).
//
// Plain main, like the other simple_sim tests: no gtest dependency.

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "simple_sim/throttle_model.hpp"

namespace
{
  using simple_sim::ThrottleModel;
  using simple_sim::ThrottleModelConfig;

  constexpr double kInnerDt    = 1.0 / 400.0;  // inner loop: 400 Hz
  constexpr double kHoverForce = 9.81;

  int failures = 0;

  void check(bool ok, const char *what)
  {
    if (!ok)
    {
      std::printf("FAIL: %s\n", what);
      ++failures;
    }
  }

  void near(double value, double expected, double tolerance, const char *what)
  {
    if (!(std::abs(value - expected) <= tolerance))
    {
      std::printf("FAIL: %s: got %.12g, expected %.12g (+/- %.3g)\n", what,
                  value, expected, tolerance);
      ++failures;
    }
  }

  ThrottleModelConfig indi_config()
  {
    ThrottleModelConfig config;
    config.realization       = "indi";
    config.eta_seed          = 2.0 * kHoverForce;  // deliberately wrong seed
    config.eta_min           = kHoverForce;
    config.eta_max           = 60.0;
    config.measurement_tau_s = 0.0;  // deterministic
    config.throttle_tau_s = 0.0;  // the pairing filter is not under test here
    config.setpoint_tau_s = 0.0;
    config.measurement_variance = 1.0e-4;
    config.var_valid_threshold  = 1.0;
    return config;
  }

  /// The plant is a = eta_true * throttle; returns the achieved specific force.
  double simulate(const ThrottleModelConfig &config, double eta_true,
                  double specific_force_sp, int ticks)
  {
    ThrottleModel model(config, kHoverForce);
    double        last = 0.0;
    for (int i = 0; i < ticks; ++i)
    {
      const double measured = eta_true * last;
      last = model.command(specific_force_sp, measured, kInnerDt);
    }
    return eta_true * last;
  }
}  // namespace

int main()
{
  // ideal_invert is the exact static inverse of the modelled effectiveness.
  {
    ThrottleModelConfig config;
    config.realization = "ideal_invert";
    ThrottleModel model(config, kHoverForce);
    double        throttle = 0.0;
    for (int i = 0; i < 10; ++i)
      throttle =
          model.command(8.0, 0.0, kInnerDt);  // below the modelled ceiling
    near(throttle, 8.0 / kHoverForce, 1e-12, "ideal_invert static inverse");
    near(model.effectiveness(), kHoverForce, 1e-12,
         "ideal_invert keeps the model");
    check(!model.estimate_valid(), "ideal_invert does not claim an estimate");
  }

  // With a mismatched plant the static inverse cannot absorb anything: the
  // achieved force stays off by exactly the plant/model ratio.
  {
    ThrottleModelConfig config;
    config.realization = "ideal_invert";
    near(simulate(config, 2.0 * kHoverForce, 5.0, 400), 10.0, 1e-9,
         "ideal_invert cannot absorb a 2x plant mismatch");
    // Asking for more than the modelled maximum saturates instead of exceeding
    // the throttle bounds: 2 * eta_seed * throttle_max.
    near(simulate(config, 2.0 * kHoverForce, 12.0, 400), 2.0 * kHoverForce,
         1e-9, "ideal_invert saturates at the throttle ceiling");
  }

  // INDI converges onto the setpoint despite a wrong seed.
  {
    // 2 s of inner-loop ticks. The increment obeys the configured INDI gain, so
    // the residual closes over a few time constants rather than instantly.
    near(simulate(indi_config(), 19.0, 12.0, 800), 12.0, 0.01,
         "indi converges with a 3% seed error");
    near(simulate(indi_config(), 19.0, 12.0, 200), 12.0, 0.25,
         "indi closes most of the error within half a second");
  }

  // INDI absorbs a large mismatch: 1.5x stronger plant than the seed. The
  // static inverse would have produced 18 m/s^2 here, so the assertion that
  // matters is the order-of-magnitude improvement, not a tight tolerance on a
  // law whose residual closes at the configured INDI gain.
  {
    const ThrottleModelConfig config = indi_config();
    const double achieved = simulate(config, 1.5 * config.eta_seed, 12.0, 800);
    near(achieved, 12.0, 0.2, "indi absorbs a 50% plant mismatch");
    check(achieved < 13.0,
          "indi beats the 18 m/s^2 a static inverse would give");
  }

  // The increment is explicit in the inner-loop period: the same error over
  // twice the period is twice the increment. A wrong tick rate would silently
  // rescale the closed-loop gain.
  {
    const ThrottleModelConfig config = indi_config();
    ThrottleModel             one_dt(config, kHoverForce);
    ThrottleModel             two_dt(config, kHoverForce);
    const double              one_0 = one_dt.command(12.0, 0.0, kInnerDt);
    const double              two_0 = two_dt.command(12.0, 0.0, kInnerDt);
    const double              one_1 = one_dt.command(12.0, 0.0, kInnerDt);
    const double              two_1 = two_dt.command(12.0, 0.0, 2.0 * kInnerDt);
    check(one_1 > one_0, "indi steps up towards the setpoint");
    check(std::abs(one_1 - one_0) > 0.0, "indi increment is not zero");
    if (one_1 > one_0)
      near((two_1 - two_0) / (one_1 - one_0), 2.0, 1e-9,
           "indi increment scales with dt");
  }

  // Invalid configurations are rejected instead of silently misbehaving.
  {
    ThrottleModelConfig config;
    config.realization = "thrust_curve";
    bool threw         = false;
    try
    {
      config.validate();
    }
    catch (const std::invalid_argument &)
    {
      threw = true;
    }
    check(threw, "unknown realization is rejected");

    ThrottleModelConfig bad;
    bad.eta_max = bad.eta_min - 1.0;
    threw       = false;
    try
    {
      bad.validate();
    }
    catch (const std::invalid_argument &)
    {
      threw = true;
    }
    check(threw, "inverted effectiveness bounds are rejected");
  }

  if (failures != 0)
  {
    std::printf("throttle model: %d check(s) failed\n", failures);
    return 1;
  }
  std::printf("throttle model: all checks passed\n");
  return 0;
}
