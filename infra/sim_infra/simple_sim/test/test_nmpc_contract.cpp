#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "nmpc_controller/tracking_controller.hpp"

// Contract tests use an explicit fake C solver, not a pretend acados
// integration.
struct nmpc_solver
{
  int unused = 0;
};
namespace
{
  int                         creates = 0, destroys = 0, status = 0;
  double                      compiled_dt = 0.1;
  std::array<double, NMPC_NX> last_state{};
  std::array<double, NMPC_NY> first_reference{};
  bool                        emit_nan = false;
  void                        check(bool c, const char *message)
  {
    if (!c)
      throw std::runtime_error(message);
  }
  template <class F>
  void throws(F f)
  {
    bool caught = false;
    try
    {
      f();
    }
    catch (const std::exception &)
    {
      caught = true;
    }
    check(caught, "expected controller failure");
  }
}  // namespace
extern "C"
{
  nmpc_solver_t *nmpc_solver_create()
  {
    ++creates;
    return new nmpc_solver;
  }
  void nmpc_solver_destroy(nmpc_solver_t *s)
  {
    if (s)
    {
      ++destroys;
      delete s;
    }
  }
  int nmpc_solver_horizon(const nmpc_solver_t *)
  {
    return 10;
  }
  double nmpc_solver_stage_dt(const nmpc_solver_t *, int)
  {
    return compiled_dt;
  }
  void nmpc_solver_set_state(nmpc_solver_t *, const double x[NMPC_NX])
  {
    std::copy_n(x, NMPC_NX, last_state.begin());
  }
  void nmpc_solver_set_ref_stage(nmpc_solver_t *, int stage,
                                 const double y[NMPC_NY], const double[NMPC_NP])
  {
    if (stage == 0)
      std::copy_n(y, NMPC_NY, first_reference.begin());
  }
  void nmpc_solver_set_ref_terminal(nmpc_solver_t *, const double[NMPC_NYN]) {}
  int  nmpc_solver_solve(nmpc_solver_t *, double du[NMPC_NU], double x[NMPC_NX])
  {
    std::fill_n(du, NMPC_NU, 0.0);
    std::fill_n(x, NMPC_NX, 0.0);
    x[6] = 0.1;
    x[7] = 0.2;
    x[8] = 0.3;
    std::fill_n(x + 13, 4, 2.4525);
    if (emit_nan)
      x[0] = std::numeric_limits<double>::quiet_NaN();
    return status;
  }
}
int main()
{
  try
  {
    std::array<double, 13> state{};
    state[9] = 1.0;
    const auto reference =
        nmpc::cost_reference({1, 2, 3}, {4, 5, 6}, {0, 1, 0}, 2.4525);
    std::vector<nmpc::CostReference> horizon(11, reference);
    {
      nmpc::TrackingController controller({});
      const auto               result = controller.compute(state, 0.0, horizon);
      check(std::abs(result.specific_force - 9.81) < 1e-12,
            "specific force incorrectly divided by mass");
      check(result.rates[0] == 0.1 && result.rates[2] == 0.3,
            "rate output layout");
      check(last_state[13] == 2.4525 && last_state[16] == 2.4525,
            "rotor state must be N/kg");
      check(first_reference[0] == 1 && first_reference[3] == 4 &&
                first_reference[10] == 1 && first_reference[12] == 2.4525,
            "cost reference layout");
      status = 2;
      check(controller.compute(state, 0.0, horizon).status == 2,
            "accepted status 2 not recorded");
      status = 4;
      throws([&] { controller.compute(state, 0.0, horizon); });
      status   = 0;
      emit_nan = true;
      throws([&] { controller.compute(state, 0.0, horizon); });
      emit_nan = false;
      horizon.pop_back();
      throws([&] { controller.compute(state, 0.0, horizon); });
      horizon.push_back(reference);
      controller.compute(state, 1.0, horizon);
      check(controller.cz() == 0.4, "aero coefficient feedback saturation");
      const int old_creates = creates, old_destroys = destroys;
      controller.reset();
      check(creates == old_creates + 1 && destroys == old_destroys + 1 &&
                controller.cz() == 0.0,
            "solver reset was incomplete");
      std::cout << "PASS NMPC units/layout/status/failure/reset contract (fake "
                   "solver)\n";
    }
    compiled_dt = 0.2;
    throws([] { nmpc::TrackingController controller({}); });
    check(creates == destroys, "solver lifetime leak");
    std::cout << "PASS stale NMPC bundle grid rejection (fake solver)\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
