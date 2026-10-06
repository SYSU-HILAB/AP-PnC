#pragma once

#include <aerodynamics/aero_interface.hpp>
#include <memory>
#include <planner_core/trajectory.hpp>

#include "simple_sim/adapters/aero_model.hpp"
#include "simple_sim/controller.hpp"

namespace simple_sim
{

  struct Se3Gains
  {
    Eigen::Vector3d position = Eigen::Vector3d(4.0, 4.0, 4.0);
    Eigen::Vector3d velocity = Eigen::Vector3d(3.0, 3.0, 3.0);
    Eigen::Vector3d attitude = Eigen::Vector3d(6.0, 6.0, 6.0);
    // Flatness feedforward on the attitude: solve the angle of attack from the
    // true aerodynamic force balance numerically, rebuild the thrust axis from
    // it, and keep the planned yb. Off by default.
    bool aero_flatness_feedforward = false;
    // Body-rate output limit [rad/s, FLU], per axis.
    Eigen::Vector3d max_rate = Eigen::Vector3d(1.5, 2.5, 1.5);
    // Zero the thrust-axis component of the aerodynamic force in the
    // feedforward model, mirroring the plant's own injection rule for the ideal
    // profile. The two must agree or the feedforward fights a force that is not
    // there.
    bool aero_zero_thrust_axis = false;
  };

  class Se3Controller final : public Controller
  {
   public:
    Se3Controller(
        std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory,
        double gravity, Se3Gains gains,
        std::shared_ptr<aerodynamics::AerodynamicsInterface> aero_model =
            nullptr,
        double mass = 0.0, bool zero_thrust_axis_aero = false,
        AeroConfig aero_config = {});
    void          reset(const State &initial) override;
    ControlResult compute(const StepContext &, const State &,
                          const Feedback &) override;

   private:
    std::shared_ptr<const planner::core::ReferenceTrajectory> trajectory_;
    double                                                    gravity_;
    Se3Gains                                                  gains_;
    std::shared_ptr<aerodynamics::AerodynamicsInterface>      aero_model_;
    double                                                    mass_;
    double                                                    alpha_prev_;
    bool       zero_thrust_axis_aero_ = false;
    bool       alpha_initialized_     = false;
    AeroConfig aero_config_;
  };

  State initial_state(const planner::core::ReferencePoint &reference,
                      double                               gravity);

}  // namespace simple_sim
