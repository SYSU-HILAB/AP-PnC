/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#pragma once
#include <memory>
#include <string>
#include <string_view>
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{
  /**
   * @brief Canonical aerodynamics model names, shared by every simulator
   *
   * The string passed to make_aero() is the single naming authority across
   * the project (simple_sim yaml `aero.model`, gazebo `<aero_model>`):
   *
   *   "lyu"      Zhang-Lyu MATLAB-generated model
   *   "phi"      linear phi-theory model
   *   "ma"       B-spline fitted model (BsplineAerodynamics)
   *   "advanced" advanced lift-drag model (stall effects)
   *   "none"     null model (zero wrench)
   */
  inline constexpr std::string_view kAeroModelNames[] = {"lyu", "phi", "ma",
                                                         "advanced", "none"};

  /**
   * @brief Create an aerodynamics model by canonical name
   *
   * Parameters load from config/aero/<name>.yaml (see project_paths.hpp).
   *
   * @param name one of kAeroModelNames
   * @return initialized model instance
   * @throws std::invalid_argument on an unknown name
   * @throws std::runtime_error if parameters cannot be loaded
   */
  std::shared_ptr<AerodynamicsInterface> make_aero(const std::string &name);
}  // namespace aerodynamics
