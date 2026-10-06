/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <aerodynamics/advanced_lift_drag.hpp>
#include <aerodynamics/aero_factory.hpp>
#include <aerodynamics/bspline_aerodynamics.hpp>
#include <aerodynamics/non_aerodynamics.hpp>
#include <aerodynamics/phi_aerodynamics.hpp>
#include <aerodynamics/zhang_lyu_aerodynamics.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace aerodynamics
{
  namespace
  {
    std::unique_ptr<AerodynamicsCreator> make_creator(const std::string &name)
    {
      if (name == "lyu")
        return std::make_unique<LyuAerodynamicsCreator>();
      if (name == "phi")
        return std::make_unique<PhiAerodynamicsCreator>();
      if (name == "ma")
        return std::make_unique<BsplineAerodynamicsCreator>();
      if (name == "advanced")
        return std::make_unique<AdvancedLiftDragCreator>();
      if (name == "none")
        return std::make_unique<NonAerodynamicsCreator>();

      std::string valid;
      for (const std::string_view model : kAeroModelNames)
        valid +=
            valid.empty() ? std::string(model) : " | " + std::string(model);
      throw std::invalid_argument("unknown aerodynamics model '" + name +
                                  "'; expected one of: " + valid);
    }
  }  // namespace

  std::shared_ptr<AerodynamicsInterface> make_aero(const std::string &name)
  {
    auto creator = make_creator(name);
    if (!creator->createAerodynamics())
      throw std::runtime_error(
          "failed to load parameters for aerodynamics "
          "model '" +
          name + "'");
    auto instance = creator->takeInstance();
    if (!instance)
      throw std::runtime_error("aerodynamics model '" + name +
                               "' produced no instance");
    return std::shared_ptr<AerodynamicsInterface>(std::move(instance));
  }
}  // namespace aerodynamics
