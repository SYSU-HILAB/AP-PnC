#pragma once

#include <array>
#include <string>
#include <string_view>

namespace aerodynamics
{
  // Validation status is a PROJECT DECISION about what we currently trust and
  // test, not a property of the model implementation itself. Keep this the only
  // list; do not scatter model names through the codebase again.
  enum class ValidationStatus
  {
    kUnderTest,    // part of the current campaign and acceptance
    kNotTested,    // compiled, but explicitly excluded from the campaign
    kNullControl,  // zero wrench; a control baseline, not a physics model
  };

  struct AeroModelEntry
  {
    std::string_view name;
    ValidationStatus status;
    std::string_view note;
  };

  inline constexpr std::array<AeroModelEntry, 5> kAeroModels{{
      {"lyu", ValidationStatus::kUnderTest, "Zhang-Lyu"},
      {"phi", ValidationStatus::kUnderTest, "phi quadratic form"},
      {"ma", ValidationStatus::kNotTested,
       "B-spline table lookup; excluded from the campaign"},
      {"advanced", ValidationStatus::kNotTested,
       "advanced lift/drag; excluded from the campaign"},
      {"none", ValidationStatus::kNullControl, "zero wrench control baseline"},
  }};

  inline const char *validation_label(ValidationStatus status)
  {
    switch (status)
    {
      case ValidationStatus::kUnderTest:
        return "under_test";
      case ValidationStatus::kNotTested:
        return "not_tested";
      case ValidationStatus::kNullControl:
        return "null_control";
    }
    return "unknown";
  }

  inline const AeroModelEntry *find_aero_model(std::string_view name)
  {
    for (const auto &entry : kAeroModels)
      if (entry.name == name)
        return &entry;
    return nullptr;
  }

  inline bool is_selectable_aero_model(const std::string &name)
  {
    return find_aero_model(name) != nullptr;
  }
}  // namespace aerodynamics
