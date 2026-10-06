#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace aerodynamics::paths
{
  inline std::filesystem::path root()
  {
    const char *value = std::getenv("AP_PNC_DIR");
    if (!value || !std::filesystem::path(value).is_absolute() ||
        !std::filesystem::is_directory(value))
      throw std::runtime_error(
          "AP_PNC_DIR must be an existing absolute project root");
    return std::filesystem::canonical(value);
  }

  inline std::filesystem::path input(const std::filesystem::path &value)
  {
    if (!value.is_absolute())
      for (const auto &part : value)
        if (part == "..")
          throw std::invalid_argument(
              "project-relative input must not contain '..'");
    return std::filesystem::weakly_canonical(
        value.is_absolute() ? value : root() / value);
  }

  inline std::filesystem::path output(const std::string &filename)
  {
    if (filename.empty() || filename == "." || filename == ".." ||
        std::filesystem::path(filename).filename() != filename)
      throw std::invalid_argument(
          "aerodynamics output requires a filename, not a path");
    const auto base = std::filesystem::weakly_canonical(root() / ".artifacts");
    const auto file =
        std::filesystem::weakly_canonical(base / "aerodynamics" / filename);
    const auto relative = file.lexically_relative(base);
    if (relative.empty() || *relative.begin() == "..")
      throw std::invalid_argument("aerodynamics output escapes .artifacts");
    std::filesystem::create_directories(file.parent_path());
    return file;
  }

  // Canonical per-model parameter file, e.g. config/aero/phi.yaml.
  // Single source of truth: the tracked config tree only (no .artifacts
  // shadowing).
  inline std::filesystem::path aero_config(const std::string &model)
  {
    if (model.empty() || model == "." || model == ".." ||
        std::filesystem::path(model).filename() != model)
      throw std::invalid_argument(
          "aero config requires a model name, not a path");
    return root() / "infra" / "sim_infra" / "aerodynamics" / "config" / "aero" /
           (model + ".yaml");
  }
}  // namespace aerodynamics::paths
