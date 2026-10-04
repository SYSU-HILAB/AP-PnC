#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "simple_sim/io/experiment.hpp"

namespace
{
  namespace fs = std::filesystem;
  void yaml_file(const fs::path &path, const YAML::Node &node)
  {
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << node;
    std::ofstream output(path);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output << emitter.c_str() << '\n';
    output.close();
  }
  std::vector<std::string> deterministic_rows(const fs::path &file)
  {
    std::ifstream input(file);
    if (!input)
      throw std::runtime_error("cannot read acceptance log");
    std::vector<std::string> rows;
    std::string              row;
    while (std::getline(input, row))
    {
      // Only the last column (diagnostic solver wall time) is excluded.
      rows.push_back(row.substr(0, row.rfind(',')));
    }
    return rows;
  }
}  // namespace

int main()
{
  try
  {
    const auto root = simple_sim::project_root();
    const auto id   = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    const auto output_base = root / ".artifacts" / "benchmark";
    fs::create_directories(output_base);
    const auto suite =
        output_base / ("simple_sim_acceptance_" + std::to_string(id));
    if (!fs::create_directory(suite))
      throw std::runtime_error("acceptance directory already exists");
    auto planning = YAML::LoadFile(
        simple_sim::bringup_config(root, "planning.yaml").string());
    planning["problem_formulation"]["cost"]["v_max"] = 9.0;
    yaml_file(suite / "planning_v9.yaml", planning);
    const auto base = YAML::LoadFile(
        simple_sim::bringup_config(root, "simple_sim.yaml").string());
    std::shared_ptr<const planner::core::ReferenceTrajectory> reference;
    std::ostringstream                                        summary;
    summary << std::setprecision(17)
            << "{\n  \"planning_v_max_mps\": 9.0,\n  \"controller\": "
               "\"nmpc\",\n  \"cases\": [\n";
    bool first_case = true;
    // Acceptance order is intentional: first no aero, then Zhang-Lyu.
    for (const std::string model : {"none", "lyu"})
    {
      auto sim                           = YAML::Clone(base);
      sim["simple_sim"]["controller"]    = "nmpc";
      sim["simple_sim"]["duration_s"]    = 0.0;
      sim["simple_sim"]["aero"]["model"] = model;
      const auto input = suite / ("simple_sim_" + model + ".yaml");
      yaml_file(input, sim);
      std::vector<fs::path> runs;
      double                max_reference_speed = 0.0, max_actual_speed = 0.0;
      for (int repeat = 0; repeat < 2; ++repeat)
      {
        auto config          = simple_sim::load_config(input, "nmpc");
        config.planning_yaml = suite / "planning_v9.yaml";
        simple_sim::Experiment experiment(config, reference);
        if (!reference)
          reference = experiment.reference_handle();
        while (experiment.runner().status() != simple_sim::RunStatus::Finished)
        {
          const auto step = experiment.step();
          max_reference_speed =
              std::max(max_reference_speed,
                       reference->sample(step.context.time_s()).v.norm());
          max_actual_speed =
              std::max(max_actual_speed, step.after.velocity.norm());
        }
        runs.push_back(experiment.output_dir());
        std::cout << "Completed " << model << " repeat " << repeat << ": "
                  << runs.back() << '\n';
      }
      if (deterministic_rows(runs[0] / "steps.csv") !=
          deterministic_rows(runs[1] / "steps.csv"))
        throw std::runtime_error(model + " acceptance replay differs");
      const auto metrics = YAML::LoadFile((runs[0] / "metrics.json").string());
      if (!first_case)
        summary << ",\n";
      first_case = false;
      summary << "    {\"aero\": " << std::quoted(model)
              << ", \"completed\": true, \"exact_replay\": true"
              << ", \"sampled_reference_peak_mps\": " << max_reference_speed
              << ", \"actual_peak_mps\": " << max_actual_speed
              << ", \"position_rmse_m\": "
              << metrics["position_rmse_m"].as<double>()
              << ", \"position_max_m\": "
              << metrics["position_max_m"].as<double>() << ", \"runs\": ["
              << std::quoted(runs[0].string()) << ", "
              << std::quoted(runs[1].string()) << "]}";
    }
    summary << "\n  ]\n}\n";
    std::ofstream summary_file(suite / "acceptance.json");
    summary_file.exceptions(std::ios::failbit | std::ios::badbit);
    summary_file << summary.str();
    summary_file.close();
    std::cout << "PASS ordered NMPC acceptance: none -> lyu, v_max=9 m/s, "
                 "exact same reference and repeatable runs\n"
              << "Summary: " << suite / "acceptance.json" << '\n';
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "FAIL acceptance: " << e.what() << '\n';
    return 1;
  }
}
