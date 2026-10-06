#include <atomic>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>

#include "simple_sim/io/experiment.hpp"

namespace
{
  volatile std::sig_atomic_t interrupted = 0;
  void                       signal_handler(int)
  {
    interrupted = 1;
  }
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    std::filesystem::path config;
    std::string           controller;
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if ((arg == "--config" || arg == "--controller") && i + 1 < argc)
      {
        if (arg == "--config")
          config = argv[++i];
        else
          controller = argv[++i];
      }
      else if (arg == "--help")
      {
        std::cout << "simple_sim_run [--config ABSOLUTE_YAML] [--controller "
                     "nmpc|se3]\n";
        return 0;
      }
      else
        throw std::invalid_argument("unknown/incomplete argument: " + arg);
    }
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    simple_sim::Experiment experiment(
        simple_sim::load_config(config, controller));
    std::cout << "Output: " << experiment.output_dir() << '\n';
    while (!interrupted &&
           experiment.runner().status() != simple_sim::RunStatus::Finished)
      experiment.step();
    if (interrupted)
    {
      experiment.finish("interrupted", "SIGINT/SIGTERM");
      return 130;
    }
    std::cout << "Completed " << experiment.runner().index()
              << " control steps\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "simple_sim: " << e.what() << '\n';
    return 1;
  }
}
