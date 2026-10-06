#include "simple_sim/io/experiment.hpp"

#include <yaml-cpp/yaml.h>

#include <aerodynamics/aero_factory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <planner_core/planner.hpp>
#include <sstream>
#include <stdexcept>

#include "simple_sim/rate_loop.hpp"
#ifdef SIMPLE_SIM_WITH_NMPC
#include "simple_sim/adapters/nmpc_controller.hpp"
#endif

namespace simple_sim
{
  namespace fs = std::filesystem;
  namespace
  {
    std::string json_string(const std::string &s)
    {
      std::ostringstream out;
      out << '"';
      for (unsigned char c : s)
      {
        if (c == '"' || c == '\\')
          out << '\\' << c;
        else if (c < 32)
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec;
        else
          out << c;
      }
      out << '"';
      return out.str();
    }
    void snapshot(const fs::path &source, const fs::path &destination)
    {
      // Content-only copy: no permissions/metadata cloning (also works on
      // mounted filesystems).
      std::ifstream input(source, std::ios::binary);
      if (!input)
        throw std::runtime_error("cannot read config snapshot: " +
                                 source.string());
      std::ofstream output(destination, std::ios::binary);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << input.rdbuf();
      output.close();
      if (input.bad())
        throw std::runtime_error("config snapshot read failed: " +
                                 source.string());
    }
    std::int64_t ns_period(double frequency)
    {
      if (!std::isfinite(frequency) || frequency <= 0.0)
        throw std::invalid_argument("invalid control frequency");
      const double ns = 1e9 / frequency;
      if (ns < 1.0 ||
          ns >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
          std::abs(ns - std::round(ns)) > 1e-4)
        throw std::invalid_argument(
            "control period must be representable in integer nanoseconds");
      return std::llround(ns);
    }
    void vector_csv(std::ostream &out, const Eigen::Vector3d &v)
    {
      out << ',' << v.x() << ',' << v.y() << ',' << v.z();
    }
    void observation_csv(std::ostream &out, const AeroObservation &a)
    {
      out << ',' << a.alpha_rad << ',' << a.beta_rad;
      vector_csv(out, a.velocity_wing);
      out << ',' << a.valid;
    }
    void state_csv(std::ostream &out, const State &s)
    {
      vector_csv(out, s.position);
      vector_csv(out, s.velocity);
      out << ',' << s.attitude.w() << ',' << s.attitude.x() << ','
          << s.attitude.y() << ',' << s.attitude.z();
      vector_csv(out, s.angular_velocity);
    }
  }  // namespace

  Experiment::Experiment(
      RuntimeConfig config,
      std::shared_ptr<const planner::core::ReferenceTrajectory>
          cached_reference)
      : config_(std::move(config))
  {
#ifndef SIMPLE_SIM_WITH_NMPC
    if (config_.controller == "nmpc")
      throw std::runtime_error(
          "NMPC support disabled in this build; rebuild on Linux with solver "
          "bundle");
#endif
    // A fresh directory for every run; never overwrite an earlier experiment.
    const auto base = config_.root / ".artifacts" / "benchmark";
    fs::create_directories(base);
    const auto id = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    for (int suffix = 0;; ++suffix)
    {
      output_dir_ = base / ("simple_sim_" + std::to_string(id) + "_" +
                            std::to_string(suffix));
      if (fs::create_directory(output_dir_))
        break;
    }
    // Constructing can fail before the destructor runs: persist startup failure
    // here.
    try
    {
      snapshot(config_.sim_yaml, output_dir_ / "simple_sim.yaml");
      snapshot(config_.planning_yaml, output_dir_ / "planning.yaml");
      snapshot(config_.nmpc_yaml, output_dir_ / "nmpc.yaml");
      // Execute the snapshots, not files that might be edited during a run.
      config_ =
          load_config(output_dir_ / "simple_sim.yaml", config_.controller);
      config_.planning_yaml = output_dir_ / "planning.yaml";
      config_.nmpc_yaml     = output_dir_ / "nmpc.yaml";
      planner::core::ProblemConfig planning;
      planning.load(config_.planning_yaml.string());
      if (std::abs(planning.problem.flatness.mass - config_.plant.mass) > 1e-9)
        throw std::invalid_argument("planner and plant mass differ");
      const YAML::Node nmpc_settings =
          YAML::LoadFile(config_.nmpc_yaml.string())["nmpc"];
      // The NMPC is mass-normalized (specific thrust N/kg, cx in 1/m) and uses
      // ground speed, so it needs no mass and only supports zero wind.
      if (config_.controller == "nmpc" && !config_.aero.wind_world.isZero(0.0))
        throw std::invalid_argument(
            "NMPC uses ground speed for cx; only zero wind is supported");
      const auto control_dt = ns_period(nmpc_settings["ctrl_frq"].as<double>());
      planner::core::Planner planner(planning);
      if (cached_reference)
      {
        trajectory_ = std::move(cached_reference);
        if (trajectory_->empty())
          throw std::invalid_argument("empty cached planner reference");
      }
      else
      {
        auto planned = planner.plan();
        if (!planned || planned->empty())
          throw std::runtime_error("planner failed to produce a trajectory");
        trajectory_ =
            std::make_shared<const planner::core::ReferenceTrajectory>(
                std::move(*planned));
      }
      YAML::Emitter reference;
      reference.SetDoublePrecision(17);
      reference << YAML::BeginMap << YAML::Key << "order" << YAML::Value << 5
                << YAML::Key << "frame" << YAML::Value << "world_ENU"
                << YAML::Key << "terminal_policy" << YAML::Value
                << "hold_last_point" << YAML::Key << "pieces" << YAML::Value
                << YAML::BeginSeq;
      for (const auto &piece : trajectory_->representation())
      {
        reference << YAML::BeginMap << YAML::Key << "duration_s" << YAML::Value
                  << piece.getDuration() << YAML::Key
                  << "coefficients_xyz_descending" << YAML::Value
                  << YAML::BeginSeq;
        const auto &coefficients = piece.getCoeffMat();
        for (int axis = 0; axis < 3; ++axis)
        {
          reference << YAML::Flow << YAML::BeginSeq;
          for (int j = 0; j < 6; ++j)
            reference << coefficients(axis, j);
          reference << YAML::EndSeq;
        }
        reference << YAML::EndSeq << YAML::EndMap;
      }
      reference << YAML::EndSeq << YAML::EndMap;
      if (!reference.good())
        throw std::runtime_error("trajectory serialization failed");
      std::ofstream reference_file(output_dir_ / "reference.yaml");
      reference_file.exceptions(std::ios::failbit | std::ios::badbit);
      reference_file << reference.c_str() << '\n';
      reference_file.close();
      const bool   automatic = config_.duration_s == 0.0;
      const double requested =
          automatic ? trajectory_->duration() + config_.terminal_hold_s
                    : config_.duration_s;
      if (!std::isfinite(requested) || requested <= 0.0)
        throw std::invalid_argument("invalid requested duration");
      // Automatic runs include the terminal state, plus optional hover dwell.
      // Explicit-duration probes retain their historical complete-step floor.
      const double periods =
          requested / (static_cast<double>(control_dt) * 1e-9);
      const double count = automatic ? std::ceil(periods) : std::floor(periods);
      if (count < 1.0 || count >= static_cast<double>(
                                      std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument(
            "requested duration has no complete control step");
      RunnerConfig runner_cfg{config_.physics_dt_ns, control_dt,
                              static_cast<std::int64_t>(count)};
      runner_cfg.validate();
      if (config_.controller == "se3")
      {
        controller_ = std::make_unique<Se3Controller>(
            trajectory_, config_.plant.gravity, config_.se3);
      }
#ifdef SIMPLE_SIM_WITH_NMPC
      else
      {
        nmpc::TrackingConfig tracking_config;
        tracking_config.horizon_s  = nmpc_settings["horizon_s"].as<double>();
        tracking_config.traj_res_s = nmpc_settings["traj_res_s"].as<double>();
        tracking_config.ctrl_frq   = nmpc_settings["ctrl_frq"].as<double>();
        tracking_config.initial_specific_thrust =
            nmpc_settings["initial_specific_thrust"].as<double>();
        tracking_config.cx_alpha_slope =
            nmpc_settings["cx_alpha_slope"].as<double>();
        tracking_config.cx_slope_estimation =
            nmpc_settings["cx_slope_estimation"].as<bool>();
        tracking_config.cx_slope_forgetting =
            nmpc_settings["cx_slope_forgetting"].as<double>();
        tracking_config.cx_slope_limit =
            nmpc_settings["cx_slope_limit"].as<double>();
        tracking_config.cx_slope_covariance_limit =
            nmpc_settings["cx_slope_covariance_limit"].as<double>();
        tracking_config.cx_slope_min_excitation =
            nmpc_settings["cx_slope_min_excitation"].as<double>();
        tracking_config.cx_slope_lag_s =
            nmpc_settings["cx_slope_lag_s"].as<double>();
        controller_ = std::make_unique<NmpcController>(
            trajectory_, tracking_config, output_dir_ / "nmpc_reference.csv");
      }
#endif
      // Any canonical model ("none" resolves to the null model, wrench = 0)
      ForceModel aero = tailsitter_aero(
          aerodynamics::make_aero(config_.aero_model), config_.aero);
      if (config_.actuator_model == "practical")
      {
        config_.motor_model.wind_world = config_.aero.wind_world;
        const auto model               = std::make_shared<const MotorModel>(
            config_.motor_model, config_.plant.mass,
            config_.rate_loop.max_specific_force, config_.rate_loop.max_rate,
            config_.physics_dt_ns);
        runner_ = std::make_unique<Runner>(
            runner_cfg,
            Plant(config_.plant, std::move(aero),
                  [model](const State &s) { return model->derivative(s); }),
            [model](const State &s, const Command &)
            { return model->wrench(s); }, *controller_,
            [model](const State &s, const Command &c)
            { return model->initialize(s, c); },
            [model](const State &s, const Command &c)
            { return model->prepare(s, c); });
      }
      else
        runner_ = std::make_unique<Runner>(
            runner_cfg, Plant(config_.plant, std::move(aero)),
            RateLoop(config_.plant, config_.rate_loop), *controller_);
      const auto    ref0    = trajectory_->sample(0.0);
      const State   initial = initial_state(ref0, config_.plant.gravity);
      const Command command{
          (ref0.a + config_.plant.gravity * Eigen::Vector3d::UnitZ()).norm(),
          ref0.omega};
      runner_->reset(initial, command);
      log_.exceptions(std::ios::failbit | std::ios::badbit);
      log_.open(output_dir_ / "steps.csv");
      log_ << std::setprecision(17)
           << "tick,time_ns,next_time_ns,p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_"
              "z,w_x,w_y,w_z"
           << ",next_p_x,next_p_y,next_p_z,next_v_x,next_v_y,next_v_z,next_q_w,"
              "next_q_x,next_q_y,next_q_z,next_w_x,next_w_y,next_w_z"
           << ",ref_p_x,ref_p_y,ref_p_z,ref_v_x,ref_v_y,ref_v_z,ref_yb_x,ref_"
              "yb_y,ref_yb_z"
           << ",specific_force_sp,rate_x_sp,rate_y_sp,rate_z_sp,imu_x,imu_y,"
              "imu_z,aero_fx,aero_fy,aero_fz,aero_mx,aero_my,aero_mz,"
              "rpm_1,rpm_2,rpm_3,rpm_4,next_rpm_1,next_rpm_2,next_rpm_3,next_"
              "rpm_4,"
              "rpm_sp_1,rpm_sp_2,rpm_sp_3,rpm_sp_4,duty_1,duty_2,duty_3,duty_4,"
              "applied_fx,applied_fy,applied_fz,applied_mx,applied_my,applied_"
              "mz,"
              "aero_alpha_rad,aero_beta_rad,air_wing_v_x,air_wing_v_y,air_wing_"
              "v_z,aero_observation_valid,"
              "next_aero_alpha_rad,next_aero_beta_rad,next_air_wing_v_x,next_"
              "air_wing_v_y,next_air_wing_v_z,next_aero_observation_valid,"
              "next_aero_fx,next_aero_fy,next_aero_fz,next_aero_mx,next_aero_"
              "my,next_aero_mz,"
              "solver_status,solve_time_ms\n";
      finish("running");
      finished_ = false;
    }
    catch (const std::exception &e)
    {
      finish("failed", e.what());
      throw;
    }
  }
  Experiment::~Experiment()
  {
    if (!finished_)
    {
      try
      {
        finish("interrupted", "experiment destroyed before completion");
      }
      catch (...)
      { /* Destructors cannot report I/O failure. Explicit finish() can. */
      }
    }
  }
  void Experiment::metrics_sample(const State &s, double t)
  {
    const auto   ref   = trajectory_->sample(t);
    const double error = (s.position - ref.p).norm();
    squared_position_error_ += error * error;
    squared_velocity_error_ += (s.velocity - ref.v).squaredNorm();
    max_position_error_ = std::max(max_position_error_, error);
    ++samples_;
  }
  StepRecord Experiment::step()
  {
    if (finished_)
      throw std::logic_error("experiment already finalized");
    try
    {
      const auto         record = runner_->step();
      const auto         ref    = trajectory_->sample(record.context.time_s());
      std::ostringstream row;
      row << std::setprecision(17) << record.context.index << ','
          << record.context.time_ns << ','
          << record.context.time_ns + record.context.control_dt_ns;
      state_csv(row, record.before);
      state_csv(row, record.after);
      vector_csv(row, ref.p);
      vector_csv(row, ref.v);
      vector_csv(row, ref.yb);
      row << ',' << record.control.command.specific_force;
      vector_csv(row, record.control.command.rates);
      vector_csv(row, record.feedback_before.specific_force);
      vector_csv(row, record.feedback_before.aerodynamic.force);
      vector_csv(row, record.feedback_before.aerodynamic.moment);
      for (const auto &v :
           {record.before.motors.rpm, record.after.motors.rpm,
            record.before.motors.rpm_sp, record.before.motors.duty})
        for (int i = 0; i < 4; ++i)
          row << ',' << v[i];
      vector_csv(row, record.feedback_before.applied.force);
      vector_csv(row, record.feedback_before.applied.moment);
      observation_csv(row, record.feedback_before.aerodynamic.observation);
      observation_csv(row, record.feedback_after.aerodynamic.observation);
      vector_csv(row, record.feedback_after.aerodynamic.force);
      vector_csv(row, record.feedback_after.aerodynamic.moment);
      row << ',' << record.control.solver_status << ',' << record.solve_time_ms
          << '\n';
      log_ << row.str();
      log_.flush();
      metrics_sample(record.before, record.context.time_s());
      if (record.control.solver_status == 2)
        ++status2_count_;
      if (runner_->status() == RunStatus::Finished)
      {
        metrics_sample(record.after,
                       static_cast<double>(runner_->time_ns()) * 1e-9);
        finish("finished");
      }
      return record;
    }
    catch (const std::exception &e)
    {
      finish("failed", e.what());
      throw;
    }
  }
  void Experiment::finish(const std::string &status, const std::string &reason)
  {
    if (log_.is_open())
      log_.flush();
    std::ofstream manifest(output_dir_ / "manifest.json.tmp");
    manifest.exceptions(std::ios::failbit | std::ios::badbit);
    manifest
        << std::setprecision(17) << "{\n  \"status\": " << json_string(status)
        << ",\n  \"reason\": " << json_string(reason)
        << ",\n  \"controller\": " << json_string(config_.controller)
        << ",\n  \"build_revision\": " << json_string(AP_PNC_BUILD_REVISION)
        << ",\n  \"build_dirty\": " << AP_PNC_BUILD_DIRTY
        << ",\n  \"compiler\": " << json_string(AP_PNC_BUILD_COMPILER)
        << ",\n  \"platform\": " << json_string(AP_PNC_BUILD_PLATFORM)
        << ",\n  \"nmpc_bundle_sha256\": "
        << json_string(AP_PNC_NMPC_BUNDLE_SHA256)
        << ",\n  \"actuator_model\": " << json_string(config_.actuator_model)
        << ",\n  \"nmpc_prediction_model\": \""
        << (config_.controller == "nmpc" ? "command_integrator"
                                         : "not_applicable")
        << "\""
        << ",\n  \"nmpc_aero_axis\": "
        << json_string(config_.controller == "nmpc" ? "body_x_lumped"
                                                    : "not_applicable")
        << ",\n  \"arm_length_scale\": " << config_.arm_length_scale
        << ",\n  \"aero_model\": " << json_string(config_.aero_model)
        << ",\n  \"aero_validation\": " << json_string(config_.aero_validation)
#ifdef SIMPLE_SIM_WITH_NMPC
        << ",\n  \"nmpc_solver_abi\": " << NMPC_SOLVER_ABI
#endif
        << ",\n  \"integrator\": \"rk4\",\n  \"physics_dt_ns\": "
        << config_.physics_dt_ns << ",\n  \"control_dt_ns\": "
        << (runner_ ? runner_->control_dt_ns() : 0)
        << ",\n  \"reference_terminal_policy\": \"hold_last_point\""
        << ",\n  \"terminal_hold_s\": " << config_.terminal_hold_s
        << ",\n  \"trajectory_duration_s\": "
        << (trajectory_ ? trajectory_->duration() : 0.0)
        << ",\n  \"committed_steps\": " << (runner_ ? runner_->index() : 0)
        << ",\n  \"time_ns\": " << (runner_ ? runner_->time_ns() : 0)
        << ",\n  \"stochastic_model\": false\n}\n";
    manifest.close();
    fs::rename(output_dir_ / "manifest.json.tmp",
               output_dir_ / "manifest.json");
    std::ofstream metrics(output_dir_ / "metrics.json.tmp");
    metrics.exceptions(std::ios::failbit | std::ios::badbit);
    const double denominator =
        static_cast<double>(std::max<std::int64_t>(samples_, 1));
    metrics << std::setprecision(17) << "{\n  \"samples\": " << samples_
            << ",\n  \"position_rmse_m\": "
            << std::sqrt(squared_position_error_ / denominator)
            << ",\n  \"velocity_rmse_mps\": "
            << std::sqrt(squared_velocity_error_ / denominator)
            << ",\n  \"position_max_m\": " << max_position_error_
            << ",\n  \"solver_status_2_count\": " << status2_count_ << "\n}\n";
    metrics.close();
    fs::rename(output_dir_ / "metrics.json.tmp", output_dir_ / "metrics.json");
    finished_ = status != "running";
  }
}  // namespace simple_sim
