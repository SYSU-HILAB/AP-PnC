// Interactive city mission: shared planner core + real NMPC + lockstep plant.
// ROS schedules committed simulation steps; the browser is only an observer.
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <nav_msgs/msg/odometry.hpp>
#include <planner_core/planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>

#include "simple_sim/io/experiment.hpp"

namespace
{
  using Trajectory = planner::core::ReferenceTrajectory;
  using Clock      = std::chrono::steady_clock;

  Trajectory hold(const Eigen::Vector3d &position, double duration)
  {
    Eigen::Matrix<double, 3, 6> coefficients =
        Eigen::Matrix<double, 3, 6>::Zero();
    coefficients.col(5) = position;
    TrajectoryRepresentation<5> pieces;
    pieces.emplace_back(duration, coefficients);
    return Trajectory(std::move(pieces));
  }

  void vector_json(std::ostream &out, const Eigen::Vector3d &v)
  {
    out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
  }

  struct Plan
  {
    std::optional<Trajectory> trajectory;
    double                    milliseconds = 0.0;
  };

  class CityMission final : public rclcpp::Node
  {
   public:
    CityMission() : Node("city_mission")
    {
      const auto mission_path = declare_parameter<std::string>("mission", "");
      if (!std::filesystem::path(mission_path).is_absolute())
        throw std::invalid_argument("absolute mission path required");
      const auto mission = YAML::LoadFile(mission_path);
      output_            = mission["output"].as<std::string>();
      for (const auto *key : {"output", "planning", "simulation", "occupancy"})
        if (!std::filesystem::path(mission[key].as<std::string>())
                 .is_absolute())
          throw std::invalid_argument("all mission paths must be absolute");
      const char *root_env = std::getenv("AP_PNC_DIR");
      if (!root_env)
        throw std::invalid_argument("AP_PNC_DIR is required");
      if (!std::filesystem::path(root_env).is_absolute())
        throw std::invalid_argument("AP_PNC_DIR must be absolute");
      const auto root = std::filesystem::weakly_canonical(root_env);
      output_         = std::filesystem::weakly_canonical(output_);
      if (!root.is_absolute() || !output_.is_absolute() ||
          output_.lexically_relative(root / ".artifacts").string().find("..") ==
              0)
        throw std::invalid_argument(
            "output must be under absolute AP_PNC_DIR/.artifacts");
      std::filesystem::create_directories(output_);
      for (const auto &point : mission["waypoints_m"])
        waypoints_.emplace_back(point[0].as<double>(), point[1].as<double>(),
                                point[2].as<double>());
      if (waypoints_.size() < 4 ||
          (waypoints_.front() - waypoints_.back()).norm() > 1e-6)
        throw std::invalid_argument(
            "city mission must be a closed waypoint loop");
      for (const auto &v : mission["velocities_mps"])
        velocities_.emplace_back(v[0].as<double>(), v[1].as<double>(),
                                 v[2].as<double>());
      if (velocities_.size() != waypoints_.size() ||
          !velocities_.front().isZero() || !velocities_.back().isZero())
        throw std::invalid_argument(
            "fly-through velocities must match route with rest endpoints");
      for (std::size_t i = 0; i < waypoints_.size(); ++i)
        if (!waypoints_[i].allFinite() || !velocities_[i].allFinite() ||
            (i > 0 && i + 1 < waypoints_.size() && velocities_[i].norm() < 9.9))
          throw std::invalid_argument("invalid fly-through PVA");
      accelerations_.assign(waypoints_.size(), Eigen::Vector3d::Zero());
      if (mission["accelerations_mps2"])
      {
        const auto values = mission["accelerations_mps2"];
        if (values.size() != waypoints_.size())
          throw std::invalid_argument("accelerations must match route");
        for (std::size_t i = 0; i < values.size(); ++i)
        {
          accelerations_[i] = Eigen::Vector3d(values[i][0].as<double>(),
                                              values[i][1].as<double>(),
                                              values[i][2].as<double>());
          if (!accelerations_[i].allFinite())
            throw std::invalid_argument("nonfinite mission acceleration");
        }
        if (!accelerations_.front().isZero() || !accelerations_.back().isZero())
          throw std::invalid_argument(
              "rest endpoints require zero acceleration");
      }
      planner::core::ProblemConfig planning;
      planning.load(mission["planning"].as<std::string>());
      planner_    = std::make_unique<planner::core::Planner>(planning);
      auto vector = [](const YAML::Node &value)
      {
        return Eigen::Vector3d(value[0].as<double>(), value[1].as<double>(),
                               value[2].as<double>());
      };
      const auto   origin      = vector(mission["map_origin_m"]);
      const auto   corner      = vector(mission["map_corner_m"]);
      const double voxel       = mission["voxel_m"].as<double>();
      const double scene_scale = mission["scene_scale"].as<double>();
      if (!origin.allFinite() || !corner.allFinite() ||
          !(corner.array() > origin.array()).all() || !std::isfinite(voxel) ||
          voxel <= 0 || !std::isfinite(scene_scale) || scene_scale <= 0)
        throw std::invalid_argument("invalid metric map transform");
      map_ = std::make_unique<voxel_map::VoxelMap>(
          ((corner - origin) / voxel).array().ceil().cast<int>().matrix(),
          origin, voxel);
      std::ifstream cloud(mission["occupancy"].as<std::string>(),
                          std::ios::binary);
      std::string   line;
      std::size_t   count  = 0;
      bool          binary = false;
      while (std::getline(cloud, line) && line != "end_header")
      {
        if (line == "format binary_little_endian 1.0")
          binary = true;
        if (line.rfind("element vertex ", 0) == 0)
          count = std::stoull(line.substr(15));
      }
      if (!binary || !count || line != "end_header")
        throw std::runtime_error("expected prepared binary xyz occupancy PLY");
      for (std::size_t i = 0; i < count; ++i)
      {
        float xyz[3];
        if (!cloud.read(reinterpret_cast<char *>(xyz), sizeof(xyz)))
          throw std::runtime_error("truncated occupancy PLY");
        const Eigen::Vector3d p =
            scene_scale * Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
        if (!p.allFinite())
          throw std::runtime_error("nonfinite occupancy point");
        if ((p.array() >= origin.array()).all() &&
            (p.array() < corner.array()).all())
          map_->setOccupied(p);
      }
      map_->dilate(mission["dilation_cells"].as<int>());
      for (const auto &p : waypoints_)
        if (map_->query(p) != 0)
          throw std::runtime_error("mission waypoint is occupied");
      simulation_path_ = mission["simulation"].as<std::string>();
      reset_run();
      odom_ = create_publisher<nav_msgs::msg::Odometry>("/simple_sim/odom", 10);
      clock_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
      start_ = create_service<std_srvs::srv::Trigger>(
          "~/resume",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            if (done_)
            {
              response->success = false;
              response->message = "mission ended; launch a new run";
              return;
            }
            paused_           = false;
            response->success = true;
          });
      pause_ = create_service<std_srvs::srv::Trigger>(
          "~/pause",
          [this](std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            paused_           = true;
            response->success = true;
          });
      const auto period =
          std::chrono::nanoseconds(experiment_->runner().control_dt_ns());
      timer_ = create_wall_timer(period, [this] { tick(); });
      publish();
      RCLCPP_INFO(get_logger(),
                  "City loop ready, paused; %zu waypoints, practical NMPC; "
                  "resume /city_mission/resume",
                  waypoints_.size());
    }

   private:
    void reset_run()
    {
      if (pending_.valid())
        throw std::logic_error(
            "cannot restart during an in-flight planner task");
      auto config = simple_sim::load_config(simulation_path_, "nmpc");
      experiment_ = std::make_unique<simple_sim::Experiment>(
          config,
          std::make_shared<const Trajectory>(hold(waypoints_.front(), 8.0)));
      if (events_.is_open())
        events_.close();
      events_.open(experiment_->output_dir() / "replans.jsonl");
      events_.exceptions(std::ios::failbit | std::ios::badbit);
      events_ << std::setprecision(17);
      flown_.clear();
      flown_.push_back(waypoints_.front());
      poses_.clear();
      remember_pose();
      target_       = 1;
      last_plan_ms_ = 0;
      done_         = false;
      paused_       = true;
      phase_        = "initial-hover";
    }

    void control()
    {
      const auto path = output_ / "control.json";
      if (!std::filesystem::exists(path))
        return;
      const auto command = YAML::LoadFile(path.string());
      std::filesystem::remove(path);
      command_id_          = command["id"].as<std::string>();
      const auto operation = command["operation"].as<std::string>();
      if (operation == "resume" && !done_)
        paused_ = false;
      else if (operation == "pause")
        paused_ = true;
      else if (operation == "restart" && (done_ || paused_) &&
               !pending_.valid())
        reset_run();
      else
        RCLCPP_WARN(get_logger(), "command rejected in current mission state");
      publish();
    }

    void tick()
    {
      try
      {
        control();
        if (paused_ || done_)
        {
          // Never append while stopped, but consume a completed future so a
          // failed/paused mission cannot permanently disable the reset button.
          if (pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) ==
                                      std::future_status::ready)
          {
            try
            {
              (void)pending_.get();
            }
            catch (const std::exception &)
            {
              RCLCPP_WARN(get_logger(),
                          "discarded planner failure while stopped");
            }
            publish();
          }
          if (++heartbeat_ % 50 == 0)
            publish();
          return;
        }
        const double now       = experiment_->runner().time_ns() * 1e-9;
        const double remaining = experiment_->trajectory().duration() - now;
        if (pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) ==
                                    std::future_status::ready)
        {
          auto plan = pending_.get();
          if (!plan.trajectory)
            throw std::runtime_error("corridor planner failed");
          const double start_time = experiment_->trajectory().duration();
          // Reject soft-constraint violations rather than drawing an impossible
          // flight.
          for (double t = 0; t <= plan.trajectory->duration(); t += 0.02)
            if (map_->query(plan.trajectory->sample(t).p) != 0)
              throw std::runtime_error(
                  "planned reference intersects the inflated map");
          auto pieces = plan.trajectory->representation();
          // Never insert a hold at an intermediate waypoint. Only the final
          // return-to-start reference ends at rest and gets a hover suffix.
          if (target_ + 1 == waypoints_.size())
            pieces.append(hold(waypoints_[target_], 5.0).representation());
          experiment_->append_reference(Trajectory(std::move(pieces)));
          last_plan_ms_ = plan.milliseconds;
          events_ << "{\"id\":" << target_ << ",\"planned_at_s\":" << now
                  << ",\"starts_at_s\":" << start_time
                  << ",\"duration_s\":" << plan.trajectory->duration()
                  << ",\"wall_ms\":" << last_plan_ms_ << ",\"target_m\":";
          vector_json(events_, waypoints_[target_]);
          events_ << ",\"terminal_velocity_mps\":";
          vector_json(events_, velocities_[target_]);
          events_ << ",\"terminal_acceleration_mps2\":";
          vector_json(events_, accelerations_[target_]);
          events_ << "}\n";
          events_.flush();
          ++target_;
          RCLCPP_INFO(
              get_logger(), "committed leg %zu / %zu: %.1f ms, starts %.2f s",
              target_ - 1, waypoints_.size() - 1, last_plan_ms_, start_time);
        }
        else if (!pending_.valid() && target_ < waypoints_.size() &&
                 remaining < 6.5)
        {
          const auto end = experiment_->trajectory().sample(
              experiment_->trajectory().duration());
          Eigen::Matrix3d start = Eigen::Matrix3d::Zero(),
                          goal  = Eigen::Matrix3d::Zero();
          start.col(0)          = end.p;
          start.col(1)          = end.v;
          start.col(2)          = end.a;
          goal.col(0)           = waypoints_[target_];
          goal.col(1)           = velocities_[target_];
          goal.col(2)           = accelerations_[target_];
          pending_              = std::async(
              std::launch::async,
              [this, start, goal]
              {
                const auto begin = Clock::now();
                auto planned     = planner_->plan_corridor(start, goal, *map_);
                const double ms  = std::chrono::duration<double, std::milli>(
                                       Clock::now() - begin)
                                       .count();
                return Plan{std::move(planned), ms};
              });
        }
        if (pending_.valid() && remaining < 1.3)
          throw std::runtime_error(
              "replan missed the protected NMPC horizon deadline");
        if (target_ == waypoints_.size() && remaining <= 1.5)
        {
          const auto &state = experiment_->runner().state();
          if ((state.position - waypoints_.front()).norm() > 1.0 ||
              state.velocity.norm() > 0.5)
            throw std::runtime_error("return hover tolerance not reached");
          done_  = true;
          phase_ = "complete";
          experiment_->finish("finished");
          publish();
          return;
        }
        experiment_->step();
        const auto &state = experiment_->runner().state();
        remember_pose();
        if (map_->query(state.position) != 0)
          throw std::runtime_error("plant entered inflated occupancy");
        if (experiment_->finalized())
          throw std::runtime_error("mission exceeded duration limit");
        phase_ = now < 8 ? "initial-hover"
                         : (target_ == waypoints_.size() && remaining < 5.0
                                ? "return-hover"
                                : "flying");
        if (experiment_->runner().index() % 5 == 0)
        {
          flown_.push_back(state.position);
          publish();
        }
      }
      catch (const std::exception &e)
      {
        done_  = true;
        phase_ = "failed";
        experiment_->finish("failed", e.what());
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        publish();
      }
    }

    void remember_pose()
    {
      const auto &state = experiment_->runner().state();
      poses_.push_back(
          {experiment_->runner().time_ns() * 1e-9, state.position.x(),
           state.position.y(), state.position.z(), state.attitude.x(),
           state.attitude.y(), state.attitude.z(), state.attitude.w()});
      while (poses_.size() > 16)
        poses_.pop_front();
    }

    void publish()
    {
      const auto               &state = experiment_->runner().state();
      const double              time  = experiment_->runner().time_ns() * 1e-9;
      rosgraph_msgs::msg::Clock clock;
      clock.clock = rclcpp::Time(experiment_->runner().time_ns());
      clock_->publish(clock);
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp            = clock.clock;
      odometry.header.frame_id         = "world";
      odometry.child_frame_id          = "base_link";
      odometry.pose.pose.position.x    = state.position.x();
      odometry.pose.pose.position.y    = state.position.y();
      odometry.pose.pose.position.z    = state.position.z();
      odometry.pose.pose.orientation.w = state.attitude.w();
      odometry.pose.pose.orientation.x = state.attitude.x();
      odometry.pose.pose.orientation.y = state.attitude.y();
      odometry.pose.pose.orientation.z = state.attitude.z();
      const auto body_velocity = state.attitude.conjugate() * state.velocity;
      odometry.twist.twist.linear.x  = body_velocity.x();
      odometry.twist.twist.linear.y  = body_velocity.y();
      odometry.twist.twist.linear.z  = body_velocity.z();
      odometry.twist.twist.angular.x = state.angular_velocity.x();
      odometry.twist.twist.angular.y = state.angular_velocity.y();
      odometry.twist.twist.angular.z = state.angular_velocity.z();
      odom_->publish(odometry);
      std::ofstream out(output_ / "live.json.tmp");
      out.exceptions(std::ios::failbit | std::ios::badbit);
      out << std::setprecision(12) << "{\"t\":" << time << ",\"phase\":\""
          << (paused_ && !done_ ? "paused" : phase_)
          << "\",\"leg\":" << target_ - 1
          << ",\"total\":" << waypoints_.size() - 1
          << ",\"plan_ms\":" << last_plan_ms_
          << ",\"pending\":" << (pending_.valid() ? "true" : "false")
          << ",\"p\":";
      vector_json(out, state.position);
      out << ",\"v\":";
      vector_json(out, state.velocity);
      out << ",\"q\":[" << state.attitude.x() << ',' << state.attitude.y()
          << ',' << state.attitude.z() << ',' << state.attitude.w()
          << "],\"reference\":[";
      bool first = true;
      for (double t = time; t <= experiment_->trajectory().duration(); t += 0.2)
      {
        if (!first)
          out << ',';
        vector_json(out, experiment_->trajectory().sample(t).p);
        first = false;
      }
      out << "],\"waypoints\":[";
      first = true;
      for (const auto &p : waypoints_)
      {
        if (!first)
          out << ',';
        vector_json(out, p);
        first = false;
      }
      out << "],\"trail\":[";
      first = true;
      for (const auto &p : flown_)
      {
        if (!first)
          out << ',';
        vector_json(out, p);
        first = false;
      }
      out << "],\"recording\":\"" << experiment_->output_dir().string()
          << "\",\"stream\":\"" << output_.string() << "\",\"command_id\":\""
          << command_id_ << "\",\"wall_unix_ms\":"
          << std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch())
                 .count()
          << "}\n";
      out.close();
      std::filesystem::rename(output_ / "live.json.tmp", output_ / "live.json");
      std::ofstream pose_file(output_ / "poses.json.tmp");
      pose_file.exceptions(std::ios::failbit | std::ios::badbit);
      pose_file << std::setprecision(17) << "{\"samples\":[";
      bool first_pose = true;
      for (const auto &pose : poses_)
      {
        if (!first_pose)
          pose_file << ',';
        pose_file << '[';
        for (std::size_t i = 0; i < pose.size(); ++i)
        {
          if (i)
            pose_file << ',';
          pose_file << pose[i];
        }
        pose_file << ']';
        first_pose = false;
      }
      pose_file << "]}\n";
      pose_file.close();
      std::filesystem::rename(output_ / "poses.json.tmp",
                              output_ / "poses.json");
    }

    std::filesystem::path        output_, simulation_path_;
    std::string                  command_id_;
    std::size_t                  heartbeat_ = 0;
    std::vector<Eigen::Vector3d> waypoints_, velocities_, accelerations_,
        flown_;
    std::unique_ptr<voxel_map::VoxelMap>    map_;
    std::unique_ptr<planner::core::Planner> planner_;
    std::unique_ptr<simple_sim::Experiment> experiment_;
    std::deque<std::array<double, 8>>       poses_;
    std::future<Plan>                       pending_;
    std::ofstream                           events_;
    std::size_t                             target_       = 1;
    double                                  last_plan_ms_ = 0;
    bool                                    paused_ = true, done_ = false;
    std::string                             phase_ = "initial-hover";
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr      start_, pause_;
    rclcpp::TimerBase::SharedPtr                            timer_;
  };
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<CityMission>());
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
  rclcpp::shutdown();
}
