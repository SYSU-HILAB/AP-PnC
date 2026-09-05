/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <iomanip>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "simple_sim/controller.hpp"
#include "simple_sim/dynamics.hpp"
#include "simple_sim/lidar_sim.hpp"
#include "simple_sim/scene.hpp"

namespace simple_sim
{

  class SimpleSimNode : public rclcpp::Node
  {
   public:
    SimpleSimNode() : Node("simple_sim")
    {
      declareParameters();

      const double    mass    = get_parameter("mass").as_double();
      const double    gravity = get_parameter("gravity").as_double();
      Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
      inertia.diagonal() << get_parameter("inertia_xx").as_double(),
          get_parameter("inertia_yy").as_double(),
          get_parameter("inertia_zz").as_double();

      ControlGains gains;
      gains.kp_pos = vectorParam("kp_pos");
      gains.kd_vel = vectorParam("kd_vel");
      gains.kp_att = vectorParam("kp_att");

      LidarConfig lidar;
      lidar.points_per_frame = get_parameter("lidar.points_per_frame").as_int();
      lidar.fov_h_deg        = get_parameter("lidar.fov_h_deg").as_double();
      lidar.fov_v_min_deg    = get_parameter("lidar.fov_v_min_deg").as_double();
      lidar.fov_v_max_deg    = get_parameter("lidar.fov_v_max_deg").as_double();
      lidar.max_range        = get_parameter("lidar.max_range").as_double();
      lidar.min_range        = get_parameter("lidar.min_range").as_double();

      dump_pcd_path_ = get_parameter("dump_pcd_path").as_string();

      dynamics_   = std::make_unique<Dynamics>(mass, inertia, gravity);
      controller_ = std::make_unique<Controller>(gains, mass, gravity);
      lidar_      = std::make_unique<LidarSim>(lidar);
      scene_      = std::make_unique<Scene>(buildScene());

      state_.position         = vectorParam("start_pos");
      state_.velocity         = Eigen::Vector3d::Zero();
      state_.attitude         = Eigen::Quaterniond::Identity();
      state_.angular_velocity = Eigen::Vector3d::Zero();

      const std::vector<double> wp =
          get_parameter("waypoints").as_double_array();
      for (std::size_t i = 0; i + 2 < wp.size(); i += 3)
      {
        waypoints_.emplace_back(wp[i], wp[i + 1], wp[i + 2]);
      }
      if (waypoints_.empty())
      {
        waypoints_.push_back(state_.position);
      }

      points_pub_ =
          create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 2);
      odom_pub_    = create_publisher<nav_msgs::msg::Odometry>("~/odom", 2);
      markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/markers", 2);

      const double sim_rate     = get_parameter("sim_rate").as_double();
      const double publish_rate = get_parameter("publish_rate").as_double();
      sim_timer_ =
          create_wall_timer(std::chrono::duration<double>(1.0 / sim_rate),
                            [this] { simulate(); });
      publish_timer_ =
          create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                            [this] { publishState(); });
    }

   private:
    void declareParameters()
    {
      declare_parameter("mass", 2.0);
      declare_parameter("gravity", 9.81);
      declare_parameter("inertia_xx", 0.02);
      declare_parameter("inertia_yy", 0.02);
      declare_parameter("inertia_zz", 0.03);
      declare_parameter("sim_rate", 200.0);
      declare_parameter("publish_rate", 10.0);
      declare_parameter("start_pos", std::vector<double>{-45.0, 0.0, 2.0});
      declare_parameter("map.length", 100.0);
      declare_parameter("map.width", 20.0);
      declare_parameter("map.height", 20.0);
      declare_parameter("waypoint_tolerance", 0.3);
      declare_parameter("kp_pos", std::vector<double>{12.0, 12.0, 12.0});
      declare_parameter("kd_vel", std::vector<double>{3.0, 3.0, 3.0});
      declare_parameter("ki_pos", std::vector<double>{0.5, 0.5, 0.5});
      declare_parameter("kp_att", std::vector<double>{8.0, 8.0, 8.0});
      declare_parameter("lidar.points_per_frame", 4000);
      declare_parameter("lidar.fov_h_deg", 360.0);
      declare_parameter("lidar.fov_v_min_deg", -7.0);
      declare_parameter("lidar.fov_v_max_deg", 52.0);
      declare_parameter("lidar.max_range", 40.0);
      declare_parameter("lidar.min_range", 0.1);
      declare_parameter("waypoints",
                        std::vector<double>{-45.0, 0.0, 2.0, 45.0, 0.0, 2.0});
      declare_parameter("build_walls", true);
      // When set, every published frame is overwritten here as ASCII PCD
      // (x y z intensity) for offline inspection.
      declare_parameter("dump_pcd_path", "");
      // Procedural pillar field: density = pillar_count along map length.
      declare_parameter("scene.pillar_count", 8);
      declare_parameter("scene.radius_min", 0.6);
      declare_parameter("scene.radius_max", 1.2);
      declare_parameter("scene.corridor_half_width", 3.0);
      declare_parameter("scene.edge_margin", 2.0);
      declare_parameter("scene.seed", 42);
    }

    Eigen::Vector3d vectorParam(const std::string &name) const
    {
      const std::vector<double> v   = get_parameter(name).as_double_array();
      Eigen::Vector3d           out = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < v.size() && i < 3; ++i)
      {
        out[i] = v[i];
      }
      return out;
    }

    Scene buildScene() const
    {
      PillarCourseConfig cfg;
      cfg.length              = get_parameter("map.length").as_double();
      cfg.width               = get_parameter("map.width").as_double();
      cfg.height              = get_parameter("map.height").as_double();
      cfg.pillar_count        = get_parameter("scene.pillar_count").as_int();
      cfg.radius_min          = get_parameter("scene.radius_min").as_double();
      cfg.radius_max          = get_parameter("scene.radius_max").as_double();
      cfg.corridor_half_width =
          get_parameter("scene.corridor_half_width").as_double();
      cfg.edge_margin         = get_parameter("scene.edge_margin").as_double();
      cfg.seed                =
          static_cast<unsigned>(get_parameter("scene.seed").as_int());
      cfg.build_walls         = get_parameter("build_walls").as_bool();
      return makePillarCourse(cfg);
    }

    void simulate()
    {
      const Eigen::Vector3d target  = waypoints_[waypoint_index_];
      const Eigen::Vector3d vel_des = Eigen::Vector3d::Zero();

      Eigen::Vector3d thrust;
      Eigen::Vector3d moment;
      controller_->compute(state_, target, vel_des, sim_dt_, &thrust, &moment);
      dynamics_->step(sim_dt_, thrust, moment, &state_);

      if ((state_.position - target).norm() <
          get_parameter("waypoint_tolerance").as_double())
      {
        waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();
      }
    }

    sensor_msgs::msg::PointCloud2 buildCloudMsg()
    {
      std::vector<LidarPoint> points;
      lidar_->scan(*scene_, state_.position, state_.attitude, frame_index_,
                   &points);
      ++frame_index_;

      sensor_msgs::msg::PointCloud2 msg;
      msg.header.frame_id = "world";
      msg.height          = 1;
      msg.width           = static_cast<std::uint32_t>(points.size());
      msg.is_dense        = true;
      msg.point_step      = 16;
      msg.row_step        = msg.point_step * msg.width;

      const auto make_field = [](const std::string &name, std::uint32_t offset)
      {
        sensor_msgs::msg::PointField f;
        f.name     = name;
        f.offset   = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
        return f;
      };
      msg.fields.push_back(make_field("x", 0));
      msg.fields.push_back(make_field("y", 4));
      msg.fields.push_back(make_field("z", 8));
      msg.fields.push_back(make_field("intensity", 12));

      msg.data.resize(msg.row_step * msg.height);
      std::array<float, 4> pt{};
      for (std::size_t i = 0; i < points.size(); ++i)
      {
        pt[0] = static_cast<float>(points[i].point.x());
        pt[1] = static_cast<float>(points[i].point.y());
        pt[2] = static_cast<float>(points[i].point.z());
        pt[3] = static_cast<float>(points[i].intensity);
        std::memcpy(msg.data.data() + i * msg.point_step, pt.data(),
                    msg.point_step);
      }
      if (!dump_pcd_path_.empty())
      {
        writeAsciiPcd(dump_pcd_path_, points);
      }
      return msg;
    }

    // Dump one frame as ASCII PCD (x y z intensity), overwritten each frame.
    void writeAsciiPcd(const std::string &path,
                       const std::vector<LidarPoint> &points)
    {
      std::ofstream out(path, std::ios::trunc);
      if (!out)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "cannot open %s for PCD dump", path.c_str());
        return;
      }
      out << "# .PCD v0.7 - Point Cloud Data file format\n"
             "VERSION 0.7\n"
             "FIELDS x y z intensity\n"
             "SIZE 4 4 4 4\n"
             "TYPE F F F F\n"
             "COUNT 1 1 1 1\n"
             "WIDTH " << points.size() << "\n"
             "HEIGHT 1\n"
             "VIEWPOINT 0 0 0 1 0 0 0\n"
             "POINTS " << points.size() << "\n"
             "DATA ascii\n";
      out << std::fixed << std::setprecision(4);
      for (const auto &p : points)
      {
        out << p.point.x() << ' ' << p.point.y() << ' ' << p.point.z() << ' '
            << p.intensity << '\n';
      }
    }

    nav_msgs::msg::Odometry buildOdomMsg() const
    {
      nav_msgs::msg::Odometry msg;
      msg.header.frame_id         = "world";
      msg.child_frame_id          = "base_link";
      msg.pose.pose.position.x    = state_.position.x();
      msg.pose.pose.position.y    = state_.position.y();
      msg.pose.pose.position.z    = state_.position.z();
      msg.pose.pose.orientation.w = state_.attitude.w();
      msg.pose.pose.orientation.x = state_.attitude.x();
      msg.pose.pose.orientation.y = state_.attitude.y();
      msg.pose.pose.orientation.z = state_.attitude.z();
      msg.twist.twist.linear.x    = state_.velocity.x();
      msg.twist.twist.linear.y    = state_.velocity.y();
      msg.twist.twist.linear.z    = state_.velocity.z();
      msg.twist.twist.angular.x   = state_.angular_velocity.x();
      msg.twist.twist.angular.y   = state_.angular_velocity.y();
      msg.twist.twist.angular.z   = state_.angular_velocity.z();
      return msg;
    }

    visualization_msgs::msg::MarkerArray buildMarkerMsg() const
    {
      visualization_msgs::msg::MarkerArray arr;
      std_msgs::msg::ColorRGBA             color;

      const auto fill_base = [this](visualization_msgs::msg::Marker *m, int id)
      {
        m->header.frame_id = "world";
        m->id              = id;
        m->lifetime        = rclcpp::Duration(0, 0);
      };

      visualization_msgs::msg::Marker drone;
      fill_base(&drone, 0);
      drone.type               = visualization_msgs::msg::Marker::CUBE;
      drone.scale.x            = 0.5;
      drone.scale.y            = 0.5;
      drone.scale.z            = 0.5;
      drone.pose.position.x    = state_.position.x();
      drone.pose.position.y    = state_.position.y();
      drone.pose.position.z    = state_.position.z();
      drone.pose.orientation.w = state_.attitude.w();
      drone.pose.orientation.x = state_.attitude.x();
      drone.pose.orientation.y = state_.attitude.y();
      drone.pose.orientation.z = state_.attitude.z();
      color.r                  = 1.0;
      color.g                  = 0.0;
      color.b                  = 0.0;
      color.a                  = 1.0;
      drone.color              = color;
      arr.markers.push_back(drone);

      visualization_msgs::msg::Marker path;
      fill_base(&path, 1);
      path.type               = visualization_msgs::msg::Marker::LINE_STRIP;
      path.scale.x            = 0.05;
      path.pose.orientation.w = 1.0;
      color.r                 = 0.0;
      color.g                 = 1.0;
      color.b                 = 0.0;
      color.a                 = 1.0;
      path.color              = color;
      for (const auto &wp : waypoints_)
      {
        geometry_msgs::msg::Point p;
        p.x = wp.x();
        p.y = wp.y();
        p.z = wp.z();
        path.points.push_back(p);
      }
      arr.markers.push_back(path);

      int id = 2;
      for (const auto &box : scene_->boxes)
      {
        visualization_msgs::msg::Marker m;
        fill_base(&m, id++);
        m.type               = visualization_msgs::msg::Marker::CUBE;
        m.pose.position.x    = box.center.x();
        m.pose.position.y    = box.center.y();
        m.pose.position.z    = box.center.z();
        m.pose.orientation.w = 1.0;
        m.scale.x            = 2.0 * box.half_extent.x();
        m.scale.y            = 2.0 * box.half_extent.y();
        m.scale.z            = 2.0 * box.half_extent.z();
        color.r              = 0.3;
        color.g              = 0.3;
        color.b              = 0.9;
        color.a              = 0.6;
        m.color              = color;
        arr.markers.push_back(m);
      }
      for (const auto &sphere : scene_->spheres)
      {
        visualization_msgs::msg::Marker m;
        fill_base(&m, id++);
        m.type               = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.x    = sphere.center.x();
        m.pose.position.y    = sphere.center.y();
        m.pose.position.z    = sphere.center.z();
        m.pose.orientation.w = 1.0;
        m.scale.x            = 2.0 * sphere.radius;
        m.scale.y            = 2.0 * sphere.radius;
        m.scale.z            = 2.0 * sphere.radius;
        color.r              = 0.9;
        color.g              = 0.6;
        color.b              = 0.2;
        color.a              = 0.6;
        m.color              = color;
        arr.markers.push_back(m);
      }
      for (const auto &cyl : scene_->cylinders)
      {
        visualization_msgs::msg::Marker m;
        fill_base(&m, id++);
        m.type               = visualization_msgs::msg::Marker::CYLINDER;
        m.pose.position.x    = cyl.center_xy.x();
        m.pose.position.y    = cyl.center_xy.y();
        m.pose.position.z    = cyl.height / 2.0;
        m.pose.orientation.w = 1.0;
        m.scale.x            = 2.0 * cyl.radius;
        m.scale.y            = 2.0 * cyl.radius;
        m.scale.z            = cyl.height;
        color.r              = 0.2;
        color.g              = 0.7;
        color.b              = 0.9;
        color.a              = 0.7;
        m.color              = color;
        arr.markers.push_back(m);
      }
      return arr;
    }

    void publishState()
    {
      sensor_msgs::msg::PointCloud2 cloud = buildCloudMsg();
      cloud.header.stamp                  = now();
      points_pub_->publish(cloud);

      nav_msgs::msg::Odometry odom = buildOdomMsg();
      odom.header.stamp            = now();
      odom_pub_->publish(odom);

      visualization_msgs::msg::MarkerArray markers = buildMarkerMsg();
      markers_pub_->publish(markers);
    }

    std::unique_ptr<Dynamics>   dynamics_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<LidarSim>   lidar_;
    std::unique_ptr<Scene>      scene_;

    SimState                     state_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::size_t                  waypoint_index_ = 0;
    std::size_t                  frame_index_    = 0;
    double                       sim_dt_         = 0.005;
    std::string                  dump_pcd_path_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                                 markers_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
  };

}  // namespace simple_sim

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_sim::SimpleSimNode>());
  rclcpp::shutdown();
  return 0;
}
