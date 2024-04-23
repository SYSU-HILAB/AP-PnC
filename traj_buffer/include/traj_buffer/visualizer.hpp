#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "gcopter/geo_utils.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// Visualizer for the planner
class Visualizer {
 private:
  // config contains the scale for some markers
  rclcpp::Node *nh;
  // These are publishers for path, waypoints on the trajectory,
  // the entire trajectory, the mesh of free-space polytopes,
  // the edge of free-space polytopes, and spheres for safety radius
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uavMeshPub;

 public:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tiltPub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bdrPub;

 public:
  explicit Visualizer(rclcpp::Node *nh_) : nh(nh_) {
    auto reliable_qos =
        rclcpp::QoS(1).reliable().transient_local().keep_last(1);
    routePub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/route", reliable_qos);
    wayPointsPub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/waypoints", 10);
    trajectoryPub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/trajectory", reliable_qos);
    meshPub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/mesh", 1000);
    edgePub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/edge", 1000);
    spherePub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/spheres", 1000);
    uavMeshPub = nh->create_publisher<visualization_msgs::msg::Marker>(
        "/visualizer/uav_traj", 10);
    speedPub =
        nh->create_publisher<std_msgs::msg::Float64>("/visualizer/speed", 1000);
    thrPub = nh->create_publisher<std_msgs::msg::Float64>(
        "/visualizer/total_thrust", 1000);
    tiltPub = nh->create_publisher<std_msgs::msg::Float64>(
        "/visualizer/tilt_angle", 1000);
    bdrPub = nh->create_publisher<std_msgs::msg::Float64>(
        "/visualizer/body_rate", 1000);
  }

  // Visualize the trajectory and its front-end path
  template <int D>
  inline void visualize(const Trajectory<D> &traj,
                        const std::vector<Eigen::Vector3d> &route) {
    visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    routeMarker.header.stamp = nh->get_clock()->now();
    routeMarker.header.frame_id = "world";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::msg::Marker::ADD;
    routeMarker.ns = "route";
    routeMarker.color.r = 1.00;
    routeMarker.color.g = 0.00;
    routeMarker.color.b = 0.00;
    routeMarker.color.a = 1.00;
    routeMarker.scale.x = 0.1;

    wayPointsMarker = routeMarker;
    wayPointsMarker.id = -wayPointsMarker.id - 1;
    wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 1.00;
    wayPointsMarker.color.g = 0.00;
    wayPointsMarker.color.b = 0.00;
    wayPointsMarker.scale.x = 0.35;
    wayPointsMarker.scale.y = 0.35;
    wayPointsMarker.scale.z = 0.35;

    trajMarker = routeMarker;
    trajMarker.header.frame_id = "world";
    trajMarker.id = 0;
    trajMarker.ns = "trajectory";
    trajMarker.color.r = 0.00;
    trajMarker.color.g = 0.50;
    trajMarker.color.b = 1.00;

    if (route.size() > 0) {
      bool first = true;
      Eigen::Vector3d last;
      for (auto it : route) {
        if (first) {
          first = false;
          last = it;
          continue;
        }
        geometry_msgs::msg::Point point;

        point.x = last(0);
        point.y = last(1);
        point.z = last(2);
        routeMarker.points.push_back(point);
        point.x = it(0);
        point.y = it(1);
        point.z = it(2);
        routeMarker.points.push_back(point);
        last = it;
      }

      routePub->publish(routeMarker);
    }

    if (traj.getPieceNum() > 0) {
      Eigen::MatrixXd wps = traj.getPositions();
      for (int i = 0; i < wps.cols(); i++) {
        geometry_msgs::msg::Point point;
        point.x = wps.col(i)(0);
        point.y = wps.col(i)(1);
        point.z = wps.col(i)(2);
        wayPointsMarker.points.push_back(point);
      }

      wayPointsPub->publish(wayPointsMarker);
    }

    if (traj.getPieceNum() > 0) {
      double T = 0.01;
      Eigen::Vector3d lastX = traj.getPos(0.0);
      for (double t = T; t < traj.getTotalDuration(); t += T) {
        geometry_msgs::msg::Point point;
        Eigen::Vector3d X = traj.getPos(t);
        point.x = lastX(0);
        point.y = lastX(1);
        point.z = lastX(2);
        trajMarker.points.push_back(point);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        trajMarker.points.push_back(point);
        lastX = X;
      }
      trajectoryPub->publish(trajMarker);
    }
  }

  // Visualize some polytopes in H-representation
  inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys) {
    // Due to the fact that H-representation cannot be directly visualized
    // We first conduct vertex enumeration of them, then apply quickhull
    // to obtain triangle meshs of polyhedra

    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < hPolys.size(); id++) {
      oldTris = mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
      geo_utils::enumerateVs(hPolys[id], vPoly);
      quickhull::QuickHull<double> tinyQH;
      const auto polyHull =
          tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
      const auto &idxBuffer = polyHull.getIndexBuffer();
      int hNum = idxBuffer.size() / 3;

      curTris.resize(3, hNum * 3);
      for (int i = 0; i < hNum * 3; i++) {
        curTris.col(i) = vPoly.col(idxBuffer[i]);
      }
      mesh.resize(3, oldTris.cols() + curTris.cols());
      mesh.leftCols(oldTris.cols()) = oldTris;
      mesh.rightCols(curTris.cols()) = curTris;
    }

    // RVIZ support tris for visualization
    visualization_msgs::msg::Marker meshMarker, edgeMarker;

    meshMarker.id = 0;
    meshMarker.header.stamp = nh->get_clock()->now();
    meshMarker.header.frame_id = "world";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;
    meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.15;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 1.00;
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::msg::Point point;

    int ptnum = mesh.cols();

    for (int i = 0; i < ptnum; i++) {
      point.x = mesh(0, i);
      point.y = mesh(1, i);
      point.z = mesh(2, i);
      meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++) {
      for (int j = 0; j < 3; j++) {
        point.x = mesh(0, 3 * i + j);
        point.y = mesh(1, 3 * i + j);
        point.z = mesh(2, 3 * i + j);
        edgeMarker.points.push_back(point);
        point.x = mesh(0, 3 * i + (j + 1) % 3);
        point.y = mesh(1, 3 * i + (j + 1) % 3);
        point.z = mesh(2, 3 * i + (j + 1) % 3);
        edgeMarker.points.push_back(point);
      }
    }

    meshPub->publish(meshMarker);
    edgePub->publish(edgeMarker);

    return;
  }

  // Visualize all spheres with centers sphs and the same radius
  inline void visualizeSphere(const Eigen::Vector3d &center,
                              const double &radius) {
    visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = 0;
    sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = nh->get_clock()->now();
    sphereMarkers.header.frame_id = "world";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
    sphereMarkers.ns = "spheres";
    sphereMarkers.color.r = 0.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 1.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::msg::Marker::DELETE;

    geometry_msgs::msg::Point point;
    point.x = center(0);
    point.y = center(1);
    point.z = center(2);
    sphereMarkers.points.push_back(point);

    spherePub->publish(sphereDeleter);
    spherePub->publish(sphereMarkers);
  }

  // Visualize uav attitude trajectory
  inline void visualizeUAVTraj(const Eigen::Vector3d &pos,
                               const Eigen::Vector4d &quat,
                               double scale = 3.0) {
    visualization_msgs::msg::Marker uavMarker;

    uavMarker.id = 0;
    uavMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    uavMarker.mesh_resource =
        "package://gcopter/meshes/tailsitter/meshes/tailsitter_fuselage.dae";
    uavMarker.header.stamp = nh->get_clock()->now();
    uavMarker.header.frame_id = "world";
    uavMarker.pose.position.x = pos(0);
    uavMarker.pose.position.y = pos(1);
    uavMarker.pose.position.z = pos(2);
    uavMarker.pose.orientation.w = quat(0);
    uavMarker.pose.orientation.x = quat(1);
    uavMarker.pose.orientation.y = quat(2);
    uavMarker.pose.orientation.z = quat(3);
    uavMarker.action = visualization_msgs::msg::Marker::ADD;
    uavMarker.ns = "uav_traj";
    uavMarker.color.r = 1.00;
    uavMarker.color.g = 1.00;
    uavMarker.color.b = 1.00;
    uavMarker.color.a = 1.00;
    uavMarker.scale.x = scale;
    uavMarker.scale.y = scale;
    uavMarker.scale.z = scale;

    uavMeshPub->publish(uavMarker);
  }

  inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                 const double &radius, const int sg) {
    visualization_msgs::msg::Marker sphereMarkers, sphereDeleter;

    sphereMarkers.id = sg;
    sphereMarkers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphereMarkers.header.stamp = nh->get_clock()->now();
    sphereMarkers.header.frame_id = "world";
    sphereMarkers.pose.orientation.w = 1.00;
    sphereMarkers.action = visualization_msgs::msg::Marker::ADD;
    sphereMarkers.ns = "StartGoal";
    sphereMarkers.color.r = 1.00;
    sphereMarkers.color.g = 0.00;
    sphereMarkers.color.b = 0.00;
    sphereMarkers.color.a = 1.00;
    sphereMarkers.scale.x = radius * 2.0;
    sphereMarkers.scale.y = radius * 2.0;
    sphereMarkers.scale.z = radius * 2.0;

    sphereDeleter = sphereMarkers;
    sphereDeleter.action = visualization_msgs::msg::Marker::DELETEALL;

    geometry_msgs::msg::Point point;
    point.x = center(0);
    point.y = center(1);
    point.z = center(2);
    sphereMarkers.points.push_back(point);
    if (sg == 0) {
      spherePub->publish(sphereDeleter);
      rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(1ns));
      sphereMarkers.header.stamp = nh->get_clock()->now();
    }
    spherePub->publish(sphereMarkers);
  }

  inline void visualizeFloat(double data, const std::string topic) {
    static std::unordered_map<
        std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>>>
        pubContainer;
    static std_msgs::msg::Float64 msg_float;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_ptr;

    auto it = pubContainer.find(topic);
    if (it == pubContainer.end()) {
      pub_ptr = nh->create_publisher<std_msgs::msg::Float64>(topic, 10);
      pubContainer[topic] = pub_ptr;
    } else {
      pub_ptr = it->second;
    }

    msg_float.data = data;

    pub_ptr->publish(msg_float);
  }
};

#endif