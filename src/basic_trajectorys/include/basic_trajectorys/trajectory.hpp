/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
 */
#pragma once
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>

namespace basic_trajectorys {
using PoseArray = geometry_msgs::msg::PoseArray;
using Vec3E = Eigen::Vector3d;
using Mat3x4 = Eigen::Matrix<double, 3, 4>;
enum class Type { UNDIFINED = 0, INIT};

class Trajectory {
 protected:
  double a = 2;
  double w = sqrt(2);
  double height = 5;

  Type __type;

  // pos vel jerk snap

 public:
  PoseArray& toMsg(int resolution = 50);
  std::vector<Mat3x4> samplingPointArray(int num = 20);

 public:
  Trajectory();
  virtual Vec3E pos(double t) = 0;
  virtual Vec3E vel(double t) = 0;
  virtual Vec3E acc(double t) = 0;
  virtual Vec3E jerk(double t) = 0;
  virtual Mat3x4 point(double t);
};

}  // namespace basic_trajectorys