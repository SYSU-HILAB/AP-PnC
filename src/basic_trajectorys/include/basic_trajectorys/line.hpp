/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
 */

#pragma once
#include <basic_trajectorys/trajectory.hpp>
namespace basic_trajectorys {

class Line : public Trajectory {
public:
  Line(/* args */) = delete;
  // in Line Setting, a is the total length, 2pi/w is the vel
  explicit Line(double a, double w, double height = 5);
  Vec3E pos(double t) override;
  Vec3E vel(double t) override;
  Vec3E acc(double t) override;
  Vec3E jerk(double t) override;
};

}  // namespace basic_trajectorys