/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
 */

#include <basic_trajectorys/circle.hpp>
namespace basic_trajectorys {

Circle::Circle(double a_in, double w_in, double height_in) {
  a = a_in;
  w = w_in;
  height = height_in;
  __type = basic_trajectorys::Type::INIT;
}

Vec3E Circle::pos(double t) {
  static Vec3E pos = Vec3E::Zero();
  // only change first two, i.e. x and y;
  pos[0] = a * cos(w * t);
  pos[1] = a * sin(w * t);
  pos[2] = height;
  return pos;
}

Vec3E Circle::vel(double t) {
  static Vec3E vel = Vec3E::Zero();
  // only change first two, i.e. x and y;
  vel[0] = -a * w * sin(w * t);
  vel[1] = a * w * (cos(w * t) * cos(w * t) - sin(w * t) * sin(w * t));
  return vel;
}
Vec3E Circle::acc(double t) {
  static Vec3E acc = Vec3E::Zero();
  // only change first two, i.e. x and y;
  acc[0] = -a * w * w * cos(w * t);
  acc[1] = -4 * a * w * (cos(w * t) * sin(w * t));
  return acc;
}

Vec3E Circle::jerk(double t) {
  static Vec3E jerk = Vec3E::Zero();
  // only change first two, i.e. x and y;
  jerk[0] = a * w * w * w * sin(w * t);
  jerk[1] =
      4 * a * w * w * w * (sin(w * t) * sin(w * t) - cos(w * t) * cos(w * t));
  return jerk;
}

}  // namespace basic_trajectorys