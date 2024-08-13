/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * Author: WarriorHanamy ; Email: rongerch@mail2.sysu.edu.cn
 */

#include <basic_trajectories/lemniscate.hpp>

namespace basic_trajectories
{

    Lemniscate::Lemniscate(double radius_in, double angular_speed_in,
                           double height_in, double curvature_factor_in)
    {
        radius           = radius_in;
        angular_speed    = angular_speed_in;
        height           = height_in;
        type             = basic_trajectories::Type::INIT;
        curvature_factor = curvature_factor_in;
    }

    Vec3E Lemniscate::pos(double t)
    {
        Vec3E pos = Vec3E::Zero();
        // Only change first two, i.e., x and y;
        pos[0] = radius * cos(angular_speed * t) /
                 (1 + curvature_factor * sin(angular_speed * t) *
                          sin(angular_speed * t));
        pos[1] = radius * sin(angular_speed * t) * cos(angular_speed * t) /
                 (1 + curvature_factor * sin(angular_speed * t) *
                          sin(angular_speed * t));
        pos[2] = height;
        return pos;
    }

    Vec3E Lemniscate::vel(double t)
    {
        Vec3E vel = Vec3E::Zero();
        // Only change first two, i.e., x and y;
        vel[0] = -radius * angular_speed * sin(angular_speed * t);
        vel[1] = radius * angular_speed *
                 (cos(angular_speed * t) * cos(angular_speed * t) -
                  sin(angular_speed * t) * sin(angular_speed * t));
        return vel;
    }

    Vec3E Lemniscate::acc(double t)
    {
        Vec3E acc = Vec3E::Zero();
        // Only change first two, i.e., x and y;
        acc[0] =
            -radius * angular_speed * angular_speed * cos(angular_speed * t);
        acc[1] = -4 * radius * angular_speed *
                 (cos(angular_speed * t) * sin(angular_speed * t));
        return acc;
    }

    Vec3E Lemniscate::jerk(double t)
    {
        Vec3E jerk = Vec3E::Zero();
        // Only change first two, i.e., x and y;
        jerk[0] = radius * angular_speed * angular_speed * angular_speed *
                  sin(angular_speed * t);
        jerk[1] = 4 * radius * angular_speed * angular_speed * angular_speed *
                  (sin(angular_speed * t) * sin(angular_speed * t) -
                   cos(angular_speed * t) * cos(angular_speed * t));
        return jerk;
    }

}  // namespace basic_trajectories