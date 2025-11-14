/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * author: WarriorHanamy ; email: rongerch@outlook.com
 */

#pragma once
#include <basic_trajectories/trajectory.hpp>
namespace basic_trajectories
{

    class Circle : public Trajectory
    {
       public:
        Circle(/* args */) = delete;
        explicit Circle(double a, double w, double height = 5);
        Vec3E pos(double t) override;
        Vec3E vel(double t) override;
        Vec3E acc(double t) override;
        Vec3E jerk(double t) override;
        // VecXE point(double t) override;
    };

}  // namespace basic_trajectories
