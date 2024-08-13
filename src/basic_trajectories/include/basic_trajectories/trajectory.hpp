/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * Author: WarriorHanamy ; Email: rongerch@mail2.sysu.edu.cn
 */
#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>

/**
 * @brief Interface for more complex trajectories
 *
 */
namespace basic_trajectories
{
    using PoseArray = geometry_msgs::msg::PoseArray;
    using Vec3E     = Eigen::Vector3d;
    using Mat3x4    = Eigen::Matrix<double, 3, 4>;

    /**
     * @brief Reserved for future use
     *
     */
    enum class Type
    {
        UNDEFINED = 0,
        INIT
    };

    class Trajectory
    {
       protected:
        /**
         * @brief The radius of the trajectory or named as the length of the
         * trajectory
         *
         */
        double radius = 2.0;

        /**
         * @brief The wave length of the trajectory, typically double the width.
         *
         */
        double wave_len = 10.0;

        /**
         * @brief The angular speed (rad/s) of the trajectory
         *
         */
        double angular_speed = sqrt(2.0);

        /**
         * @brief The vertical height of the trajectory
         *
         */
        double height = 5.0;

        /**
         * @brief Reserved for future use
         *
         */
        Type type;

       public:
        /**
         * @brief Transforms the trajectory to a PoseArray message
         *
         * @param resolution The time segments of the whole trajectory
         * @return PoseArray&
         */
        PoseArray& toMsg(int resolution = 50);

        /**
         * @brief Sampling points of the trajectory
         *
         * @param num The number of sampling points while the starting point is
         * not counted.
         * @return std::vector<Mat3x4>
         */
        std::vector<Mat3x4> samplingPointArray(int num = 20);

       public:
        Trajectory();
        /**
         * @brief Get the rate of the trajector
         *
         * @return double
         */
        double getAngularSpeed() { return angular_speed; }
        /**
         * @brief Get the position of the trajectory at time t
         *
         * @param t Time parameter
         * @return Vec3E (alias for Eigen::Vector3d)
         */
        virtual Vec3E pos(double t) = 0;

        /**
         * @brief Get the velocity of the trajectory at time t
         *
         * @param t Time parameter
         * @return Vec3E (alias for Eigen::Vector3d)
         */
        virtual Vec3E vel(double t) = 0;

        /**
         * @brief Get the acceleration of the trajectory at time t
         *
         * @param t Time parameter
         * @return Vec3E (alias for Eigen::Vector3d)
         */
        virtual Vec3E acc(double t) = 0;

        /**
         * @brief Get the jerk of the trajectory at time t
         *
         * @param t Time parameter
         * @return Vec3E (alias for Eigen::Vector3d)
         */
        virtual Vec3E jerk(double t) = 0;

        /**
         * @brief Get the aggregate form of position, velocity, acceleration,
         * and jerk of the trajectory at time t
         *
         * @param t Time parameter
         * @return Mat3x4 (alias for Eigen::Matrix<double, 3, 4>)
         */
        virtual Mat3x4 point(double t);
    };

}  // namespace basic_trajectories