/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

/*
 * Created on Wed Sep 13 2023
 *
 * Copyright (c) 2023 SYSU
 * Authors:
 * Erchao Rong: rongerch@outlook.com
 * Zihao Liu: liuzh297@gmail.com
 * Junning Liang: gordonliang27@foxmail.com
 */
#pragma once

#include <Eigen/Geometry>

/**
 * @brief Interface for more complex trajectories
 *
 */
namespace basic_trajectories
{
  using Vec3E  = Eigen::Vector3d;
  using Mat3x4 = Eigen::Matrix<double, 3, 4>;

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
     * @brief Sampling points of the trajectory
     *
     * @param num The number of sampling points while the starting point is
     * not counted.
     * @return std::vector<Mat3x4>
     */
    std::vector<Mat3x4> samplingPointArray(int num = 20);

   public:
    Trajectory();
    virtual ~Trajectory() = default;
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
