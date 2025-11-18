/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.
 */

#include <basic_trajectories/sin.hpp>
#include <basic_trajectories/trajectory.hpp>
#include <basic_trajectories/vertical_loop.hpp>
#include <fstream>
#include <iostream>
void generateTrajectoryCSV(basic_trajectories::Trajectory& trajectory,
                           const std::string& filename, int resolution = 100)
{
  std::ofstream csvFile(filename);
  csvFile << "time,x,y,z,vx,vy,vz\n";
  auto traj_points = trajectory.samplingPointArray(100);

  for (int i = 0; i < traj_points.size(); ++i)
  {
    auto position = traj_points[i].col(0);
    auto velocity = traj_points[i].col(1);

    csvFile << i << "," << position.x() << "," << position.y() << ","
            << position.z() << "," << velocity.x() << "," << velocity.y() << ","
            << velocity.z() << "\n";
  }
}

int main()
{
  // Example usage
  basic_trajectories::Sin sinTrajectory(1.0, 2.0, 5.0);
  generateTrajectoryCSV(sinTrajectory, "sin_trajectory.csv");

  basic_trajectories::VerticalLoop verticalLoop(1.0, 0.1, 5.0);
  generateTrajectoryCSV(verticalLoop, "vertical_loop_trajectory.csv");

  return 0;
}
// Note: The concrete trajectory class implementation would be elsewhere
