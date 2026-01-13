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

#include <aerodynamics/cz_calculate.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace aerodynamics
{

CzCalculate::CzCalculate(const std::string& csv_file)
{
  loadData(csv_file);
}

CzCalculate::CzResult CzCalculate::getCzWithSlope(double alpha_deg) const
{
  if (data_.empty())
  {
    std::cerr << "Error: No aerodynamic data loaded!" << std::endl;
    return {0.0, 0.0};
  }

  // Boundary handling: below minimum
  if (alpha_deg <= data_.front().first)
  {
    double slope = (data_.size() > 1)
                       ? (data_[1].second - data_[0].second) /
                             (data_[1].first - data_[0].first)
                       : 0.0;
    return {data_.front().second, slope};
  }

  // Boundary handling: above maximum
  if (alpha_deg >= data_.back().first)
  {
    double slope =
        (data_.size() > 1)
            ? (data_.back().second - data_[data_.size() - 2].second) /
                  (data_.back().first - data_[data_.size() - 2].first)
            : 0.0;
    return {data_.back().second, slope};
  }

  // Binary search to find the interval containing alpha_deg
  auto upper =
      std::upper_bound(data_.begin(), data_.end(), alpha_deg,
                       [](double val, const std::pair<double, double>& p)
                       { return val < p.first; });

  auto lower = upper - 1;  // Previous element

  // Linear interpolation
  double alpha1 = lower->first;
  double cz1    = lower->second;
  double alpha2 = upper->first;
  double cz2    = upper->second;

  double slope    = (cz2 - cz1) / (alpha2 - alpha1);
  double cz_value = cz1 + slope * (alpha_deg - alpha1);

  return {cz_value, slope};
}

double CzCalculate::getCz(double alpha_deg) const
{
  return getCzWithSlope(alpha_deg).cz_value;
}

void CzCalculate::loadData(const std::string& csv_file)
{
  std::ifstream file(csv_file);
  if (!file.is_open())
  {
    std::cerr << "Error: Unable to open CSV file: " << csv_file << std::endl;
    return;
  }

  std::string line;
  bool first_line = true;
  while (std::getline(file, line))
  {
    // Skip header line
    if (first_line)
    {
      first_line = false;
      continue;
    }

    std::stringstream ss(line);
    double alpha, cx, cz;
    char comma;

    // Parse: alpha,cx,cz
    if (ss >> alpha >> comma >> cx >> comma >> cz)
    {
      data_.emplace_back(alpha, cz);
    }
  }

  file.close();

  // Ensure data is sorted by alpha
  std::sort(data_.begin(), data_.end());
}

}  // namespace aerodynamics
