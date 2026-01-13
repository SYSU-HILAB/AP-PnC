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
    : prev_alpha_(90.0)  // Default initial alpha at time=0
{
  loadData(csv_file);
}

double CzCalculate::getCz(double alpha_deg) const
{
  if (data_.empty())
  {
    std::cerr << "Error: No aerodynamic data loaded!" << std::endl;
    return 0.0;
  }

  // Boundary handling: below minimum
  if (alpha_deg <= data_.front().first)
  {
    return data_.front().second;
  }

  // Boundary handling: above maximum
  if (alpha_deg >= data_.back().first)
  {
    return data_.back().second;
  }

  // Binary search to find the interval containing alpha_deg
  auto upper =
      std::upper_bound(data_.begin(), data_.end(), alpha_deg,
                       [](double val, const std::pair<double, double>& p)
                       { return val < p.first; });

  auto lower = upper - 1;

  // Linear interpolation
  double alpha1 = lower->first;
  double cz1    = lower->second;
  double alpha2 = upper->first;
  double cz2    = upper->second;

  double slope    = (cz2 - cz1) / (alpha2 - alpha1);
  double cz_value = cz1 + slope * (alpha_deg - alpha1);

  return cz_value;
}

double CzCalculate::getAlpha(double cz) const
{
  if (data_.empty())
  {
    std::cerr << "Error: No aerodynamic data loaded!" << std::endl;
    return prev_alpha_;
  }

  // Tolerance for considering cz values as equal
  constexpr double CZ_TOLERANCE = 1e-6;

  // Find all alpha values that produce the given cz (within tolerance)
  std::vector<double> matching_alphas;

  for (const auto& [alpha_deg, cz_val] : data_)
  {
    if (std::abs(cz_val - cz) < CZ_TOLERANCE)
    {
      matching_alphas.push_back(alpha_deg);
    }
  }

  // If no exact match, find the closest cz values and interpolate
  if (matching_alphas.empty())
  {
    // Find the interval containing cz
    auto it = std::lower_bound(data_.begin(), data_.end(), cz,
                                [](const std::pair<double, double>& a, double val)
                                { return a.second < val; });

    if (it == data_.end())
    {
      // Above maximum - use last alpha
      prev_alpha_ = data_.back().first;
      return prev_alpha_;
    }

    if (it == data_.begin())
    {
      // Below minimum - use first alpha
      prev_alpha_ = data_.front().first;
      return prev_alpha_;
    }

    auto lower = it - 1;
    // Linear interpolation to get alpha
    double alpha1 = lower->first;
    double cz1    = lower->second;
    double alpha2 = it->first;
    double cz2    = it->second;

    double t = (cz - cz1) / (cz2 - cz1);
    double alpha = alpha1 + t * (alpha2 - alpha1);

    prev_alpha_ = alpha;
    return alpha;
  }

  // Select alpha closest to previous alpha (temporal coherence)
  double selected_alpha = matching_alphas[0];
  double min_diff      = std::abs(matching_alphas[0] - prev_alpha_);
  for (size_t i = 1; i < matching_alphas.size(); ++i)
  {
    double diff = std::abs(matching_alphas[i] - prev_alpha_);
    if (diff < min_diff)
    {
      min_diff      = diff;
      selected_alpha = matching_alphas[i];
    }
  }

  // Update prev_alpha_ for next iteration
  prev_alpha_ = selected_alpha;
  return selected_alpha;
}

void CzCalculate::resetAlpha(double initial_alpha)
{
  prev_alpha_ = initial_alpha;
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
