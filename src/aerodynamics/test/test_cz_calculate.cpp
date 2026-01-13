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

#include <fstream>
#include <gtest/gtest.h>
#include <string>

using namespace aerodynamics;

/**
 * @brief Create a temporary CSV file for testing
 */
std::string createTestCSV(const std::string& filename, const std::vector<std::pair<double, double>>& data)
{
  std::ofstream file(filename);

  file << "alpha_deg,cx,cz\n";  // Header
  for (const auto& [alpha, cz] : data)
  {
    // Add a dummy cx value (0.0)
    file << alpha << ",0.0," << cz << "\n";
  }
  file.close();

  return filename;
}

/**
 * @brief Test: Load valid CSV file and perform lookup
 */
TEST(CzCalculate, LoadValidCSV)
{
  // Create test data: linear relationship cz = 0.1 * alpha
  std::vector<std::pair<double, double>> test_data;
  for (int alpha = -180; alpha <= 180; alpha += 10)
  {
    test_data.push_back({static_cast<double>(alpha), 0.1 * alpha});
  }

  std::string filename = "/tmp/test_cz_calculate_valid.csv";
  createTestCSV(filename, test_data);

  CzCalculate calc(filename);

  // Test exact lookup at known points
  auto result1 = calc.getCzWithSlope(0.0);
  EXPECT_NEAR(result1.cz_value, 0.0, 1e-6);
  EXPECT_NEAR(result1.slope, 0.1, 1e-6);

  auto result2 = calc.getCzWithSlope(90.0);
  EXPECT_NEAR(result2.cz_value, 9.0, 1e-6);

  auto result3 = calc.getCzWithSlope(-90.0);
  EXPECT_NEAR(result3.cz_value, -9.0, 1e-6);

  // Cleanup
  std::remove(filename.c_str());
}

/**
 * @brief Test: Boundary values
 */
TEST(CzCalculate, BoundaryValues)
{
  // Create test data from -90 to 90 degrees
  std::vector<std::pair<double, double>> test_data;
  for (int alpha = -90; alpha <= 90; alpha += 10)
  {
    test_data.push_back({static_cast<double>(alpha), 0.1 * alpha});
  }

  std::string filename = "/tmp/test_cz_calculate_boundary.csv";
  createTestCSV(filename, test_data);
  CzCalculate calc(filename);

  // Test below minimum (extrapolation should use boundary slope)
  auto result1 = calc.getCzWithSlope(-100.0);
  EXPECT_NEAR(result1.cz_value, -10.0, 1e-6);
  EXPECT_NEAR(result1.slope, 0.1, 1e-6);

  // Test above maximum
  auto result2 = calc.getCzWithSlope(100.0);
  EXPECT_NEAR(result2.cz_value, 10.0, 1e-6);
  EXPECT_NEAR(result2.slope, 0.1, 1e-6);

  // Test at exact boundaries
  auto result3 = calc.getCzWithSlope(-90.0);
  EXPECT_NEAR(result3.cz_value, -9.0, 1e-6);

  auto result4 = calc.getCzWithSlope(90.0);
  EXPECT_NEAR(result4.cz_value, 9.0, 1e-6);

  // Cleanup
  std::remove(filename.c_str());
}

/**
 * @brief Test: Interpolation accuracy
 */
TEST(CzCalculate, InterpolationAccuracy)
{
  // Create test data with quadratic relationship: cz = 0.01 * alpha^2
  std::vector<std::pair<double, double>> test_data;
  for (int alpha = -180; alpha <= 180; alpha += 30)
  {
    test_data.push_back({static_cast<double>(alpha), 0.01 * alpha * alpha});
  }

  std::string filename = "/tmp/test_cz_calculate_interp.csv";
  createTestCSV(filename, test_data);
  CzCalculate calc(filename);

  // Test interpolation between points
  auto result1 = calc.getCzWithSlope(-165.0);
  EXPECT_GT(result1.cz_value, 22.5);
  EXPECT_LT(result1.cz_value, 32.4);

  // Test at a known data point (should be exact)
  auto result2 = calc.getCzWithSlope(-90.0);
  EXPECT_NEAR(result2.cz_value, 81.0, 1e-6);

  // Test that getCz() returns same value as getCzWithSlope().cz_value
  double cz1 = calc.getCz(45.0);
  double cz2 = calc.getCzWithSlope(45.0).cz_value;
  EXPECT_NEAR(cz1, cz2, 1e-10);

  // Cleanup
  std::remove(filename.c_str());
}

/**
 * @brief Test: Binary search correctness with sparse data
 */
TEST(CzCalculate, BinarySearchCorrectness)
{
  // Create sparse test data to test binary search
  std::vector<std::pair<double, double>> test_data;
  test_data.push_back({-180.0, -1.0});
  test_data.push_back({-90.0, -0.5});
  test_data.push_back({0.0, 0.0});
  test_data.push_back({90.0, 0.5});
  test_data.push_back({180.0, 1.0});

  std::string filename = "/tmp/test_cz_calculate_binary.csv";
  createTestCSV(filename, test_data);
  CzCalculate calc(filename);

  // Test interpolation between sparse points
  auto result1 = calc.getCzWithSlope(-135.0);  // Midpoint between -180 and -90
  EXPECT_NEAR(result1.cz_value, -0.75, 1e-6);

  auto result2 = calc.getCzWithSlope(45.0);  // Midpoint between 0 and 90
  EXPECT_NEAR(result2.cz_value, 0.25, 1e-6);

  // Cleanup
  std::remove(filename.c_str());
}

/**
 * @brief Test: Fine-grained lookup with realistic aerodynamic data
 */
TEST(CzCalculate, FineGrainedLookup)
{
  // Create more realistic aerodynamic data
  // Using a simplified lift curve: cz = 2*pi*sin(alpha) for small angles
  std::vector<std::pair<double, double>> test_data;
  constexpr double deg_to_rad = M_PI / 180.0;

  for (int alpha = -180; alpha <= 180; alpha++)
  {
    double alpha_rad = alpha * deg_to_rad;
    double cz = 2.0 * M_PI * std::sin(alpha_rad);
    test_data.push_back({static_cast<double>(alpha), cz});
  }

  std::string filename = "/tmp/test_cz_calculate_fine.csv";
  createTestCSV(filename, test_data);
  CzCalculate calc(filename);

  // Test lookup at various angles
  auto result1 = calc.getCzWithSlope(0.0);
  EXPECT_NEAR(result1.cz_value, 0.0, 0.1);

  auto result2 = calc.getCzWithSlope(15.0);
  EXPECT_NEAR(result2.cz_value, 1.63, 0.1);

  auto result3 = calc.getCzWithSlope(-15.0);
  EXPECT_NEAR(result3.cz_value, -1.63, 0.1);

  // Test slope calculation - slope should be positive around 45°
  auto result4 = calc.getCzWithSlope(45.0);
  EXPECT_GT(result4.slope, 0.0);

  // Cleanup
  std::remove(filename.c_str());
}

/**
 * @brief Test: Invalid file handling
 */
TEST(CzCalculate, InvalidFile)
{
  // Test with non-existent file
  CzCalculate calc("/nonexistent/file.csv");

  // Should return default values without crashing
  auto result = calc.getCzWithSlope(0.0);
  EXPECT_EQ(result.cz_value, 0.0);
  EXPECT_EQ(result.slope, 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
