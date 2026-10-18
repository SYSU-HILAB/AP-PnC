#include <gcopter/geo_utils.hpp>
#include <gcopter/sfc_gen.hpp>
#include <iostream>
#include <stdexcept>

int main()
{
  try
  {
    // Real city corridor regression: tiny axis rotation terms used to collapse
    // the dual hull to one/nonfinite vertex, subsequently overflowing flatness.
    Eigen::MatrixX4d poly(7, 4);
    poly << -1, 5.42414e-17, -3.64424e-16, 9.075, 3.53583e-16, 2.27466e-16, 1,
        -8.875, -1.40841e-16, 1, 2.92851e-16, 0.925, -3.53583e-16, -2.27466e-16,
        -1, 2.875, 1, -5.42414e-17, 3.64424e-16, -15.425, 1.40841e-16, -1,
        -2.92851e-16, -6.925, -0.0237035, -0.29644, -0.192476, -0.742255;
    for (double offset : {0.0, 40.0, -120.0})
    {
      auto shifted = poly;
      shifted.col(3) -= poly.leftCols<3>() * Eigen::Vector3d::Constant(offset);
      Eigen::Matrix3Xd vertices;
      if (!geo_utils::enumerateVs(shifted, vertices) || vertices.cols() < 4 ||
          !vertices.allFinite())
        throw std::runtime_error("corridor vertex enumeration degenerated");
      for (int i = 0; i < vertices.cols(); ++i)
        if ((shifted.leftCols<3>() * vertices.col(i) + shifted.col(3))
                .maxCoeff() > 1e-6)
          throw std::runtime_error("enumerated vertex outside corridor");
    }
    const std::vector<Eigen::Vector3d> path{{0, 0, 3}, {1, 0, 3}};
    const std::vector<Eigen::Vector3d> remote_surface{{100, 100, 100}};
    std::vector<Eigen::MatrixX4d>      corridor;
    sfc_gen::convexCover(path, remote_surface, {-10, -10, -10}, {10, 10, 10}, 6,
                         3, corridor);
    if (corridor.empty() || !corridor.front().allFinite())
      throw std::runtime_error(
          "empty local obstacle set did not produce a bounding box");
    std::cout << "city corridor geometry passed\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
