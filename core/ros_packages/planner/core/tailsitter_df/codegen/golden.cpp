// Golden oracle: compile against the unchanged C++/autodiff implementation.
#include <iomanip>
#include <iostream>
#include <tailsitter_df/tailsitter_df.hpp>

int main()
{
  Eigen::Vector3d v, a, j;
  std::cout << std::setprecision(17);
  while (std::cin >> v.x() >> v.y() >> v.z() >> a.x() >> a.y() >> a.z() >>
         j.x() >> j.y() >> j.z())
  {
    double          thrust;
    Eigen::Vector3d omega;
    const auto      jac = tailsitter_df::Data::getInstance().df_backward_auto(
        v, a, j, thrust, omega);
    std::cout << thrust;
    for (int i = 0; i < 3; ++i)
      std::cout << ' ' << omega[i];
    // Match the generated C ABI's column-major layout.
    for (int col = 0; col < 9; ++col)
      for (int row = 0; row < 4; ++row)
        std::cout << ' ' << jac(row, col);
    std::cout << '\n';
  }
}
