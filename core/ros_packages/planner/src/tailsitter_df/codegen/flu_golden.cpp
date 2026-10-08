#include <iomanip>
#include <iostream>
#include <tailsitter_df/flu_reference.hpp>

int main()
{
  Eigen::Vector3d v, a, j;
  std::cout << std::setprecision(17);
  while (std::cin >> v.x() >> v.y() >> v.z() >> a.x() >> a.y() >> a.z() >>
         j.x() >> j.y() >> j.z())
  {
    const auto f = tailsitter_df::flu_reference(v, a, j);
    std::cout << f.specific_thrust;
    for (int i = 0; i < 3; ++i)
      std::cout << ' ' << f.omega[i];
    for (int i = 0; i < 3; ++i)
      for (int k = 0; k < 3; ++k)
        std::cout << ' ' << f.rotation(i, k);
    std::cout << '\n';
  }
}
