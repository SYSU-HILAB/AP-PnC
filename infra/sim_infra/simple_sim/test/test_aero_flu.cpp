#include <aerodynamics/aero_factory.hpp>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
  void check(bool value, const char *message)
  {
    if (!value)
      throw std::runtime_error(message);
  }
}  // namespace

int main()
{
  try
  {
    // Public fitted-model regression at v_air_B=[2,.49,-2] m/s. These are
    // physical FLU wrenches, independently captured before the implementation
    // refactor. All three moment axes are checked, not only the pitch axis.
    const std::array<Eigen::Vector3d, 5> forces{
        Eigen::Vector3d(-1.5050342040399931, -.21021405032314355,
                        -.048402581427783038),
        Eigen::Vector3d(-1.6540263722304152, 0, .16540263722304155),
        Eigen::Vector3d(-1.3752711872715011, 0, -.25430145867372517),
        Eigen::Vector3d(-.80433511075311626, -.014002967927807586,
                        -.8043351107531076),
        Eigen::Vector3d::Zero()};
    const Eigen::Vector3d lyu_moment(.015302854651535242, .16771525213907904,
                                     .023181314851439112);
    const std::array<const char *, 5> names{"lyu", "phi", "ma", "advanced",
                                            "none"};
    const std::array<double, 9>       values{-12,  -2,  -.49, -1e-5, 0,
                                             1e-5, .49, 2,    12};
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      auto            model = aerodynamics::make_aero(names[i]);
      Eigen::Vector3d f, m;
      double          alpha = 100, beta = 100;
      check(model->getAeroWrench({2, .49, -2}, f, m, alpha, beta),
            "model failed");
      check((f - forces[i]).norm() < 1e-11, "FLU force polar regression");
      check(
          (m - (i == 0 ? lyu_moment : Eigen::Vector3d::Zero())).norm() < 1e-11,
          "FLU moment polar regression");
      for (double x : values)
        for (double y : values)
          for (double z : values)
          {
            alpha = beta = 100;
            check(model->getAeroWrench({x, y, z}, f, m, alpha, beta),
                  "model evaluation failed");
            check(f.allFinite() && m.allFinite(), "nonfinite model wrench");
            check(std::abs(alpha - std::atan2(x, z)) < 1e-14,
                  "model alpha differs from FLU kinematic angle");
            check(std::abs(beta - std::atan2(y, std::hypot(x, z))) < 1e-14,
                  "model beta differs from FLU kinematic angle");
          }
      for (double invalid : {std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
      {
        bool rejected = false;
        try
        {
          rejected = !model->getAeroWrench({invalid, 0, 0}, f, m, alpha, beta);
        }
        catch (const std::invalid_argument &)
        {
          rejected = true;
        }
        check(rejected, "nonfinite air velocity accepted");
      }
    }
    std::cout << "PASS 3645 native-FLU angle/wrench cases, fitted polar "
                 "fixtures, invalid inputs\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
