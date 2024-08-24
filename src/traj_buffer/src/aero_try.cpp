/*
 * ########################################################################
 * The contents of this file is free and unencumbered software released into the
 * public domain. For more information, please refer to <http://unlicense.org/>
 * ########################################################################
 */

// #include <bspline/Core.h>

// #include <fstream>
// #include <iostream>
// #include <limits>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <vector>

// class BsplineAeroDyn
// {
//     static constexpr size_t SPLINE_DEGREE = 3;
//     using Spline = bspline::Spline<double, SPLINE_DEGREE>;

//    private:
//     std::vector<double> coefs_;
//     std::vector<Spline> splines_;

//    public:
//     BsplineAeroDyn()
//     {
//         const std::vector<double> knots{
//             0.5235987755982989, 0.5235987755982989, 0.5235987755982989,
//             0.5235987755982989, 0.7853981633974483, 0.8726646259971648,
//             0.9599310885968813, 1.0471975511965976, 1.1344640137963142,
//             1.3089969389957472, 1.8325957145940461, 2.2689280275926285,
//             2.617993877991494,  2.617993877991494,  2.617993877991494,
//             2.617993877991494};
//         splines_ = bspline::generateBSplines<SPLINE_DEGREE>(knots);
//         coefs_.resize(splines_.size(), 0.0);
//     }
//     bool setCoefs(std::vector<double> coefs)
//     {
//         if (coefs.size() != coefs_.size())
//         {
//             return false;
//         }
//         for (auto i = 0; i < coefs.size(); i++)
//         {
//             coefs_[i] = coefs[i];
//         }
//         return true;
//     }
//     double getAeroDyn(double x)
//     {
//         double val = 0;
//         for (size_t i = 0; i < splines_.size(); i++)
//         {
//             val += splines_[i](x) * coefs_[i];
//         }
//         return val;
//     }
// };
#include <cassert>
int main(int argc, char **argv)
{
    assert(5 == 10);
    // std::vector<double> coefs{
    //     0.08869374325729838, 0.11713376801189856, 0.12567841873810845,
    //     0.15005373297969835, 0.1324185630496418,  0.24883612641861838,
    //     0.20757431730331458, 0.5001433323539398,  0.3877382239367887,
    //     0.08974898500006634, 0.14520286916801822, 0.08072293755821285};
    // // Generate Splines.
    // BsplineAeroDyn aeroDyn;
    // aeroDyn.setCoefs(coefs);
    // std::ofstream out("aero-test.csv");
    // out.precision(std::numeric_limits<double>::max_digits10);
    // for (double x = 0.3; x <= 2.8; x += 0.01)
    // {
    //     out << x << ",";
    //     out << aeroDyn.getAeroDyn(x) << "\n";
    // }
}