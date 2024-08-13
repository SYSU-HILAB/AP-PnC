#include <cmath>
#include <eigen3/Eigen/Dense>

namespace fit_data
{
    // double a0,a1,
    // trancated fourier series 4 order
    double a0, w;
    double a1, a2, a3, a4;
    double b1, b2, b3, b4;
    double kPho;
    double kS;
    double mass;
};  // namespace fit_data

double get_cz_from_vb(const Eigen::Vector3d &vb);