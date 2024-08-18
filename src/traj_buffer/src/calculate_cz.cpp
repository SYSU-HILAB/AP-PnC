#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <traj_buffer/aero_dynamics_guess.hpp>
// #include <Eigen::Dense>

namespace fit_data
{
    void init()
    {
        w    = 1.0;
        a0   = -11.8;
        a1   = -1.099;
        b1   = 22.35;
        a2   = 13.05;
        b2   = 1.314;
        a3   = 0.788;
        b3   = -5.099;
        a4   = -1.015;
        b4   = -0.2115;
        kPho = 1.225;
        kS   = 0.25;
        mass = 1.132;
    }
}  // namespace fit_data

double get_cz_from_vb(const Eigen::Vector3d &vb)
{
    double v = vb.norm();
    // Assumming 80-120 alpha is the coefficients is always the same.
    if (v < 0.5)
    {
        return 0.24;
    }
    // MPC state is in quadrotor frame
    // atan2 follows the rules of typical angle counterwise

    double alpha = std::atan2(vb(0), vb(2));
    if (alpha < 0.0)
    {
        // std::cerr << "Sing" << std::endl;

        return 0.0;
    }
    else
    {
        // normal
        // std::cerr << "Case3" << std::endl;

        using namespace fit_data;
        static bool init = false;
        if (!init)
        {
            init = true;
            fit_data::init();
        }
        // fourier series about alpha
        double cz =
            a0 + a1 * std::cos(w * alpha) + b1 * std::sin(w * alpha) +
            a2 * std::cos(2 * w * alpha) + b2 * std::sin(2 * w * alpha) +
            a3 * std::cos(3 * w * alpha) + b3 * std::sin(3 * w * alpha) +
            a4 * std::cos(4 * w * alpha) + b4 * std::sin(4 * w * alpha);

        // factor using rho and area;
        // std::cerr << "Cz" << 0.5 * fit_data::kPho * fit_data::kS * cz
        //           << std::endl;
        return 0.5 * fit_data::kPho * fit_data::kS * cz / mass;
    }
}
