#include <iostream>
#include <rcpputils/asserts.hpp>
#include <traj_buffer/aerodyn.hpp>
namespace aerodyn
{
    BsplineAeroDyn::BsplineAeroDyn()
    {
        const std::vector<double> exp_knots{
            0.5235987755982989, 0.5235987755982989, 0.5235987755982989,
            0.5235987755982989, 0.7853981633974483, 0.8726646259971648,
            0.9599310885968813, 1.0471975511965976, 1.1344640137963142,
            1.3089969389957472, 1.8325957145940461, 2.2689280275926285,
            2.617993877991494,  2.617993877991494,  2.617993877991494,
            2.617993877991494};
        exp_splines_ = bspline::generateBSplines<SPLINE_DEGREE>(exp_knots);
        exp_coefs_.resize(exp_splines_.size(), 0.0);

        const std::vector<double> sim_knots{0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.4363323129985824,
                                            0.6108652381980153,
                                            0.7853981633974483,
                                            0.9599310885968813,
                                            1.3089969389957472,
                                            2.356194490192345,
                                            2.6179938779914944,
                                            3.141592653589793,
                                            3.141592653589793,
                                            3.141592653589793,
                                            3.141592653589793};
        sim_splines_ = bspline::generateBSplines<SPLINE_DEGREE>(sim_knots);
        sim_coefs_.resize(sim_splines_.size(), 0.0);
    }

    bool BsplineAeroDyn::setCoefs(std::vector<double> coefs, AeroDynType type)
    {
        if (type == AeroDynType::Exp)
        {
            assert(coefs.size() == exp_coefs_.size());
            if (coefs.size() != exp_coefs_.size())
            {
                return false;
            }
            for (size_t i = 0; i < coefs.size(); i++)
            {
                exp_coefs_[i] = coefs[i];
            }
        }
        else
        {
            assert(coefs.size() == sim_coefs_.size());
            if (coefs.size() != sim_coefs_.size())
            {
                return false;
            }
            for (size_t i = 0; i < coefs.size(); i++)
            {
                sim_coefs_[i] = coefs[i];
            }
        }
        intialized = true;
        return true;
    }

    double BsplineAeroDyn::getAerodynCz(double alpha, AeroDynType type)
    {
        double val = 0;
        if (type == AeroDynType::Exp)
        {
            for (size_t i = 0; i < exp_splines_.size(); i++)
            {
                val += exp_splines_[i](alpha) * exp_coefs_[i];
            }
        }
        else
        {
            for (size_t i = 0; i < sim_splines_.size(); i++)
            {
                val += sim_splines_[i](alpha) * sim_coefs_[i];
            }
        }
        return val;
    }

    double BsplineAeroDyn::getAeroCzVb(const Eigen::Vector3d      &vb,
                                       BsplineAeroDyn::AeroDynType type)
    {
        if (vb.norm() < 0.5)
        {
            return getAerodynCz(M_PI_2, type);
        }
        // quadrotor body frame x forward, z downward
        double alpha = std::atan2(vb(0), vb(2));
        // if alpha in range[pi/6 ~ 5pi/6] use the model, otherwise return 0.0
        // and raise error
        constexpr double pi_6   = M_PI / 6;
        constexpr double pi_5_6 = 5 * M_PI / 6;
        if (alpha < pi_6 || alpha > pi_5_6)
        {
            return 0.1;
        }
        return getAerodynCz(alpha, type);
    }
}  // namespace aerodyn