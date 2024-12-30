// #include <bspline/Core.h>

#include <aerodynamics/bspline_aerodynamics.hpp>
#include <sq_config_reader/bspline_aero_config_reader.hpp>

#ifndef NODEBUG
#include <iostream>
#define DEBUG_PRINT(x) std::cerr << x << std::endl
#else
#define DEBUG_PRINT(x)
#endif

namespace aerodynamics
{
    BsplineAerodynamics::BsplineAerodynamics(
        const std::vector<double>& cx_coefs,
        const std::vector<double>& cx_knots,
        const std::vector<double>& cz_coefs,
        const std::vector<double>& cz_knots, const double scale_factor)
        : cx_knots_(cx_knots),
          cz_knots_(cz_knots),
          cx_coefficients_(cx_coefs),
          cz_coefficients_(cz_coefs),
          scale_factor_(scale_factor)
    {
    }

    bool BsplineAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                            Eigen::Vector3d&       aero_force_b,
                                            Eigen::Vector3d& aero_moment_b,
                                            double& alpha, double& beta)
    {
        // Generate bspline basis functions
        static std::vector<Spline> cx_splines =
            bspline::generateBSplines<SPLINE_ORDER>(cx_knots_);
        static std::vector<Spline> cz_splines =
            bspline::generateBSplines<SPLINE_ORDER>(cz_knots_);
        // Calculate alpha and beta first, similar to PhiAerodynamics
        // alpha, beta  [rad]
        constexpr double vThreshold = 5e-1;
        if (body_speed.norm() < vThreshold)
        {
            alpha = 0.0;
            beta  = 0.0;
        }
        else
        {
            alpha = std::atan2(body_speed(2), body_speed(0));
            beta  = std::asin(body_speed(1) / body_speed.norm());
        }

        double v_squared = body_speed.squaredNorm();

        double           aeroForceBx = 0.0;
        constexpr double aeroForceBy = 0.0;
        double           aeroForceBz = 0.0;

        for (size_t i = 0; i < cx_splines.size(); ++i)
        {
            aeroForceBx += cx_coefficients_[i] * cx_splines[i](alpha);
        }
        for (size_t i = 0; i < cz_splines.size(); ++i)
        {
            aeroForceBz += cz_coefficients_[i] * cz_splines[i](beta);
        }

        aero_force_b = scale_factor_ *
                       Eigen::Vector3d(aeroForceBx, aeroForceBy, aeroForceBz) *
                       v_squared * cos(beta);
        aero_moment_b = Eigen::Vector3d(0.0, 0.0, 0.0);

        return true;
    }

    bool BsplineAerodynamicsCreator::createAerodynamics()
    {
        // Create config reader instance
        sq_config_reader::BSplineAeroConfigReader db;

        if (!db.access_and_fetch_data())
        {
            return false;
        }

        // Get bspline parameters from database
        auto cx_coefs = db.get_cx_coefs();
        auto cx_knots = db.get_cx_knots();
        auto cz_coefs = db.get_cz_coefs();
        auto cz_knots = db.get_cz_knots();

        auto scale_factor = db.get_scale_factor();

        DEBUG_PRINT("Loaded bspline parameters");

        // Create and initialize BsplineAerodynamics instance
        auto instance = std::make_unique<BsplineAerodynamics>(
            cx_coefs, cx_knots, cz_coefs, cz_knots, scale_factor);

        _instance = std::move(instance);
        return true;
    }

};  // namespace aerodynamics
