#pragma once
#include <bspline/Core.h>

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace aerodyn
{

    class BsplineAeroDyn
    {
       public:
        static constexpr size_t SPLINE_DEGREE = 3;
        using Spline = bspline::Spline<double, SPLINE_DEGREE>;

       private:
        bool intialized = false;

        std::vector<Spline> exp_splines_;
        std::vector<double> exp_coefs_;

        std::vector<Spline> sim_splines_;
        std::vector<double> sim_coefs_;

       public:
        enum class AeroDynType
        {
            Exp,
            Sim
        };

        /*
         * @brief Constructor
         * only construct splines from static knots
         */
        BsplineAeroDyn();

        /*
         * @brief Set the coefficients of the aerodynamic model
         * @param coefs Coefficients
         * @param type Exp or Sim
         * @return True if successful
         */
        bool setCoefs(std::vector<double> coefs, AeroDynType type);

        /*
         * @brief Get the aerodynamic coefficient Cz
         * @param alpha Angle of attack in radians
         * @param type Exp or Sim
         * @return Cz
         */
        double getAerodynCz(double alpha, AeroDynType type);
    };

    static BsplineAeroDyn aero_dyn;

    /*
     * @brief Get the aerodynamic coefficient Cz
     * @param vb Body velocity
     * @return Cz
     */
    double getAeroCz(const Eigen::Vector3d      &vb,
                     BsplineAeroDyn::AeroDynType type);
}  // namespace aerodyn
