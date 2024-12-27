#include <bspline/Core.h>

#include <aerodynamics/bspline_aerodynamics.hpp>

#include "Zhang/Aerodynamics.h"

namespace aerodynamics
{

    bool BsplineAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                            Eigen::Vector3d&       aero_force_b,
                                            Eigen::Vector3d& aero_moment_b,
                                            double& alpha, double& beta)
    {
        double v_b[3] = {body_speed(0), body_speed(1), body_speed(2)};
        double f_aero_b[3];
        double moment_aero_b[3];

        Aerodynamics(v_b, f_aero_b, moment_aero_b, &alpha, &beta);

        aero_force_b  = Eigen::Vector3d(f_aero_b[0], f_aero_b[1], f_aero_b[2]);
        aero_moment_b = Eigen::Vector3d(moment_aero_b[0], moment_aero_b[1],
                                        moment_aero_b[2]);
        return true;
    }

    bool BsplineAerodynamicsCreator::createAerodynamics()
    {
        std::unique_ptr<BsplineAerodynamics> instance =
            std::make_unique<BsplineAerodynamics>();
        __instance = std::move(instance);
        // in this case because C++'s Return Value Optimization (RVO) will take
        // care of moving the returned object.
        return true;
    }

};  // namespace aerodynamics

// phi_theory::phi_theory()
// {
//     // Initialize the phi_matrix if necessary (could be identity or zero
//     // matrix, etc.)
//     __phi_matrix = Eigen::Matrix3d::Identity();
// }

// bool phi_theory::setPhi(const Eigen::Matrix3d& phi_matrix)
// {
//     __phi_matrix = phi_matrix;
//     return true;
// }

// phi_theory& phi_theory::getInstance()
// {
//     static phi_theory instance;
//     return instance;
// }

// std::tuple<Eigen::Vector3d, Eigen::Vector3d> phi_theory::getAeroWrench(
//     const Eigen::Vector3d& body_speed)
// {
//     Eigen::Vector3d aero_force  = __phi_matrix * body_speed;
//     Eigen::Vector3d aero_torque = Eigen::Vector3d::Zero();
//     return std::make_tuple(aero_force, aero_torque);
// }

// bool phi_theory::getAeroForcePlainAPI(const double body_speed[3],
//                                       double       aero_force[3])
// {
//     Eigen::Vector3d __body_speed(body_speed[0], body_speed[1],
//                                  body_speed[2]);
//     Eigen::Vector3d aero_force_eigen = __phi_matrix * __body_speed;

//     aero_force[0] = aero_force_eigen(0);
//     aero_force[1] = aero_force_eigen(1);
//     aero_force[2] = aero_force_eigen(2);

//     return true;
// }

// Eigen::Matrix3d phi_theory::getPhi()
// {
//     return __phi_matrix;
// }

// phi_theory& exportPhiInstance = phi_theory::getInstance();