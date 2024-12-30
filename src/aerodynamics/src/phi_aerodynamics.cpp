#include <aerodynamics/phi_aerodynamics.hpp>
#include <sq_config_reader/phi_aero_config_reader.hpp>

#ifndef NODEBUG
#include <iostream>
#define DEBUG_PRINT(x) std::cerr << x << std::endl
#else
#define DEBUG_PRINT(x)
#endif

namespace aerodynamics
{

    bool PhiAerodynamics::getAeroWrench(const Eigen::Vector3d& body_speed,
                                        Eigen::Vector3d&       aero_force_b,
                                        Eigen::Vector3d&       aero_moment_b,
                                        double& alpha, double& beta)
    {
        // Calculate alpha and beta from body velocity
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

        // Calculate aerodynamic forces using phi matrix
        aero_force_b  = _phi_matrix * body_speed * body_speed.norm();
        aero_moment_b = Eigen::Vector3d::Zero();
        return true;
    }

    bool PhiAerodynamicsCreator::createAerodynamics()
    {
        // Create config reader instance
        sq_config_reader::PhiAeroConfigReader db;

        // Initialize phi matrix with zeros
        Eigen::Matrix3d phi_matrix = Eigen::Matrix3d::Zero();

        if (!db.access_and_fetch_data())
        {
            return false;
        }

        // Get phi values from database
        auto phi_values = db.get_phi();

        phi_matrix << phi_values[0], phi_values[1], phi_values[2],
            phi_values[3], phi_values[4], phi_values[5], phi_values[6],
            phi_values[7], phi_values[8];

        DEBUG_PRINT("phi_matrix: " << phi_matrix);

        // Create and initialize PhiAerodynamics instance
        auto instance = std::make_unique<PhiAerodynamics>();
        instance->setPhi(phi_matrix);

        _instance = std::move(instance);
        return true;
    }

}  // namespace aerodynamics
