#pragma once
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{
    class PhiAerodynamcis : public AerodynamicsInterface
    {
       private:
        Eigen::Matrix3d __phi_matrix;

       public:
        PhiAerodynamcis() = default;

        bool setPhi(const Eigen::Matrix3d& phi_matrix)
        {
            __phi_matrix = phi_matrix;
            return true;
        }

        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta) override;
        ~PhiAerodynamcis() = default;
    };

    class PhiAerodynamicsCreator : public AerodynamicsCreator
    {
       public:
        bool createAerodynamics() override;
        ~PhiAerodynamicsCreator() = default;
    };
};  // namespace aerodynamics