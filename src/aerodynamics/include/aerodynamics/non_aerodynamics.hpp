#pragma once
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{
    class NonAerodynamics : public AerodynamicsInterface
    {
       public:
        NonAerodynamics()  = default;
        ~NonAerodynamics() = default;

        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta) override;
    };

    class NonAerodynamicsCreator : public AerodynamicsCreator
    {
       public:
        bool createAerodynamics() override;
        ~NonAerodynamicsCreator() = default;
    };

}  // namespace aerodynamics