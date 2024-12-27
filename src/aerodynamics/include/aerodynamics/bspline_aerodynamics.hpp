#pragma once

#include <aerodynamics/aero_interface.hpp>
namespace aerodynamics
{
    class BsplineAerodynamics : public AerodynamicsInterface
    {
       public:
        BsplineAerodynamics() = default;

        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta) override;
        ~BsplineAerodynamics() = default;
    };

    class BsplineAerodynamicsCreator : public AerodynamicsCreator
    {
       public:
        bool createAerodynamics() override;
        ~BsplineAerodynamicsCreator() = default;
    };
};  // namespace aerodynamics