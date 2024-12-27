#pragma once
#include <aerodynamics/aero_interface.hpp>

namespace aerodynamics
{
    class LyuAerodynamics : public AerodynamicsInterface
    {
       public:
        LyuAerodynamics() = default;

        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta) override;
        ~LyuAerodynamics() = default;
    };

    class LyuAerodynamicsCreator : public AerodynamicsCreator
    {
       public:
        bool createAerodynamics() override;
        ~LyuAerodynamicsCreator() = default;
    };
};  // namespace aerodynamics