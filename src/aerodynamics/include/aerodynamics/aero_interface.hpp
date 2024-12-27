#pragma once
#include <Eigen/Dense>
#include <memory>  // For std::unique_ptr
namespace aerodynamics
{
    class AerodynamicsInterface
    {
       public:
        virtual bool getAeroWrench(const Eigen::Vector3d& body_speed,
                                   Eigen::Vector3d&       aero_force_b,
                                   Eigen::Vector3d&       aero_moment_b,
                                   double& alpha, double& beta) = 0;
        virtual ~AerodynamicsInterface()                        = default;
    };

    class AerodynamicsCreator
    {
       protected:
        std::unique_ptr<AerodynamicsInterface> __instance;

       public:
        virtual bool createAerodynamics() = 0;
        virtual ~AerodynamicsCreator()    = default;

        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta)
        {
            return __instance->getAeroWrench(body_speed, aero_force_b,
                                             aero_moment_b, alpha, beta);
        }
    };

}  // namespace aerodynamics