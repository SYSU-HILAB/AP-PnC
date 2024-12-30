#pragma once
#include <Eigen/Dense>
#include <memory>     // For std::unique_ptr
#include <stdexcept>  // For std::runtime_error

namespace aerodynamics
{

    /**
     * @brief Interface for aerodynamic force and moment calculations
     *
     * This interface defines the contract for computing aerodynamic forces and
     * moments on a body given its velocity in the body frame.
     */
    class AerodynamicsInterface
    {
       public:
        /**
         * @brief Compute aerodynamic wrench (force and moment) in body frame
         *
         * @param[in] body_speed Velocity vector in body frame (m/s)
         * @param[out] aero_force_b Aerodynamic force vector in body frame (N)
         * @param[out] aero_moment_b Aerodynamic moment vector in body frame
         * (N⋅m)
         * @param[out] alpha Angle of attack (rad)
         * @param[out] beta Sideslip angle (rad)
         * @return true if computation successful, false otherwise
         *
         * @note All vectors are expressed in the body frame following the
         * Forward-Right-Down (FRD) convention
         */
        virtual bool getAeroWrench(const Eigen::Vector3d& body_speed,
                                   Eigen::Vector3d&       aero_force_b,
                                   Eigen::Vector3d&       aero_moment_b,
                                   double& alpha, double& beta) = 0;

        // Enable proper cleanup of derived classes
        virtual ~AerodynamicsInterface() = default;
    };

    /**
     * @brief Abstract factory for creating aerodynamics implementations
     *
     * This class follows the Factory pattern to create concrete implementations
     * of the AerodynamicsInterface. Each derived class should implement the
     * createAerodynamics method to return its specific implementation.
     */
    class AerodynamicsCreator
    {
       protected:
        std::unique_ptr<AerodynamicsInterface> _instance;

       public:
        /**
         * @brief Create a concrete implementation of AerodynamicsInterface
         *
         * @return true if creation successful, false otherwise
         */
        virtual bool createAerodynamics() = 0;

        /**
         * @brief Forward the getAeroWrench call to the concrete implementation
         *
         * @throws std::runtime_error if _instance is null
         */
        bool getAeroWrench(const Eigen::Vector3d& body_speed,
                           Eigen::Vector3d&       aero_force_b,
                           Eigen::Vector3d& aero_moment_b, double& alpha,
                           double& beta)
        {
            if (!_instance)
            {
                throw std::runtime_error("Aerodynamics instance not created");
            }
            return _instance->getAeroWrench(body_speed, aero_force_b,
                                            aero_moment_b, alpha, beta);
        }

        // Enable proper cleanup of derived classes
        virtual ~AerodynamicsCreator() = default;
    };

}  // namespace aerodynamics