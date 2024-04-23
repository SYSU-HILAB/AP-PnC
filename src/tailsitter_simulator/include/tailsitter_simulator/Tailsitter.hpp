#pragma once

#include <Eigen/Core>
#include <boost/array.hpp>
#include "Zhang/Aerodynamics.h"

// TODO ADD aerodynamic forces and moments

namespace TailsitterSimulator {
    struct Control {
        double thrust_vec[4];
        Control() {thrust_vec[0] = 0; thrust_vec[1] = 0; thrust_vec[2] = 0; thrust_vec[3] = 0;}
    };

    // alpha, beta in radians; V_b in body frame but robotics, flu definition, thrust is zb up.
    void sim_aerodynamics(const Eigen::Vector3d &V_b, Eigen::Vector3d &F_b, Eigen::Vector3d &M_b, double& alpha, double& beta);

    class Tailsitter {
    public:
        struct State {
            Eigen::Vector3d p = Eigen::Vector3d::Zero();
            Eigen::Vector3d v = Eigen::Vector3d::Zero();
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            Eigen::Vector3d omega = Eigen::Vector3d::Zero();
            Eigen::Array4d motor_rpm = Eigen::Array4d::Zero();

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        Tailsitter();

        const Tailsitter::State &getState(void) const;

        void setState(const Tailsitter::State &state);

        void setStatePos(const Eigen::Vector3d &Pos);

        double getMass(void) const;

        void setMass(double mass);

        double getGravity(void) const;

        void setGravity(double g);

        const Eigen::Matrix3d &getInertia(void) const;

        void setInertia(const Eigen::Matrix3d &inertia);

        double getArmLength(void) const;

        void setArmLength(double d);

        double getPropellerThrustCoefficient(void) const;

        void setPropellerThrustCoefficient(double kf);

        double getPropellerMomentCoefficient(void) const;

        void setPropellerMomentCoefficient(double km);

        double getMotorTimeConstant(void) const;

        void setMotorTimeConstant(double k);

        const Eigen::Vector3d &getExternalForce(void) const;

        void setExternalForce(const Eigen::Vector3d &force);

        const Eigen::Vector3d &getExternalMoment(void) const;

        void setExternalMoment(const Eigen::Vector3d &moment);

        double getMaxRPM(void) const;

        void setMaxRPM(double max_rpm);

        double getMinRPM(void) const;

        void setMinRPM(double min_rpm);

        // Inputs are desired RPM for the motors
        // Rotor numbering is:
        // # 2 yb 3
        // #   oo xb
        // # 1    0
        // with 0 and 2 clockwise, 1 and 3 counter-clockwise (looking from top)
        void setInput(double u0, double u1, double u2, double u3);

        void setControl(const TailsitterSimulator::Control &control);
        // Runs the actual dynamics simulation with a time step of dt
        void step(double dt);

        // For internal use, but needs to be public for odeint
        typedef boost::array<double, 22> InternalState;

        void operator()(const Tailsitter::InternalState &x,
                        Tailsitter::InternalState &dxdt, const double /* t */);

        Eigen::Vector3d getAcc() const;

    private:
        void updateInternalState(void);

        double alpha0; // AOA
        double g_;     // gravity
        double mass_;
        Eigen::Matrix3d J_; // Inertia
        double kf_;
        double km_;
        double prop_radius_;
        double arm_length_;
        double motor_time_constant_; // unit: sec
        double max_rpm_;
        double min_rpm_;

        Tailsitter::State state_;

        Eigen::Vector3d acc_;

        Eigen::Array4d input_;
        Eigen::Vector3d external_force_;
        Eigen::Vector3d external_moment_;

        InternalState internal_state_;
        bool aero_moment_on_;
        bool aero_force_on_;
    };
}
