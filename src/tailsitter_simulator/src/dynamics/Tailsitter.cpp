#include <toml++/toml.h>

#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <tailsitter_simulator/Tailsitter.hpp>
namespace odeint = boost::numeric::odeint;

namespace TailsitterSimulator
{

    // Eigen::Matrix3d R = q.toRotationMatrix();
    void sim_aerodynamics(const Eigen::Vector3d &V_b, Eigen::Vector3d &F_b,
                          Eigen::Vector3d &M_b, double &alpha, double &beta)
    {
        // Aerodynamics(V_b, F_b, M_b, alpha, beta);
        // const Eigen::Matrix3d Rot_frame = Eigen::Matrix3d::
        Eigen::Quaterniond q_frame_trans = Eigen::Quaterniond(0, 1, 0, 1);
        q_frame_trans.normalize();
        const Eigen::Matrix3d R_frame_trans = q_frame_trans.toRotationMatrix();

        auto   v_b_trans = R_frame_trans * V_b;
        double v_b[3]    = {v_b_trans(0), v_b_trans(1), v_b_trans(2)};
        double f_b[3]    = {0, 0, 0};
        double m_b[3]    = {0, 0, 0};
        Aerodynamics(v_b, f_b, m_b, &alpha, &beta);
        F_b = R_frame_trans * Eigen::Vector3d(f_b[0], f_b[1], f_b[2]);
        M_b = R_frame_trans * Eigen::Vector3d(m_b[0], m_b[1], m_b[2]);
    }

    Tailsitter::Tailsitter(void)
    {
        const char *env_p = std::getenv("TAILSITTER_PLANNING_DIR");
        std::string sv(env_p);
        sv += "/global.toml";
        toml::table tbl    = toml::parse_file(sv);
        auto        config = tbl["tailsitter_simulator"];
        g_                 = 9.81;
        mass_              = config["mass"].as_floating_point()->get();

        auto            iner = config["inertia"].as_array();
        Eigen::Vector3d diag_j;
        for (int i = 0; i < 3; i++)
        {
            diag_j(i) = iner->get(i)->as_floating_point()->get();
        }
        J_ = diag_j.asDiagonal();

        kf_ = config["kf"].as_floating_point()->get();

        km_ = config["tau"].as_floating_point()->get() * kf_;

        arm_length_          = config["arm_length"].as_floating_point()->get();
        motor_time_constant_ = config["motor_time"].as_floating_point()->get();
        min_rpm_             = config["min_rpm"].as_floating_point()->get();
        max_rpm_             = config["max_rpm"].as_floating_point()->get();

        aero_moment_on_ = config["aero_moment_on"].as_boolean()->get();
        aero_force_on_  = config["aero_force_on"].as_boolean()->get();
        external_force_.setZero();
        state_.motor_rpm =
            Eigen::Array4d::Ones() * sqrt(mass_ * g_ / (4 * kf_));
        updateInternalState();
        input_ = Eigen::Array4d::Ones() * sqrt(mass_ * g_ / (4 * kf_));
    }

    void Tailsitter::step(double dt)
    {
        auto save = internal_state_;

        odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

        for (int i = 0; i < 22; ++i)
        {
            if (std::isnan(internal_state_[i]))
            {
                std::cout << "dump " << i << " << pos ";
                for (int j = 0; j < 22; ++j)
                {
                    std::cout << save[j] << " ";
                }
                std::cout << std::endl;
                internal_state_ = save;
                break;
            }
        }

        for (int i = 0; i < 3; i++)
        {
            state_.p(i)     = internal_state_[0 + i];
            state_.v(i)     = internal_state_[3 + i];
            state_.R(i, 0)  = internal_state_[6 + i];
            state_.R(i, 1)  = internal_state_[9 + i];
            state_.R(i, 2)  = internal_state_[12 + i];
            state_.omega(i) = internal_state_[15 + i];
        }
        state_.motor_rpm(0) = internal_state_[18];
        state_.motor_rpm(1) = internal_state_[19];
        state_.motor_rpm(2) = internal_state_[20];
        state_.motor_rpm(3) = internal_state_[21];

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
        Eigen::Matrix3d             P = llt.matrixL();
        Eigen::Matrix3d             R = state_.R * P.inverse();
        state_.R                      = R;

        // Don't go below zero, simulate floor, and set all state to default
        // "zero" value, and return, don't step further
        if (state_.p(2) < -10.0 && state_.v(2) < 0)
        {
            state_.p     = Eigen::Vector3d(0, 0, -10);
            state_.v     = Eigen::Vector3d(0, 0, 0);
            state_.R     = Eigen::Matrix3d::Identity();
            state_.omega = Eigen::Vector3d(0, 0, 0);
            // std::cerr << "Crash detected, resetting state to zero" <<
            // std::endl;
        }
        updateInternalState();

        // and then, exit the whole program
    }

    void Tailsitter::operator()(const Tailsitter::InternalState &x,
                                Tailsitter::InternalState       &dxdt,
                                const double /* t */)
    {
        State cur_state;
        for (int i = 0; i < 3; i++)
        {
            cur_state.p(i)     = x[0 + i];
            cur_state.v(i)     = x[3 + i];
            cur_state.R(i, 0)  = x[6 + i];
            cur_state.R(i, 1)  = x[9 + i];
            cur_state.R(i, 2)  = x[12 + i];
            cur_state.omega(i) = x[15 + i];
        }
        for (int i = 0; i < 4; i++)
        {
            cur_state.motor_rpm(i) = x[18 + i];
        }

        // std::cout << "Omega: " << cur_state.omega << std::endl;
        // std::cout << "motor_rpm: " << cur_state.motor_rpm << std::endl;

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
        Eigen::Matrix3d             P = llt.matrixL();
        Eigen::Matrix3d             R = cur_state.R * P.inverse();

        Eigen::Vector3d p_dot, v_dot, omega_dot;
        Eigen::Matrix3d R_dot;
        Eigen::Array4d  motor_rpm_dot;
        Eigen::Vector3d vnorm;
        Eigen::Array4d  motor_rpm_sq;
        Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

        omega_vee(2, 1) = cur_state.omega(0);
        omega_vee(1, 2) = -cur_state.omega(0);
        omega_vee(0, 2) = cur_state.omega(1);
        omega_vee(2, 0) = -cur_state.omega(1);
        omega_vee(1, 0) = cur_state.omega(2);
        omega_vee(0, 1) = -cur_state.omega(2);

        motor_rpm_sq = cur_state.motor_rpm.array().square();

        // TODO more flexible parameter to config the rotors configuration
        // x configuration of the rotors, Assuming the inter-motors vertex
        // consistutes a square.
        double factor = arm_length_ / sqrt(2) * kf_;
        // double totalF = kf_ * motor_rpm_sq.sum();
        double thrust = kf_ * motor_rpm_sq.sum();
        // Propeller dynamics
        Eigen::Vector3d moments_prop;
        Eigen::Vector3d moments_aero;
        Eigen::Vector3d moments;
        moments_prop(0) = factor * (-motor_rpm_sq(0) - motor_rpm_sq(1) +
                                    motor_rpm_sq(2) + motor_rpm_sq(3));
        moments_prop(1) = factor * (-motor_rpm_sq(0) + motor_rpm_sq(1) +
                                    motor_rpm_sq(2) - motor_rpm_sq(3));
        moments_prop(2) = km_ * (-motor_rpm_sq(0) + motor_rpm_sq(1) -
                                 motor_rpm_sq(2) + motor_rpm_sq(3));
        // Aerodynamics
        Eigen::Vector3d F_aero;
        Eigen::Vector3d vb = cur_state.R.transpose() * cur_state.v;
        // not using alpha and beta
        double alpha, beta;
        sim_aerodynamics(vb, F_aero, moments_aero, alpha, beta);
        // TODO change it to aerodynamics of the wing
        // F_aero = Eigen::Vector3d(0, 0, 0);
        if (!aero_moment_on_)
            moments_aero = Eigen::Vector3d(0, 0, 0);
        if (!aero_force_on_)
            F_aero = Eigen::Vector3d(0, 0, 0);
        moments = moments_prop + moments_aero;
        // F_aero(1) = 0.0; // no side force
        // if (!)
        vnorm = cur_state.v;

        p_dot = cur_state.v;
        v_dot = -Eigen::Vector3d(0, 0, g_) + thrust * R.col(2) / mass_ +
                external_force_ / mass_ + R * (F_aero / mass_);
        // std::cerr << "v_dot: " << v_dot.transpose() << std::endl;
        // std::cerr << "thrust_acc: " << (thrust * R.col(2) /
        // mass_).transpose() << std::endl; std::cerr << "aero_acc: " << (F_aero
        // / mass_).transpose() << std::endl;
        acc_ = v_dot;
        //  acc_[2] = -acc_[2]; // to NED

        R_dot     = R * omega_vee;
        omega_dot = J_.inverse() *
                    (moments - cur_state.omega.cross(J_ * cur_state.omega) +
                     external_moment_);
        motor_rpm_dot = (input_ - cur_state.motor_rpm) / motor_time_constant_;

        for (int i = 0; i < 3; i++)
        {
            dxdt[0 + i]  = p_dot(i);
            dxdt[3 + i]  = v_dot(i);
            dxdt[6 + i]  = R_dot(i, 0);
            dxdt[9 + i]  = R_dot(i, 1);
            dxdt[12 + i] = R_dot(i, 2);
            dxdt[15 + i] = omega_dot(i);
        }
        for (int i = 0; i < 4; i++)
        {
            dxdt[18 + i] = motor_rpm_dot(i);
        }
        for (int i = 0; i < 22; ++i)
        {
            if (std::isnan(dxdt[i]))
            {
                dxdt[i] = 0;
                //      std::cout << "nan apply to 0 for " << i << std::endl;
            }
        }
    }

    void Tailsitter::setInput(double u1, double u2, double u3, double u4)
    {
        input_(0) = u1;
        input_(1) = u2;
        input_(2) = u3;
        input_(3) = u4;

        for (int i = 0; i < 4; i++)
        {
            if (std::isnan(input_(i)))
            {
                input_(i) = (max_rpm_ + min_rpm_) / 2;
                std::cout << "NAN input ";
            }
            if (input_(i) > max_rpm_)
            {
                input_(i) = max_rpm_;
                std::cerr << "Input " << i
                          << " is larger than max_rpm, setting to max_rpm"
                          << std::endl;
            }
            else if (input_(i) < min_rpm_)
                input_(i) = min_rpm_;
        }
    }
    void Tailsitter::setControl(const TailsitterSimulator::Control &control)
    {
        assert(control.thrust_vec[0] >= 0);
        assert(control.thrust_vec[1] >= 0);
        assert(control.thrust_vec[2] >= 0);
        assert(control.thrust_vec[3] >= 0);

        setInput(std::sqrt(control.thrust_vec[0] / kf_),
                 std::sqrt(control.thrust_vec[1] / kf_),
                 std::sqrt(control.thrust_vec[2] / kf_),
                 std::sqrt(control.thrust_vec[3] / kf_));
    }

    const Tailsitter::State &Tailsitter::getState(void) const
    {
        return state_;
    }

    void Tailsitter::setState(const Tailsitter::State &state)
    {
        state_.p         = state.p;
        state_.v         = state.v;
        state_.R         = state.R;
        state_.omega     = state.omega;
        state_.motor_rpm = state.motor_rpm;

        updateInternalState();
    }

    void Tailsitter::setStatePos(const Eigen::Vector3d &Pos)
    {
        state_.p = Pos;

        updateInternalState();
    }

    double Tailsitter::getMass(void) const
    {
        return mass_;
    }

    void Tailsitter::setMass(double mass)
    {
        mass_ = mass;
    }

    double Tailsitter::getGravity(void) const
    {
        return g_;
    }

    void Tailsitter::setGravity(double g)
    {
        g_ = g;
    }

    const Eigen::Matrix3d &Tailsitter::getInertia(void) const
    {
        return J_;
    }

    void Tailsitter::setInertia(const Eigen::Matrix3d &inertia)
    {
        if (inertia != inertia.transpose())
        {
            std::cerr << "Inertia matrix not symmetric, not setting"
                      << std::endl;
            return;
        }
        J_ = inertia;
    }

    double Tailsitter::getArmLength(void) const
    {
        return arm_length_;
    }

    void Tailsitter::setArmLength(double d)
    {
        if (d <= 0)
        {
            std::cerr << "Arm length <= 0, not setting" << std::endl;
            return;
        }

        arm_length_ = d;
    }

    double Tailsitter::getPropellerThrustCoefficient(void) const
    {
        return kf_;
    }

    void Tailsitter::setPropellerThrustCoefficient(double kf)
    {
        if (kf <= 0)
        {
            std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
            return;
        }

        kf_ = kf;
    }

    double Tailsitter::getPropellerMomentCoefficient(void) const
    {
        return km_;
    }

    void Tailsitter::setPropellerMomentCoefficient(double km)
    {
        if (km <= 0)
        {
            std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
            return;
        }

        km_ = km;
    }

    double Tailsitter::getMotorTimeConstant(void) const
    {
        return motor_time_constant_;
    }

    void Tailsitter::setMotorTimeConstant(double k)
    {
        if (k <= 0)
        {
            std::cerr << "Motor time constant <= 0, not setting" << std::endl;
            return;
        }

        motor_time_constant_ = k;
    }

    const Eigen::Vector3d &Tailsitter::getExternalForce(void) const
    {
        return external_force_;
    }

    void Tailsitter::setExternalForce(const Eigen::Vector3d &force)
    {
        external_force_ = force;
    }

    const Eigen::Vector3d &Tailsitter::getExternalMoment(void) const
    {
        return external_moment_;
    }

    void Tailsitter::setExternalMoment(const Eigen::Vector3d &moment)
    {
        external_moment_ = moment;
    }

    double Tailsitter::getMaxRPM(void) const
    {
        return max_rpm_;
    }

    void Tailsitter::setMaxRPM(double max_rpm)
    {
        if (max_rpm <= 0)
        {
            std::cerr << "Max rpm <= 0, not setting" << std::endl;
            return;
        }
        max_rpm_ = max_rpm;
    }

    double Tailsitter::getMinRPM(void) const
    {
        return min_rpm_;
    }

    void Tailsitter::setMinRPM(double min_rpm)
    {
        if (min_rpm < 0)
        {
            std::cerr << "Min rpm < 0, not setting" << std::endl;
            return;
        }
        min_rpm_ = min_rpm;
    }

    void Tailsitter::updateInternalState(void)
    {
        for (int i = 0; i < 3; i++)
        {
            internal_state_[0 + i]  = state_.p(i);
            internal_state_[3 + i]  = state_.v(i);
            internal_state_[6 + i]  = state_.R(i, 0);
            internal_state_[9 + i]  = state_.R(i, 1);
            internal_state_[12 + i] = state_.R(i, 2);
            internal_state_[15 + i] = state_.omega(i);
        }
        internal_state_[18] = state_.motor_rpm(0);
        internal_state_[19] = state_.motor_rpm(1);
        internal_state_[20] = state_.motor_rpm(2);
        internal_state_[21] = state_.motor_rpm(3);
    }

    Eigen::Vector3d Tailsitter::getAcc() const
    {
        return acc_;
    }
}  // namespace TailsitterSimulator
