// C++ includes
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <cassert>
#include <tailsitter_df/df.hpp>
#include <toml++/toml.hpp>
// #include <
// #include
// use abort to signal coredump
#include <cstdlib>

constexpr auto &out = std::cout;

//
template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}
// to be extern

using namespace autodiff;
namespace tailsitter_df
{
    Eigen::Matrix<double, 4, 9> J;

    tailsitter_df::Model &tailsitter_model = Model::getInstance();
    Data                 &data             = Data::getInstance();

    using vec = autodiff::VectorXreal;

    Model &Model::getInstance()
    {
        static Model instance;  // 静态局部变量
        return instance;
    }

    Model::Model()
    {
        // using TModel::Type;
        // tailsitter_df::yb_prev.normalize();
        // tailsitter_df::zb_fixed.normalize();
        type_ = Model::Type::HighLiftRange;
        // Update staitc parameter;
        const char *env_p = std::getenv("TAILSITTER_PLANNING_DIR");
        std::string sv(env_p);
        sv += "/global.toml";
        toml::table tbl    = toml::parse_file(sv);
        auto        config = tbl["tailsitter_df"];

        auto phi = config["phi"].as_array();
        for (int i = 0; i < phi->size(); i++)
        {
            // Real val(phi->get(i)->as_array()->size());
            // Real val(phi->get(i)->as_floating_point()->get());
            double val  = phi->get(i)->as_floating_point()->get();
            getPhi()(i) = val;
        }
        auto it = config["bias"].as_array();
        for (int i = 0; i < size_Models; i++)
        {
            bias_[i] << it->get(i)->as_floating_point()->get();
        }

        mass_      = config["mass"].as_floating_point()->get();
        area_      = config["area"].as_floating_point()->get();
        rho_       = config["air_density"].as_floating_point()->get();
        alpha_rad_ = 0;

        scale_ = 1.0;
    }

    realM3 &Model::getPhi()
    {
        if (type_ == Model::Type::Plain)
        {
            return Phis_[0];
        }
        if (type_ == Model::Type::HighLiftRange)
        {
            return Phis_[1];
        }
    };

    Eigen::Vector3d &Model::Phi_z()
    {
        static Eigen::Vector3d temp;
        temp << getPhi().row(2)(0).val(), getPhi().row(2)(1).val(),
            getPhi().row(2)(2).val();
        return temp;
    };

    // now Assuming there is no bias
    double Model::aeroForceZ(const Eigen::Vector3d &vb)
    {
        auto  &phi_z = Phi_z();
        double temp  = (vb.norm() * phi_z.transpose() * vb);
        return scale_ * temp;
    };

    double Model::dAeroForceZ(const Eigen::Vector3d &vb,
                              const Eigen::Vector3d &acc)
    {
        double vb_norm = vb.norm();
        if (vb_norm < 0.5)
        {
            return 0;
        }
        auto              &phi_z = Phi_z();
        Eigen::RowVector3d temp =
            (scale_) * (((phi_z.transpose() * vb)) * vb.transpose()) *
                (1 / vb_norm) +
            (vb_norm * phi_z.transpose());
        return temp * acc;
    };

    realV3 &Model::getBias()
    {
        if (type_ == Model::Type::Plain)
        {
            return bias_[0];
        }
        if (type_ == Model::Type::HighLiftRange)
        {
            return bias_[1];
        }
    };

    realV3 Model::getAeroForceB(const realV3 &vb)
    {
        auto V = vb.norm();

        return scale_ * (V * getPhi() * vb + V * V * getBias());
    };

    realV3 Model::getAeroForceAccB(const realV3 &vb)
    {
        return getAeroForceB(vb) / mass_;
    };

    realM3 Model::getPartialDerivativeOfFbOverVb(const realV3 &vb)
    {
        realM3 temp =
            getPhi() * vb * vb.transpose() / vb.norm() + getPhi() * vb.norm();
        temp += 2 * getBias() * vb.transpose();
        return scale_ * temp;
    };

    real Model::computeAlpha(real h, real gamma)
    {
        auto &bias = getBias();
        auto  b    = (h * sin(gamma) + getPhi().row(2)(0));
        auto  a    = (getPhi().row(2)(2) - h * cos(gamma));
        auto  c    = getBias()(2);
        if (bias.norm().val() <= 1e-5)
        {
            // in double atan function, it returns (-90, 90);
            // TODO check not to touch 90;

            return atan2(b, -a);
        }
        else
        {
            Vector2real sol;

            triangleFunctionEquationSolver(a, b, c, sol);

            if (sol(0) == error_alpha)
            {
                std::cout << "Wrong Alpha" << std::endl;
            }
            else
            {
                alpha_rad_ =
                    abs(sol(0) - 0) < abs(sol(1) - 0) ? sol(0) : sol(1);
                return alpha_rad_;
            }
        }
        // temp now for conveniency , only use -50 to 50;
        // return 0;
    };
    // void init(const Eigen::Matrix3d Phi[], realV3& bias){

    void Model::triangleFunctionEquationSolver(const real a, const real b,
                                               const real c, Vector2real &sol)
    {
        if (a * a + b * b - c * c >= 0)
        {
            data.tri_success = true;
            auto temp        = b * sqrt(a * a + b * b - c * c);

            auto sin1 = (a * c + temp) / (a * a + b * b);
            auto cos1 = (c - a * sin1) / b;

            // atan2 maps a coordinate to [-pi,pi]
            sol(0) = atan2(sin1, cos1);

            auto sin2 = (a * c - temp) / (a * a + b * b);
            auto cos2 = (c - a * sin2) / b;
            sol(1)    = atan2(sin2, cos2);
        }
        else if (a * a + b * b - c * c >= -0.1)
        {
            data.tri_success = false;
            std::cout << "warning alpha"
                      << "\n";
            Model::triangleFunctionEquationSolver(1.2 * a, 1.2 * b, c, sol);
        }
        else
        {
            data.tri_success = false;
            sol << error_alpha, error_alpha;
        }
        // std::asin()
    };
    // };
    // autodiff::Real;

    void init(VectorXreal &z)
    {
        z << 4, 6, 5, 1, 2, 3, 7, 8, 9;
        // 1, 2, 3;
    }

    VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                     const Eigen::Vector3d &jerk)
    {
        VectorXreal z;
        z.resize(9);
        z << vel(0), vel(1), vel(2), acc(0), acc(1), acc(2), jerk(0), jerk(1),
            jerk(2);
        return z;
    }

    void toEigen(const VectorXreal &F, double &acc_thrust,
                 Eigen::Vector3d &omega)
    {
        acc_thrust = F(0).val();
        omega << F(1).val(), F(2).val(), F(3).val();
    }

    using namespace std;

    realM3 skew(const realV3 &v)
    {
        realM3 rtn;
        rtn << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

        return rtn;
    }

    Eigen::Vector4<real> computeDotAtAndBodyRates(
        const Eigen::Matrix<real, 1, 4> &N1,
        const Eigen::Matrix<real, 3, 4> &N2, const real &h1, const realV3 &h2)
    {
        Eigen::Vector4<real> h;
        h << h1, h2;
        Eigen::Matrix4<real> N;
        N << N1, N2;

        bool is_singular = abs(N.determinant()) <= 1e-4;

        if (is_singular)
        {
            std::cerr << "N:\n" << N << "\n";
            std::cerr << "h:\n" << h.transpose() << "\n";

            abort();
            return Eigen::Vector4<real>::Zero();
        }
        else
        {
            return N.inverse() * h;
        }
    }

    VectorXreal df_forward_nominal(const Vector3real &v, const Vector3real &a,
                                   const Vector3real &j)
    {
        realV3 &eta      = data.eta;
        realV3 &zb_fixed = data.zb_fixed;
        // realV3 temp = v.cross(eta);
        realV3 &xb = data.xb;
        realV3 &yb = data.yb;
        realV3 &zb = data.zb;

        realV3 last_xb = data.xb;
        realV3 last_yb = data.yb;
        realV3 last_zb = data.zb;

        auto   &attitude   = data.attitude;
        auto   &q          = data.q;
        auto   &alpha      = data.alpha;
        real    alpha_prev = alpha;
        auto   &at         = data.at;
        auto   &magic      = data.magic;
        auto   &yb_prev    = data.yb_prev;
        realV3 &aeroForceB = data.aeroForceB;

        realV3 x_v  = v / v.norm();
        realV3 temp = -(realM3::Identity() - x_v * x_v.transpose()) * (a - g);
        real   a_vz = -temp.norm();
        realV3 z_v  = temp / temp.norm();
        realV3 y_v  = z_v.cross(x_v);
        real   a_vx = x_v.transpose() * (a - g);
        real   V    = v.norm();
        real   omega_vz = (y_v.transpose() * g);
        omega_vz        = omega_vz / V;
        real omega_vx   = (y_v.transpose() * j);
        omega_vx        = ((omega_vz * a_vx) - omega_vx) / a_vz;
        real omega_vy   = (z_v.transpose() * g);
        omega_vy        = (-a_vz - omega_vy) / V;

        realV3 aero_acc = tailsitter_model.getPhi().row(2).transpose() * V * V;
        aeroForceB      = aero_acc;
        alpha           = atan2(aero_acc(0) - a_vz, a_vx - aero_acc(2));

        yb    = y_v;
        xb    = cos(alpha) * x_v - sin(alpha) * z_v;
        zb    = cos(alpha) * z_v + sin(alpha) * x_v;
        at(0) = a_vx * cos(alpha) - a_vz * sin(alpha);
        attitude << xb, yb, zb;

        real omega_bx = omega_vx * cos(alpha) - omega_vz * sin(alpha);
        real omega_bz = omega_vx * sin(alpha) + omega_vz * cos(alpha);

        real phi31 = tailsitter_model.getPhi().row(2).transpose()(0);
        real phi33 = tailsitter_model.getPhi().row(2).transpose()(2);
        // auto &q = data.q;
        q             = Eigen::Quaternion<real>(attitude);
        yb            = -yb;
        yb_prev       = yb;
        real av_z_dot = z_v.transpose() * j;
        av_z_dot      = av_z_dot + omega_vy * a_vx;

        real av_x_dot = x_v.transpose() * j;
        av_x_dot      = av_x_dot - omega_vy * a_vz;

        real v2_acc = v.transpose() * a;
        // method1  singularity
        // real alpha_dot = (av_x_dot - 2 * phi33 * v2_acc) * sin(alpha) +
        //                  (av_z_dot - 2 * phi31 * v2_acc) * cos(alpha);
        // alpha_dot     = alpha_dot / ((aero_acc(0) - a_vx) * cos(alpha) +
        //                          (a_vz - aero_acc(2)) * sin(alpha));

        // real demoninator = (a_vz + aero_acc(2)) * sin(alpha) -
        //                    (a_vx + aero_acc(0)) * cos(alpha);
        // real numerator = (av_x_dot - 2 * phi33 * v2_acc) * sin(alpha) +
        //                  (av_z_dot - 2 * phi31 * v2_acc) * cos(alpha);
        // real   alpha_dot = numerator / demoninator;
        // realM3 last_R    = (realM3() << last_xb, last_yb,
        // last_zb).finished(); realV3 vb        = attitude.transpose() * v;
        // realV3 vb_dot    = attitude.transpose() * a;

        // method approximation: too small
        realV3 vb    = attitude.transpose() * v;
        realV3 term3 = vb(0) * zb - vb(2) * xb;

        real term1 = zb.transpose() * a;
        term1      = term1 * vb(0);
        real term2 = xb.transpose() * a;
        term2      = term2 * vb(2);
        real V2    = v.transpose() * v;
        // real alpha_dot = (term1 - term2);
        // alpha_dot      = alpha_dot / V2;
        real alpha_dot = term3.transpose() * a;
        alpha_dot      = alpha_dot / V2;
        // real alpha_dot = (vb_dot(2) * vb(0) - vb_dot(0) * vb(2)) / (V * V);
        real aero_b_z = aero_acc(0) * cos(alpha) + aero_acc(2) * sin(alpha);

        real av_bz_est = a_vz;

        real aero_w_z = aero_b_z * zb(2);
        real aero_w_x = aero_b_z * zb(2);
        real aero_w_y = aero_b_z * zb(2);

        real aero_w_xy = sqrt(aero_w_x * aero_w_x + aero_w_y * aero_w_y);
        real aerosz    = aero_acc(0) * cos(alpha) + aero_acc(2) * sin(alpha);
        aeroForceB(2)  = alpha_dot;
        alpha_dot      = 0;
        real at_dot    = (av_x_dot + alpha_dot * a_vz) * cos(alpha) -
                      (av_z_dot - alpha_dot * a_vx) * sin(alpha);
        return (Eigen::Vector4<real>() << at_dot, omega_vx,
                alpha_dot + omega_vy, omega_vz)
            .finished();
    }

    VectorXreal df_forward_nominal_2(const Vector3real &v, const Vector3real &a,
                                     const Vector3real &j)
    {
        // auto& yb_prev = data.yb_prev;

        realV3 &eta      = data.eta;
        realV3 &zb_fixed = data.zb_fixed;
        // realV3 temp = v.cross(eta);
        realV3 &xb         = data.xb;
        realV3 &yb         = data.yb;
        realV3 &zb         = data.zb;
        auto   &attitude   = data.attitude;
        auto   &q          = data.q;
        auto   &alpha      = data.alpha;
        auto   &at         = data.at;
        auto   &magic      = data.magic;
        auto   &yb_prev    = data.yb_prev;
        realV3 &aeroForceB = data.aeroForceB;

        eta         = a - g;
        realV3 temp = v.cross(eta);
        // std::cout << "sing" << temp.dot(yb_prev) << "\n";
        temp.normalize();

        //  do this r is always the same sign? give me an answer?
        int r = sgn(temp.dot(yb_prev));
        yb    = r * (temp);
        // cout << "----------" << endl;
        // caculate alpha
        //  -150~-15  -15~15 15~150
        constexpr real sin_pre = 1e-1;
        if (eta.squaredNorm() < sin_pre)
        {
            std::cout << "[error singularity of eta]" << std::endl;
        }
        real           gamma     = r * atan2(v.cross(eta).norm(), v.dot(eta));
        constexpr real sin_gamma = 5 * M_PI / 180;

        //        std::cerr << "nominal case input gamma\n" << gamma << "\n";
        //        std::cerr << "nominal case input va\n" << v << "\n";
        //        std::cerr << "nominal case input eta\n" << eta << "\n";
        real V = v.norm();

        real h = (2 * tailsitter_model.mass_ * (eta.norm())) /
                 (tailsitter_model.rho_ * V * V * tailsitter_model.area_);
        // real &alpha = data.alpha;
        alpha = tailsitter_model.computeAlpha(h, gamma);
        realV3 vb;
        vb << V * cos(alpha), 0, V * sin(alpha);
        at << eta.norm() * cos(gamma - alpha) -
                  tailsitter_model.getAeroForceAccB(vb)(0),
            0, 0;
        aeroForceB = tailsitter_model.getAeroForceB(vb);

        if (abs(gamma) < sin_gamma)
        {
            yb = zb_fixed.cross(eta);
            yb.normalize();

            int r2 = sgn(yb.dot(yb_prev));

            yb = r2 * yb;
        }

        Eigen::AngleAxis<real> rot(alpha, yb);
        xb = rot * v;
        xb.normalize();

        zb = xb.cross(yb);
        zb.normalize();
        // realM3 &Attitude = data.attitude;
        attitude << xb, yb, zb;

        // auto &q = data.q;
        q = Eigen::Quaternion<real>(attitude);

        real &h1 = data.h1;
        h1       = yb.dot(a);

        realV3 &h2 = data.h2;
        h2         = j - (1 / tailsitter_model.mass_) * attitude *
                     tailsitter_model.getPartialDerivativeOfFbOverVb(vb) *
                     attitude.transpose() * a;
        Eigen::Matrix<real, 1, 4> &N1 = data.N1;
        N1 << 0, v.transpose() * attitude * skew(realV3::UnitY());
        Eigen::Matrix<real, 3, 4> &N2 = data.N2;
        realM3 temp1 = -1 * skew((at.norm() * realV3::UnitX()) +
                                 (tailsitter_model.getAeroForceAccB(vb)));
        realM3 temp2 =
            ((1 / tailsitter_model.mass_) *
             tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * skew(vb));
        N2.col(0)            = attitude * realV3::UnitX();
        N2.block<3, 3>(0, 1) = attitude * (temp1 + temp2);

        if (abs(gamma) < sin_gamma)
        {
            // yb change to non singularity
            h1 = ((skew(zb_fixed) * j).transpose() * zb).value();
            N1.block<1, 3>(0, 1) =
                (zb_fixed.cross(eta)).norm() * (realV3::UnitX().transpose());
        }
        //  TODO in previous setting, I wanna zb to change according last
        //  setting to minimize the yaw change, now turn to back on else {
        //    zb_fixed = zb;
        //  }
        Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
        // Eigen::Vector4<real> &magic = data.magic;

        zb_fixed = zb;
        magic(0) = at(0);
        // magic(0) = alpha;
        magic.tail(3) = lora.tail(3);
        yb_prev       = yb;
        return magic;
        // return (Eigen::Vector2<real>()<<at.norm(),0).finished();

        // return p;
    }

    VectorXreal df_forward_singularity_vel(const Vector3real &v,
                                           const Vector3real &a,
                                           const Vector3real &j)
    {
        // at is eta//
        realV3 &eta = data.eta;
        // realV3 temp = v.cross(eta);
        realV3 &xb       = data.xb;
        realV3 &yb       = data.yb;
        realV3 &zb       = data.zb;
        auto   &q        = data.q;
        auto   &at       = data.at;
        auto   &magic    = data.magic;
        realV3 &zb_fixed = data.zb_fixed;
        auto   &yb_prev  = data.yb_prev;

        eta = a - g;
        at  = eta;
        xb  = at;
        xb.normalize();

        real   scaling   = exp(-v.norm());
        realV3 yb_from_v = v.cross(eta);
        yb_from_v.normalize();
        yb = zb_fixed.cross(xb);
        yb.normalize();
        yb = yb_from_v * (1 - scaling) + yb * scaling;
        yb.normalize();
        zb = xb.cross(yb);
        zb.normalize();

        auto   e1       = realV3::UnitX();
        realV3 xb_fixed = (yb_prev.cross(zb_fixed));
        xb_fixed.normalize();
        realM3 rot = (realM3() << xb, yb, zb).finished();
        q          = Eigen::Quaternion<real>(rot);
        Eigen::Matrix<real, 3, 4> N2;
        Eigen::Vector3<real>      N2_1    = rot * e1;
        real                      at_norm = at.norm();
        Eigen::Matrix<real, 3, 3> N2_2    = -at_norm * rot * skew(e1);

        N2 << N2_1, N2_2;

        Eigen::Matrix<real, 1, 4> N1;

        Eigen::Matrix<real, 1, 3> N1_2 =
            (zb_fixed.cross(eta)).norm() * e1.transpose();
        N1 << 0, N1_2;
        // out << "norm " << N1.tail(3) * e1 << "\n";
        real                  h1 = (skew(zb) * j).transpose() * zb;
        tailsitter_df::realV3 h2 = j;

        Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
        // Eigen::Vector4<real> magic;
        yb       = -yb;
        real avx = v.transpose() * a;
        magic(0) = avx;
        // magic(0) = alpha;
        magic.tail(3) = lora.tail(3);

        // zb_fixed = zb;
        return magic;
    }

    VectorXreal df_forward(const VectorXreal &z)
    {
        // Vector3real p = z.segment(0, 3); // the input vector x with 5
        // variables
        Vector3real v = z.segment(0, 3);  // the input vector x with 5 variables
        Vector3real a = z.segment(3, 3);  // the input vector x with 5 variables
        Vector3real j = z.segment(6, 3);  // the input vector x with 5 variables

        if (v.norm() < vThreshold)
        {
            return df_forward_singularity_vel(v, a, j);
        }
        else
        {
            return df_forward_nominal(v, a, j);
        }
    }

    Eigen::MatrixXd df_backward_auto(autodiff::VectorXreal &P, double &thrust,
                                     Eigen::Vector3d &omega)
    {
        static autodiff::VectorXreal F_auto;
        J = autodiff::jacobian(tailsitter_df::df_forward,
                               autodiff::detail::wrt(P),
                               autodiff::detail::at(P), F_auto);

        tailsitter_df::toEigen(F_auto, thrust, omega);
        return J;
    }

    Eigen::MatrixXd df_backward(autodiff::VectorXreal &P)
    {
        static autodiff::VectorXreal F;
        J = autodiff::jacobian(tailsitter_df::df_forward,
                               autodiff::detail::wrt(P),
                               autodiff::detail::at(P), F);
        return J;
    }

    void jacobian_at(Eigen::Vector3d &gradv, Eigen::Vector3d &grada,
                     Eigen::Vector3d &gradj)
    {
        // Assuming Vector3d;
        static Eigen::RowVector4d acc_thrust_index =
            (Eigen::RowVector4d() << 1, 0, 0, 0).finished();
        gradv = (acc_thrust_index * J).head(3).transpose();
        grada = (acc_thrust_index * J).segment(3, 3).transpose();
        gradj = (acc_thrust_index * J).tail(3).transpose();
    }

    void jacobian_omega(Eigen::Matrix3d &gradv, Eigen::Matrix3d &grada,
                        Eigen::Matrix3d &gradj)
    {
        // for example gradv =
        // [
        //   o_1_v1, o_2_v1, o_3_v1,
        //   o_1_v2, o_2_v2, o_3_v2,
        //   o_1_v3, o_2_v3, o_3_v3,
        // ]
        gradv = J.block<3, 3>(1, 0).transpose();
        grada = J.block<3, 3>(1, 3).transpose();
        gradj = J.block<3, 3>(1, 6).transpose();
    }

    Eigen::Matrix3d &getVelJacobOmega()
    {
        // for example gradv =
        // [
        //   o_1_v1, o_2_v1, o_3_v1,
        //   o_1_v2, o_2_v2, o_3_v2,
        //   o_1_v3, o_2_v3, o_3_v3,
        // ]
        static Eigen::Matrix3d gradv = J.block<3, 3>(1, 0).transpose();
        gradv                        = J.block<3, 3>(1, 0).transpose();
        return gradv;
    }

    Eigen::Matrix3d &getAccJacobOmega()
    {
        // for example gradv =
        // [
        //   o_1_v1, o_2_v1, o_3_v1,
        //   o_1_v2, o_2_v2, o_3_v2,
        //   o_1_v3, o_2_v3, o_3_v3,
        // ]
        static Eigen::Matrix3d grada = J.block<3, 3>(1, 3).transpose();
        grada                        = J.block<3, 3>(1, 3).transpose();
        return grada;
    }

    Eigen::Matrix3d &getJerJacobOmega()
    {
        // for example gradv =
        // [
        //   o_1_v1, o_2_v1, o_3_v1,
        //   o_1_v2, o_2_v2, o_3_v2,
        //   o_1_v3, o_2_v3, o_3_v3,
        // ]
        static Eigen::Matrix3d gradj = J.block<3, 3>(1, 6).transpose();
        gradj                        = J.block<3, 3>(1, 6).transpose();
        return gradj;
    }

    void delimiter()
    {
        std::cout << "-----------------------\n";
    }

    void df_test()
    {
        VectorXreal z;
        z.resize(9);
        init(z);

        std::cout << df_backward(z);
        delimiter();
        // Eigen::Vector3d gradv, grada, gradj;
        Eigen::Matrix3d gradv, grada, gradj;
        jacobian_omega(gradv, grada, gradj);
        delimiter();

        std::cout << gradv << "\n";
        delimiter();

        std::cout << grada << "\n";
        delimiter();

        std::cout << gradj << "\n";

        z(8) = 0;
        delimiter();

        delimiter();

        delimiter();

        delimiter();

        std::cout << df_backward(z);

        delimiter();

        std::cout << getVelJacobOmega() << "\n";

        delimiter();

        std::cout << getAccJacobOmega() << "\n";
        delimiter();

        std::cout << getJerJacobOmega() << "\n";
    }

}  // namespace tailsitter_df
