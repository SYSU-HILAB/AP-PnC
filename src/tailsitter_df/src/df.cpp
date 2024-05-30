// C++ includes
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <cassert>
#include <tailsitter_df/df.hpp>
#include <utils/utils_files.hpp>

// use abort to signal coredump
#include <cstdlib>

#define out std::cout

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
        type_ = Model::Type::LowAlpha;
        // Update staitc parameter;
        YAML::Node config =
            YAML::LoadFile(utils::files::get_config_file_path("tailsitter_df"));
        getPhi() << config["phi1"][0].as<double>(),
            config["phi1"][1].as<double>(), config["phi1"][2].as<double>(),
            config["phi2"][0].as<double>(), config["phi1"][1].as<double>(),
            config["phi1"][2].as<double>(), config["phi3"][0].as<double>(),
            config["phi3"][1].as<double>(), config["phi3"][2].as<double>();

        getBias() << config["bias"][0].as<double>(),
            config["bias"][1].as<double>(), config["bias"][2].as<double>();

        mass_      = config["mass"].as<double>();
        area_      = config["area"].as<double>();
        rho_       = config["air_density"].as<double>();
        alpha_rad_ = 0;

        scale_ = rho_ * area_ / 2;
    }

    realM3 &Model::getPhi()
    {
        if (type_ == Model::Type::HighPositveAlpha)
        {
            return Phis_[0];
        }
        if (type_ == Model::Type::LowAlpha)
        {
            return Phis_[1];
        }
        if (type_ == Model::Type::HighNegativeAlpha)
        {
            return Phis_[2];
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
        if (type_ == Model::Type::HighPositveAlpha)
        {
            return bias_[0];
        }
        if (type_ == Model::Type::LowAlpha)
        {
            return bias_[1];
        }
        if (type_ == Model::Type::HighNegativeAlpha)
        {
            return bias_[2];
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
            std::cout << "warning alpha" << "\n";
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
            // real cmp = 1e4;
            // if(N.norm() * (N.inverse()).norm() > cmp)
            // std::cout << "condition number " << N.norm() *
            // (N.inverse()).norm() <<
            // "\n"; std::cout << "not Singularity N" << N << "\n";

            return N.inverse() * h;
        }
    }

    VectorXreal df_forward_nominal(const Vector3real &v, const Vector3real &a,
                                   const Vector3real &j)
    {
        // auto& yb_prev = data.yb_prev;

        realV3 &eta      = data.eta;
        realV3 &zb_fixed = data.zb_fixed;
        // realV3 temp = v.cross(eta);
        realV3 &xb       = data.xb;
        realV3 &yb       = data.yb;
        realV3 &zb       = data.zb;
        auto   &attitude = data.attitude;
        auto   &q        = data.q;
        auto   &alpha    = data.alpha;
        auto   &at       = data.at;
        auto   &magic    = data.magic;
        auto   &yb_prev  = data.yb_prev;

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
        q           = Eigen::Quaternion<real>(attitude);
        realV3 vbXX = attitude.transpose() * v;

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
        auto   &attitude = data.attitude;
        auto   &q        = data.q;
        auto   &alpha    = data.alpha;
        auto   &at       = data.at;
        auto   &magic    = data.magic;
        realV3 &zb_fixed = data.zb_fixed;
        auto   &yb_prev  = data.yb_prev;

        eta = a - g;
        at  = eta;
        xb  = at;
        xb.normalize();

        yb = zb_fixed.cross(xb);
        yb.normalize();
        zb = xb.cross(yb);
        zb.normalize();
        // out << "xb " << xb.transpose() << " norm " << xb.norm()<< "\n";
        // out << "yb " << yb.transpose() << " norm " << yb.norm()<< "\n";
        // out << "zb " << zb.transpose() << " norm " << zb.norm()<< "\n";
        // out << "zb fixed" << zb_fixed.transpose() << " norm " <<
        // zb_fixed.norm()<<
        // "\n";
        auto   e1       = realV3::UnitX();
        realV3 xb_fixed = (yb_prev.cross(zb_fixed));
        xb_fixed.normalize();
        realM3 rot = (realM3() << xb, yb, zb).finished();
        // rot << xb_fixed, yb_prev, zb_fixed;
        // out << "rot\n" << rot << "\n";
        // out << "rot det " << rot.determinant() << "\n";
        // rot.colse
        q = Eigen::Quaternion<real>(rot);

        // construct N1 N2 h1 h2;
        // out << "e1\n" << e1 << "\n";
        Eigen::Matrix<real, 3, 4> N2;
        Eigen::Vector3<real>      N2_1 = rot * e1;
        // out << "rot e1\n" << N2_1 << "\n";
        // out << "at\n" << at << "\n";
        // out << "at norm\n" << at.norm() << "\n";
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

        //  std::cerr << "N1: " << N1 << "\n";
        //  std::cerr << "N2: \n" << N2 << "\n";
        //  std::cerr << "h1: \n" << h1 << "\n";
        //  std::cerr << "h2: \n" << h2 << "\n";
        //  std::cerr << "xb: \n" << xb << "\n";
        //  std::cerr << "yb: \n" << yb << "\n";
        //  std::cerr << "zb: \n" << zb << "\n";
        Eigen::Matrix<real, 3, 3> temp = at_norm * rot * skew(e1);

        Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
        // Eigen::Vector4<real> magic;
        magic(0) = at.norm();
        // magic(0) = alpha;
        magic.tail(3) = lora.tail(3);

        // zb_fixed = zb;
        return magic;
    }

    // VectorXreal df_forward_singularity_gamma(const Vector3real &v,
    //                                          const Vector3real &a,
    //                                          const Vector3real &j) {
    //   // auto& yb_prev = data.yb_prev;
    //   realV3 &eta = data.eta;
    //   realV3 &zb_fixed = data.zb_fixed;
    //   realV3 temp = v.cross(eta);
    //   realV3 &xb = data.xb;
    //   realV3 &yb = data.yb;
    //   realV3 &zb = data.zb;
    //   auto &attitude = data.attitude;
    //   auto &q = data.q;
    //   auto &alpha = data.alpha;
    //   auto &at = data.at;
    //   auto &magic = data.magic;
    //   auto &yb_prev = data.yb_prev;
    //
    //   eta = a - g;
    //   yb = eta.cross(zb_fixed);
    //   yb.normalize();
    //
    //   int r = sgn(yb.dot(yb_prev));
    //   yb = r * (yb);
    //   constexpr real sin_pre = 1e-1;
    //   if (eta.squaredNorm() < sin_pre) {
    //     std::cerr << "[error singularity of eta]" << std::endl;
    //   }
    //   real gamma = r * atan2((eta.cross(v)).norm(), eta.dot(v));
    //   if (gamma < 5 * M_PI/180)
    //   {
    //     abort();
    //   }
    //   real V = v.norm();
    //   real h = (2 * tailsitter_model.mass_ * (eta.norm())) /
    //            (tailsitter_model.rho_ * V * V * tailsitter_model.area_);
    //   // real &alpha = data.alpha;
    //   alpha = tailsitter_model.computeAlpha(h, gamma);
    //
    //   Eigen::AngleAxis<real> rot(alpha, yb);
    //   xb = rot * v;
    //   xb.normalize();
    //
    //   zb = xb.cross(yb);
    //   zb.normalize();
    //   realM3 &Attitude = data.attitude;
    //   Attitude << xb, yb, zb;
    //
    //   // auto &q = data.q;
    //   q = Eigen::Quaternion<real>(Attitude);
    //   realV3 vb = Attitude.transpose() * v;
    //
    //   at << eta.norm() * cos(gamma - alpha) -
    //             tailsitter_model.getAeroForceB(vb)(0) /
    //             tailsitter_model.mass_,
    //       0, 0;
    //
    //   real &h1 = data.h1;
    //   h1 = yb.dot(a);
    //
    //   realV3 &h2 = data.h2;
    //   h2 = j - (1 / tailsitter_model.mass_) * Attitude *
    //                tailsitter_model.getPartialDerivativeOfFbOverVb(vb) *
    //                Attitude.transpose() * a;
    //   Eigen::Matrix<real, 1, 4> &N1 = data.N1;
    //   N1 << 0, v.transpose() * Attitude * skew(realV3::UnitY());
    //   Eigen::Matrix<real, 3, 4> &N2 = data.N2;
    //   realM3 temp1 =
    //       -1 *
    //       skew((at.norm() * realV3::UnitX()) +
    //            (tailsitter_model.getAeroForceB(vb) * (1 /
    //            tailsitter_model.mass_)));
    //   realM3 temp2 =
    //       ((1 / tailsitter_model.mass_) *
    //        tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * skew(vb));
    //   N2.col(0) = Attitude * realV3::UnitX();
    //   N2.block<3, 3>(0, 1) = temp1 + temp2;
    //   Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
    //   // Eigen::Vector4<real> &magic = data.magic;
    //   magic(0) = at.norm();
    //   // magic(0) = alpha;
    //   magic.tail(3) = lora.tail(3);
    //   return magic;
    //   // return (Eigen::Vector2<real>()<<at.norm(),0).finished();
    //
    //   // return p;
    // }

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

    Eigen::Matrix3d &get_v_jacobian_omega()
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

    Eigen::Matrix3d &get_a_jacobian_omega()
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

    Eigen::Matrix3d &get_j_jacobian_omega()
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

        std::cout << get_v_jacobian_omega() << "\n";

        delimiter();

        std::cout << get_a_jacobian_omega() << "\n";
        delimiter();

        std::cout << get_j_jacobian_omega() << "\n";
    }

}  // namespace tailsitter_df
