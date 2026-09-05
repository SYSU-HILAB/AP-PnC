/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Paper:
 * Aerodynamic Prior-free Trajectory Generation and Tracking Control for a
 * Tail-sitter UAV.
 */

// C++ includes
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <cassert>
#include <cstdlib>
#include <tailsitter_df/tailsitter_df.hpp>
#include <vector>

constexpr auto &out = std::cout;

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using namespace autodiff;
namespace tailsitter_df
{

  Eigen::Vector3d &getVelJacobThrAcc();
  Eigen::Vector3d &getAccJacobThrAcc();
  Eigen::Vector3d &getJerJacobThrAcc();

  Eigen::Matrix3d &getVelJacobOmega();
  Eigen::Matrix3d &getAccJacobOmega();
  Eigen::Matrix3d &getJerJacobOmega();
  void             toEigen(const VectorXreal &F, double &acc_thrust,
                           Eigen::Vector3d &omega);
  void             init(VectorXreal &z);
  VectorXreal      init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &jerk);
  // differential flatness forward, returns at_norm and angular rate
  VectorXreal     df_forward_nominal(const Vector3real &v, const Vector3real &a,
                                     const Vector3real &j);
  VectorXreal     df_forward_singularity_vel(const Vector3real &v,
                                             const Vector3real &a,
                                             const Vector3real &j);
  VectorXreal     df_forward(const VectorXreal &z);
  Eigen::MatrixXd df_backward(autodiff::VectorXreal &P);

  void df_test();

  class Model
  {
   private:
    realM3 phi_;
    double mass_ = 2;

    Model();
    Model(const Model &)            = delete;
    Model &operator=(const Model &) = delete;

   public:
    static Model    &getInstance();
    realM3          &getPhi();
    Eigen::Vector3d &Phi_z();

    double aeroForceZ(const Eigen::Vector3d &vb);
    realV3 getBAeroForce(const realV3 &vb);
    realM3 getPartialDerivativeOfFbOverVb(const realV3 &vb);
    realV3 getAeroForceAccB(const realV3 &vb);
  };

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
    // Update staitc parameter;
    const char *env_p = std::getenv("AP_PNC_DIR");
    std::string sv(env_p);
    sv += "/config/tailsitter_df.yaml";
    YAML::Node config = YAML::LoadFile(sv);

    const auto phi = config["phi"].as<std::vector<double>>();
    for (int i = 0; i < static_cast<int>(phi.size()); i++)
    {
      getPhi()(i) = phi[i];
    }

    mass_ = config["mass"].as<double>();
  }

  realM3 &Model::getPhi()
  {
    return phi_;
  };

  Eigen::Vector3d &Model::Phi_z()
  {
    static Eigen::Vector3d temp;
    temp << getPhi().row(2)(0).val(), getPhi().row(2)(1).val(),
        getPhi().row(2)(2).val();
    return temp;
  };

  realV3 Model::getBAeroForce(const realV3 &vb)
  {
    real   V    = vb.norm();
    realV3 temp = V * getPhi() * vb;
    temp        = temp * mass_;
    return temp;
  };

  realV3 Model::getAeroForceAccB(const realV3 &vb)
  {
    real V = vb.norm();
    return V * getPhi() * vb;
  };

  void toEigen(const VectorXreal &F, double &acc_thrust, Eigen::Vector3d &omega)
  {
    acc_thrust = F(0).val();
    omega << F(1).val(), F(2).val(), F(3).val();
  }

  using namespace std;

  VectorXreal df_forward_nominal(const realV3 &v, const realV3 &a,
                                 const realV3 &j)
  {
    realV3 &xb = data.xb;
    realV3 &yb = data.yb;
    realV3 &zb = data.zb;

    auto   &attitude        = data.attitude;
    auto   &q               = data.q;
    auto   &alpha           = data.alpha;
    auto   &at              = data.at;
    realV3 &B_aero_force    = data.B_aero_force;
    real   &B_aero_force_z  = data.B_aero_force_z;
    real   &S_ap_x        = data.S_ap_x;
    real   &S_ap_z        = data.S_ap_z;
    real   &B_Omega_b_y   = data.B_Omega_b_y;
    real   &alpha_dot     = data.alpha_dot;
    real   &S_Omega_x     = data.S_Omega_x;
    real   &S_Omega_y     = data.S_Omega_y;
    real   &S_Omega_z     = data.S_Omega_z;
    real   &S_ap_x_dot    = data.S_ap_x_dot;
    real   &S_ap_z_dot    = data.S_ap_z_dot;

    realV3 x_s    = v / v.norm();
    realV3 temp   = -(realM3::Identity() - x_s * x_s.transpose()) * (a - g);
    realV3 z_s    = temp / temp.norm();
    realV3 y_s    = z_s.cross(x_s);
    real   temp_x = x_s.transpose() * (a - g);
    real   temp_z = z_s.transpose() * (a - g);
    S_ap_x        = temp_x;
    S_ap_z        = temp_z;

    real tan_acc = x_s.transpose() * a;

    real V        = v.norm();
    real omega_sz = (y_s.transpose() * g);
    omega_sz      = omega_sz / V;
    real omega_sx = (y_s.transpose() * j);
    omega_sx      = ((omega_sz * S_ap_x) - omega_sx) / S_ap_z;
    // real omega_sy = (z_s.transpose() * g);
    // omega_sy      = (-S_ap_z - omega_sy) / V;
    real omega_sy = -z_s.transpose() * a;
    omega_sy      = omega_sy / V;
    // omega_sy = () / V;

    double phi_zz              = tailsitter_model.getPhi().row(2)(2).val();
    B_aero_force               = tailsitter_model.getBAeroForce(v);
    B_aero_force_z             = B_aero_force(2);
    real             V_squared = V * V;
    constexpr double phi_33    = -0.09408;
    alpha                      = atan2(-S_ap_z, S_ap_x - phi_33 * V_squared);

    yb    = y_s;
    xb    = cos(alpha) * x_s - sin(alpha) * z_s;
    zb    = cos(alpha) * z_s + sin(alpha) * x_s;
    at(0) = S_ap_x * cos(alpha) - S_ap_z * sin(alpha);
    attitude << xb, yb, zb;
    q               = Eigen::Quaternion<real>(attitude);

    S_ap_z_dot      = z_s.transpose() * j;
    S_ap_z_dot      = S_ap_z_dot + omega_sy * S_ap_x;

    S_ap_x_dot      = x_s.transpose() * j;
    S_ap_x_dot      = S_ap_x_dot - omega_sy * S_ap_z;

    realV3 vb = attitude.transpose() * v;

    real S_a_x = x_s.transpose() * a;
    real S_a_z = z_s.transpose() * a;

    real phi_dot_xx = phi_33 * 2 * v.transpose() * a;

    real increment = S_ap_x - phi_33 * V_squared;
    //
    real denominator = (S_ap_z * S_ap_z + increment * increment);

    // real numerator = (S_ap_x_dot - phi_dot_xx);

    real acc_on_vel = v.transpose() * a;

    real numerator1 =
        phi_33 * (V_squared * S_ap_z_dot - 2 * S_ap_z * acc_on_vel);

    real numerator2 = S_ap_z * S_ap_x_dot - S_ap_x * S_ap_z_dot;

    real numerator = numerator1 + numerator2;

    alpha_dot = numerator / denominator;

    // A = Sapz ; B = Sapx;

    // procedure1
    // realV3 ab   = attitude.transpose() * a;
    // real   ab_x = x_s.transpose() * a;
    // real   ab_z = z_s.transpose() * a;

    // real infs = ab_z * cos(alpha) - ab_x * sin(alpha);
    // infs      = infs / V;
    // alpha_dot = (infs - omega_sy) / 2.0;

    // real term1 = zb.transpose() * a;
    // term1      = term1 * vb(0);
    // real term2 = xb.transpose() * a;
    // term2      = term2 * vb(2);
    // real V2    = v.transpose() * v;
    // alpha_dot  = (term1 - term2);
    // alpha_dot  = alpha_dot / V2;

    // real term1 = S_a_x * cos(alpha);
    // real term2 = S_a_z * sin(alpha);
    // alpha_dot  = term1 - term2;
    // alpha_dot  = alpha_dot / V;
    // alpha_dot  = alpha_dot - omega_sy;
    // alpha_dot = S_a_z / V;
    // alpha_dot = alpha_dot - omega_sy;
    // B_omega_s_y = S_omega_s_y;
    // alpha_dot = alpha_dot - omega_sy;
    // alpha_dot = alpha_dot / 2.0;

    // real at_dot = (S_ap_x_dot - alpha_dot * S_ap_z) * cos(alpha) -
    //               (S_ap_z_dot + alpha_dot * S_ap_x) * sin(alpha);

    B_Omega_b_y = omega_sy + alpha_dot;
    S_Omega_x   = omega_sx;
    S_Omega_y   = omega_sy;
    S_Omega_z   = omega_sz;

    return (Eigen::Vector4<real>() << tan_acc, omega_sx, B_Omega_b_y, omega_sz)
        .finished();
  }

  VectorXreal df_forward_singularity_vel(const Vector3real &v,
                                         const Vector3real &a,
                                         const Vector3real &)
  {
    realV3 &eta      = data.eta;
    realV3 &xb       = data.xb;
    realV3 &yb       = data.yb;
    realV3 &zb       = data.zb;
    auto   &q        = data.q;
    auto   &at       = data.at;
    realV3 &zb_fixed = data.zb_fixed;

    eta = a - g;
    at  = eta;
    xb  = at;
    xb.normalize();

    real   scaling   = v.norm() / vThreshold;
    realV3 yb_from_v = v.cross(eta);
    yb_from_v.normalize();
    realV3 yb_from_fixed_frame = zb_fixed.cross(xb);
    yb_from_fixed_frame.normalize();
    yb = (1 - scaling) * yb_from_fixed_frame +  scaling * yb_from_v;
    yb.normalize();
    zb = xb.cross(yb);
    zb.normalize();

    realM3 rot = (realM3() << xb, yb, zb).finished();
    q          = Eigen::Quaternion<real>(rot);

    return Eigen::Vector4<real>::Zero();
  }

  VectorXreal df_forward(const VectorXreal &z)
  {
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

  Eigen::MatrixXd df_backward(autodiff::VectorXreal &P)
  {
    static autodiff::VectorXreal F;
    data.J =
        autodiff::jacobian(tailsitter_df::df_forward, autodiff::detail::wrt(P),
                           autodiff::detail::at(P), F);
    return data.J;
  }

  Eigen::MatrixXd Data::df_backward_auto(const Eigen::Vector3d &vel,
                                         const Eigen::Vector3d &acc,
                                         const Eigen::Vector3d &jerk,
                                         double &thrust, Eigen::Vector3d &omega)
  {
    static autodiff::VectorXreal F_auto;
    VectorXreal                  P = init(vel, acc, jerk);
    J = autodiff::jacobian(tailsitter_df::df_forward, autodiff::detail::wrt(P),
                           autodiff::detail::at(P), F_auto);

    tailsitter_df::toEigen(F_auto, thrust, omega);
    return J;
  }

  void jacobian_at(Eigen::Vector3d &gradv, Eigen::Vector3d &grada,
                   Eigen::Vector3d &gradj)
  {
    // Assuming Vector3d;
    static Eigen::RowVector4d acc_thrust_index =
        (Eigen::RowVector4d() << 1, 0, 0, 0).finished();
    gradv = (acc_thrust_index * data.J).head(3).transpose();
    grada = (acc_thrust_index * data.J).segment(3, 3).transpose();
    gradj = (acc_thrust_index * data.J).tail(3).transpose();
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
    gradv = data.J.block<3, 3>(1, 0).transpose();
    grada = data.J.block<3, 3>(1, 3).transpose();
    gradj = data.J.block<3, 3>(1, 6).transpose();
  }

  Eigen::Matrix3d &getVelJacobOmega()
  {
    // for example gradv =
    // [
    //   o_1_v1, o_2_v1, o_3_v1,
    //   o_1_v2, o_2_v2, o_3_v2,
    //   o_1_v3, o_2_v3, o_3_v3,
    // ]
    static Eigen::Matrix3d gradv = data.J.block<3, 3>(1, 0).transpose();
    gradv                        = data.J.block<3, 3>(1, 0).transpose();
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
    static Eigen::Matrix3d grada = data.J.block<3, 3>(1, 3).transpose();
    grada                        = data.J.block<3, 3>(1, 3).transpose();
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
    static Eigen::Matrix3d gradj = data.J.block<3, 3>(1, 6).transpose();
    gradj                        = data.J.block<3, 3>(1, 6).transpose();
    return gradj;
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

}  // namespace tailsitter_df
