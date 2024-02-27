// C++ includes
#include <Eigen/Geometry>
#include <tailsitter_df/df.hpp>
// #include
#define out std::cout
//
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using namespace autodiff;
namespace tailsitter_df
{
  using vec = autodiff::VectorXreal;

  Model &Model::getInstance()
  {
    static Model instance; // 静态局部变量
    return instance;
  }

  Model::Model()
  {
    // using TModel::Type;
    // tailsitter_df::yb_prev.normalize();
    // tailsitter_df::zb_fixed.normalize();
    type_ = Model::Type::LowAlpha;

    // getPhi() << 0, 0, 0,
    //             0, 0, 0,
    //             0.204904741127358, 0, 2.351241594296870;

    // getBias() << 0.2, 0, -0.133799520571864;

    getPhi() << 0, 0, 0,
        0, 0, 0,
        -0.204904741127358, 0, -2.351241594296870;

    getBias() << -0.1, 0, 0.133799520571864;
    mass_ = 1.5;
    area_ = 0.25;
    rho_ = 1.25;
    scale_ = rho_ * area_ / 2;
    alpha_rad_ = 0;
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
    realM3 temp = getPhi() * vb * vb.transpose() / vb.norm() + getPhi() * vb.norm();
    temp += 2 * getBias() * vb.transpose();
    return scale_ * temp;
  };

  real Model::computeAlpha(real h, real gamma)
  {
    auto b = (h * sin(gamma) + getPhi().row(2)(0));
    auto a = (getPhi().row(2)(2) - h * cos(gamma));
    auto c = getBias()(2);

    Vector2real sol;

    triangleFunctionEquationSolver(a, b, c, sol);

    if (sol(0) == error_alpha)
    {
      std::cout << "Wrong Alpha" << std::endl;
    }
    else
    {
      // std::cout << "sol" << sol(0) << " " << sol(1) << std::endl;

      // auto alpha0 = abs(sol(0) - alpha_rad_) < abs(sol(0) - alpha_rad_ - M_2_PI) ? abs(sol(0) - alpha_rad_) : abs(sol(0) - alpha_rad_ - M_2_PI);
      // auto alpha1 = abs(sol(1) - alpha_rad_) < abs(sol(1) - alpha_rad_ - M_2_PI) ? abs(sol(1) - alpha_rad_) : abs(sol(1) - alpha_rad_ - M_2_PI);
      // if(alpha0 < alpha1)
      //   alpha_rad_ = sol(0);
      // else
      //   alpha_rad_ = sol(1);

      // auto alpha = abs(sol(0) - alpha_rad_) < abs(sol(1) - alpha_rad_) ? sol(0) : sol(1);
      // if (sol(0) >= 0 && sol(1) >= 0) {
      //   alpha_rad_ = abs(sol(0) - 0) < abs(sol(1) - 0) ? sol(0) : sol(1);
      // } else if (sol(0) < 0 && sol(1) >= 0) {
      //   alpha_rad_ = sol(1);
      // } else if (sol(1) < 0 && sol(0) >= 0) {
      //   alpha_rad_ = sol(0);
      // } else {
      //   alpha_rad_ = abs(sol(0) - 0) < abs(sol(1) - 0) ? sol(0) : sol(1);
      // }
      // auto alpha = sol(0) > 0 ? sol(0) : sol(1);

      // alpha_rad_ = alpha;
      // std::cout << "alpha" << alpha << std::endl;
      alpha_rad_ = abs(sol(0) - 0) < abs(sol(1) - 0) ? sol(0) : sol(1);
      return alpha_rad_;
    }
    // temp now for conveniency , only use -50 to 50;
    // return 0;
  };
  // void init(const Eigen::Matrix3d Phi[], realV3& bias){

  void Model::triangleFunctionEquationSolver(const real a, const real b, const real c, Vector2real &sol)
  {
    if (a * a + b * b - c * c >= 0)
    {
      data.tri_success = true;
      auto temp = b * sqrt(a * a + b * b - c * c);

      auto sin1 = (a * c + temp) / (a * a + b * b);
      auto cos1 = (c - a * sin1) / b;

      // atan2 maps a coordinate to [-pi,pi]
      sol(0) = atan2(sin1, cos1);

      auto sin2 = (a * c - temp) / (a * a + b * b);
      auto cos2 = (c - a * sin2) / b;
      sol(1) = atan2(sin2, cos2);
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
    z << 4, 6, 5,
        1, 2, 3,
        7, 8, 9;
    // 1, 2, 3;
  }
  VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, const Eigen::Vector3d &jerk)
  {
    VectorXreal z;
    z.resize(9);
    z << vel(0), vel(1), vel(2),
        acc(0), acc(1), acc(2),
        jerk(0), jerk(1), jerk(2);
    return z;
  }

  void toEigen(const VectorXreal &F, double &acc_thrust, Eigen::Vector3d &omega)
  {
    acc_thrust = F(0).val();
    omega << F(1).val(), F(2).val(), F(3).val();
  }

  using namespace std;

  realM3 skew(const realV3 &v)
  {
    realM3 rtn;
    rtn << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

    return rtn;
  }

  Eigen::Vector4<real> computeDotAtAndBodyRates(const Eigen::Matrix<real, 1, 4> &N1, const Eigen::Matrix<real, 3, 4> &N2, const real &h1, const realV3 &h2)
  {
    Eigen::Vector4<real> h;
    h << h1, h2;
    Eigen::Matrix4<real> N;
    N << N1, N2;

    bool is_singular = N.determinant() == 0;
    // std::cout << "deter \n" << N.determinant() << "\n" ;

    if (is_singular)
    {
      // std::cout << "Singularity" << "\n";
      // std::cout << "Singularity N" << N << "\n";
      return Eigen::Vector4<real>::Zero();
    }
    else
    {
      // real cmp = 1e4;
      // if(N.norm() * (N.inverse()).norm() > cmp)
      // std::cout << "condition number " << N.norm() * (N.inverse()).norm() << "\n";
      // std::cout << "not Singularity N" << N << "\n";

      return N.inverse() * h;
    }
  }
  VectorXreal df_forward_nominal(const Vector3real &v, const Vector3real &a, const Vector3real &j)
  {
    // auto& yb_prev = data.yb_prev;
    realV3 &eta = data.eta;
    realV3 &zb_fixed = data.zb_fixed;
    // realV3 temp = v.cross(eta);
    realV3 &xb = data.xb;
    realV3 &yb = data.yb;
    realV3 &zb = data.zb;
    auto &attitude = data.attitude;
    auto &q = data.q;
    auto &alpha = data.alpha;
    auto &at = data.at;
    auto &magic = data.magic;
    auto &yb_prev = data.yb_prev;

    realV3 &aeroForceB = data.aeroForceB;

    eta = a - g;
    realV3 temp = v.cross(eta);
    // std::cout << "sing" << temp.dot(yb_prev) << "\n";
    temp.normalize();
    const int r = sgn(temp.dot(yb_prev));
    yb = r * (temp);
    // cout << "----------" << endl;
    // caculate alpha
    //  -150~-15  -15~15 15~150
    constexpr real sin_pre = 1e-1;
    if (eta.squaredNorm() < sin_pre)
    {
      std::cout << "[error singularity of eta]" << std::endl;
    }
    real gamma = r * atan2(v.cross(eta).norm(), v.dot(eta));
    constexpr real sin_gamma = 5 * M_PI / 180;

    real V = v.norm();

    real h = (2 * tailsitter_model.mass_ * (eta.norm())) / (tailsitter_model.rho_ * V * V * tailsitter_model.area_);
    // real &alpha = data.alpha;
    alpha = tailsitter_model.computeAlpha(h, gamma);
    realV3 vb;
    vb << V * cos(alpha), 0, V * sin(alpha);
    at << eta.norm() * cos(gamma - alpha) - tailsitter_model.getAeroForceAccB(vb)(0), 0, 0;
    aeroForceB = tailsitter_model.getAeroForceB(vb);

    if (abs(gamma) < sin_gamma)
    {
      // std::cout << "sin_gamma " << sin_gamma << "gamma " << gamma << "\n";
      // yb change to non singularity
      // std::cout << " gamma " << gamma << std::endl;
      // std::cout << " eta " << eta.transpose() << std::endl;
      // std::cout << " vb " << vb.transpose() << std::endl;
      // std::cout << " v " << v.transpose() << std::endl;
      // std::cout << " alpha " << alpha << std::endl;
      // std::cout << " forceAccB " << (attitude.inverse() * tailsitter_model.getAeroForceAccB(vb)).transpose() << std::endl;

      yb = zb_fixed.cross(eta);
      yb.normalize();

      const int r2 = yb.dot(yb_prev).val();
      // if(yb.dot(yb_prev).val() != r) {
      //   std::cout << "turn over" << std::endl;
      // };
      // std::cout << "[gamma]\n";
      // std::cout << "alpha " << alpha << "\n";
      // std::cout << "vb " << alpha << "\n";
      // std::cout << "r_sin_gamma " << r2 << "\n";

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
    realV3 vbXX = attitude.transpose() * v;

    real &h1 = data.h1;
    h1 = yb.dot(a);

    realV3 &h2 = data.h2;
    h2 = j - (1 / tailsitter_model.mass_) * attitude * tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * attitude.transpose() * a;
    Eigen::Matrix<real, 1, 4> &N1 = data.N1;
    N1 << 0, v.transpose() * attitude * skew(realV3::UnitY());
    Eigen::Matrix<real, 3, 4> &N2 = data.N2;
    realM3 temp1 = -1 * skew((at.norm() * realV3::UnitX()) + (tailsitter_model.getAeroForceAccB(vb)));
    realM3 temp2 = ((1 / tailsitter_model.mass_) * tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * skew(vb));
    N2.col(0) = attitude * realV3::UnitX();
    N2.block<3, 3>(0, 1) = attitude * (temp1 + temp2);

    if (abs(gamma) < sin_gamma)
    {
      // yb change to non singularity
      h1 = (skew(zb_fixed) * j).transpose() * zb;
      N1.block<1, 3>(0, 1) = (zb_fixed.cross(eta)).norm() * (realV3::UnitX().transpose());
    }
    else
    {
      zb_fixed = zb;
    }
    Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
    // Eigen::Vector4<real> &magic = data.magic;
    magic(0) = at(0);
    // magic(0) = alpha;
    magic.tail(3) = lora.tail(3);
    yb_prev = yb;
    return magic;
    // return (Eigen::Vector2<real>()<<at.norm(),0).finished();

    // return p;
  }

  VectorXreal df_forward_singularity_vel(const Vector3real &v, const Vector3real &a, const Vector3real &j)
  {
    // at is eta//
    realV3 &eta = data.eta;
    // realV3 temp = v.cross(eta);
    realV3 &xb = data.xb;
    realV3 &yb = data.yb;
    realV3 &zb = data.zb;
    auto &attitude = data.attitude;
    auto &q = data.q;
    auto &alpha = data.alpha;
    auto &at = data.at;
    auto &magic = data.magic;
    realV3 &zb_fixed = data.zb_fixed;
    auto &yb_prev = data.yb_prev;

    eta = a - g;
    at = eta;
    xb = at;
    xb.normalize();

    yb = zb_fixed.cross(xb);
    yb.normalize();
    zb = xb.cross(yb);
    zb.normalize();
    // out << "xb " << xb.transpose() << " norm " << xb.norm()<< "\n";
    // out << "yb " << yb.transpose() << " norm " << yb.norm()<< "\n";
    // out << "zb " << zb.transpose() << " norm " << zb.norm()<< "\n";
    // out << "zb fixed" << zb_fixed.transpose() << " norm " << zb_fixed.norm()<< "\n";
    auto e1 = realV3::UnitX();
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
    Eigen::Vector3<real> N2_1 = rot * e1;
    // out << "rot e1\n" << N2_1 << "\n";
    // out << "at\n" << at << "\n";
    // out << "at norm\n" << at.norm() << "\n";
    real at_norm = at.norm();
    Eigen::Matrix<real, 3, 3> N2_2 = -at_norm * rot * skew(e1);
    Eigen::Matrix<real, 3, 3> temp = at_norm * rot * skew(e1);

    N2 << N2_1, N2_2;

    Eigen::Matrix<real, 1, 4> N1;

    Eigen::Matrix<real, 1, 3> N1_2 = (zb_fixed.cross(eta)).norm() * e1.transpose();
    N1 << 0, N1_2;
    // out << "norm " << N1.tail(3) * e1 << "\n";
    real h1 = (skew(zb) * j).transpose() * zb;
    tailsitter_df::realV3 h2 = j;
    Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
    // Eigen::Vector4<real> magic;
    magic(0) = at.norm();
    // magic(0) = alpha;
    magic.tail(3) = lora.tail(3);

    // zb_fixed = zb;
    return magic;
  }

  VectorXreal df_forward_singularity_gamma(const Vector3real &v, const Vector3real &a, const Vector3real &j)
  {
    // auto& yb_prev = data.yb_prev;
    realV3 &eta = data.eta;
    realV3 &zb_fixed = data.zb_fixed;
    realV3 temp = v.cross(eta);
    realV3 &xb = data.xb;
    realV3 &yb = data.yb;
    realV3 &zb = data.zb;
    auto &attitude = data.attitude;
    auto &q = data.q;
    auto &alpha = data.alpha;
    auto &at = data.at;
    auto &magic = data.magic;
    auto &yb_prev = data.yb_prev;

    eta = a - g;
    yb = eta.cross(zb_fixed);
    yb.normalize();
    // auto &magic = data.magic;
    // temp.normalize();
    int r = sgn(yb.dot(yb_prev));
    yb = r * (yb);
    // cout << "----------" << endl;
    // caculate alpha
    //  -150~-15  -15~15 15~150
    constexpr real sin_pre = 1e-1;
    if (eta.squaredNorm() < sin_pre)
    {
      std::cout << "[error singularity of eta]" << std::endl;
    }
    real gamma = r * atan2((eta.cross(v)).norm(), eta.dot(v));

    real V = v.norm();
    real h = (2 * tailsitter_model.mass_ * (eta.norm())) / (tailsitter_model.rho_ * V * V * tailsitter_model.area_);
    // real &alpha = data.alpha;
    alpha = tailsitter_model.computeAlpha(h, gamma);

    Eigen::AngleAxis<real> rot(alpha, yb);
    xb = rot * v;
    xb.normalize();

    zb = xb.cross(yb);
    zb.normalize();
    realM3 &Attitude = data.attitude;
    Attitude << xb, yb, zb;

    // auto &q = data.q;
    q = Eigen::Quaternion<real>(Attitude);
    realV3 vb = Attitude.transpose() * v;

    at << eta.norm() * cos(gamma - alpha) - tailsitter_model.getAeroForceB(vb)(0) / tailsitter_model.mass_, 0, 0;

    real &h1 = data.h1;
    h1 = yb.dot(a);

    realV3 &h2 = data.h2;
    h2 = j - (1 / tailsitter_model.mass_) * Attitude * tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * Attitude.transpose() * a;
    Eigen::Matrix<real, 1, 4> &N1 = data.N1;
    N1 << 0, v.transpose() * Attitude * skew(realV3::UnitY());
    Eigen::Matrix<real, 3, 4> &N2 = data.N2;
    realM3 temp1 = -1 * skew((at.norm() * realV3::UnitX()) + (tailsitter_model.getAeroForceB(vb) * (1 / tailsitter_model.mass_)));
    realM3 temp2 = ((1 / tailsitter_model.mass_) * tailsitter_model.getPartialDerivativeOfFbOverVb(vb) * skew(vb));
    N2.col(0) = Attitude * realV3::UnitX();
    N2.block<3, 3>(0, 1) = temp1 + temp2;
    Eigen::Vector4<real> lora = computeDotAtAndBodyRates(N1, N2, h1, h2);
    // Eigen::Vector4<real> &magic = data.magic;
    magic(0) = at.norm();
    // magic(0) = alpha;
    magic.tail(3) = lora.tail(3);
    return magic;
    // return (Eigen::Vector2<real>()<<at.norm(),0).finished();

    // return p;
  }

  // VectorXreal df_forward_singularity_gamma(const Vector3real &v, const Vector3real &a, const Vector3real &j)
  // {
  // }
  VectorXreal df(const VectorXreal &z)
  {
    // Vector3real p = z.segment(0, 3); // the input vector x with 5 variables
    Vector3real v = z.segment(0, 3); // the input vector x with 5 variables
    Vector3real a = z.segment(3, 3); // the input vector x with 5 variables
    Vector3real j = z.segment(6, 3); // the input vector x with 5 variables

    if (v.norm() < vThreshold)
    {
      return df_forward_singularity_vel(v, a, j);
    }
    else
    {
      return df_forward_nominal(v, a, j);
    }
    // if ()
    //
    // cout << p.transpose() << endl;
    // cout << v.transpose() << endl;
    // cout << a.transpose() << endl;
    // cout << j.transpose() << endl;

    // typedef Vector3real realV3;
    // typedef Matrix3real realM3;
    // typedef real1st real;
    // auto adf = g(0);
    // realV3 adsf = g(0);
    // cout << "adsf" << adf;
  }
} // namespace df
