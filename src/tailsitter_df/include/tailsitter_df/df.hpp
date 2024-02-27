#pragma once

#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <iostream>
#include <fstream>
// #include "timer.cpp"

namespace tailsitter_df
{
  using namespace autodiff;
  using namespace autodiff::detail;
  typedef autodiff::real1st real;
  typedef autodiff::Vector3real realV3;
  typedef autodiff::Matrix3real realM3;
  constexpr int size_Models = 3;
  constexpr double error_alpha = 100;
  const realV3 g = (realV3() << 0, 0, -9.8).finished();
  constexpr real vThreshold = 2;
  constexpr real gammaThreshold = 5 * M_PI / 180;

  class Model
  {
    // testing only use -50 - 50;
    enum class Type
    {
      HighPositveAlpha,
      HighNegativeAlpha,
      LowAlpha
    };

  private:
    std::array<realM3, size_Models> Phis_;
    std::array<realV3, size_Models> bias_;
    Model();
    Model(const Model &) = delete;
    Model &operator=(const Model &) = delete;

  public:
    static Model &getInstance();
    realM3 &getPhi();
    realV3 &getBias();
    realV3 getAeroForceB(const realV3 &vb);
    realM3 getPartialDerivativeOfFbOverVb(const realV3 &vb);
    realV3 getAeroForceAccB(const realV3 &vb);

    real computeAlpha(real h, real gamma);
    void triangleFunctionEquationSolver(const real a, const real b, const real c, Vector2real &sol);

  public:
    // 1/2 * rho * S
    double mass_, rho_, area_, scale_;
    real alpha_rad_ = 30 * M_PI / 180;
    real alpha_prev_ = 0;
    Type type_;
  };
  void init(VectorXreal &z);
  void toEigen(const VectorXreal &F, double &acc_thrust, Eigen::Vector3d &omega);
  VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, const Eigen::Vector3d &jerk);
  VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, const Eigen::Vector3d &jerk);
  realM3 skew(const realV3 &v);
  Eigen::Vector4<real> computeDotAtAndBodyRates(const Eigen::Matrix<real, 1, 4> &N1, const Eigen::Matrix<real, 3, 4> &N2, const real &h1, const realV3 &h2);
  VectorXreal df_forward_nominal(const Vector3real &v, const Vector3real &a, const Vector3real &j);
  VectorXreal df_forward_singularity_vel(const Vector3real &v, const Vector3real &a, const Vector3real &j);
  VectorXreal df_forward(const VectorXreal &z);

  class Data
  {
  private:
    // 私有构造函数
    Data()
    {
      yb_prev.normalize();
      zb_fixed.normalize();
    }

  public:
    realV3 aeroForceB = realV3::Zero();
    realV3 eta;
    realV3 xb, yb, zb;
    realV3 vel;
    realV3 temp;
    realM3 attitude;
    Eigen::Quaternion<real> q;
    real alpha;
    real alpha_prev;
    realV3 at;
    real h1;
    realV3 h2;
    Eigen::Matrix<real, 1, 4> N1;
    Eigen::Matrix<real, 3, 4> N2;
    Eigen::Matrix<real, 4, 1> magic;
    bool tri_success;
    realV3 yb_prev = (realV3() << 0, -1, 0).finished();
    realV3 zb_fixed = (realV3() << 1, 0, 0).finished();

    // 删除拷贝构造函数和赋值操作符
    Data(const Data &) = delete;
    Data &operator=(const Data &) = delete;

    // 提供全局访问点
    static Data &getInstance()
    {
      static Data instance;
      return instance;
    }

    void reset(Model &model)
    {
      yb_prev = (realV3() << 0, -1, 0).finished();
      zb_fixed = (realV3() << 1, 0, 0).finished();
      yb_prev.normalize();
      zb_fixed.normalize();
      model.alpha_rad_ = 0 * M_PI / 180;
      alpha = 0;
    }

    Eigen::Quaterniond getQuaternion()
    {
      static Eigen::Quaterniond rtn;
      rtn.w() = q.w().val();
      rtn.x() = q.x().val();
      rtn.y() = q.y().val();
      rtn.z() = q.z().val();
      return rtn;
    }

    Eigen::Quaterniond getArrow(const realV3 &arr)
    {
      auto r = q.toRotationMatrix();
      Eigen::Quaternion<real> qq;
      realV3 xx = r * arr;
      xx.normalize();
      auto yy = xx;
      yy(0) = yy(0) + 0.2;
      yy = xx.cross(yy);
      yy.normalize();
      auto zz = xx.cross(yy);
      zz.normalize();
      auto mat = (realM3() << xx, yy, zz).finished();
      qq = mat;
      Eigen::Quaterniond rtn;
      // auto saf = .q.w();
      rtn.w() = qq.w().val();
      rtn.x() = qq.x().val();
      rtn.y() = qq.y().val();
      rtn.z() = qq.z().val();
      return rtn;
      // q.w() = q.w
    }
  };

  // to be extern
  tailsitter_df::Model &tailsitter_model = Model::getInstance();
  Data &data = Data::getInstance();
} // namespace df