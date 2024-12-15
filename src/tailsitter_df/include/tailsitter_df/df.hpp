#ifndef TAILSITTER_DF
#define TAILSITTER_DF

#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

namespace tailsitter_df
{
    using namespace autodiff;
    using namespace autodiff::detail;
    using real                      = autodiff::real1st;
    using realV3                    = autodiff::Vector3real;
    using realM3                    = autodiff::Matrix3real;
    constexpr int    size_Models    = 2;
    constexpr double error_alpha    = 100;
    const realV3     g              = (realV3() << 0, 0, 9.8).finished();
    constexpr real   vThreshold     = 1.0;
    constexpr real   gammaThreshold = 5 * M_PI / 180;

    class Model
    {
       public:
        enum class Type
        {
            Plain,
            HighLiftRange,  // typically, it refers to low-alpha and high speed
                            // range.
        };

       private:
        std::array<realM3, size_Models> Phis_;
        std::array<realV3, size_Models> bias_;
        Model();
        Model(const Model &)            = delete;
        Model &operator=(const Model &) = delete;

       public:
        static Model &getInstance();
        realM3       &getPhi();

        // return the body_z row Phi as vector
        Eigen::Vector3d &Phi_z();
        // now Assuming there is no bias
        double aeroForceZ(const Eigen::Vector3d &vb);
        double dAeroForceZ(const Eigen::Vector3d &vb,
                           const Eigen::Vector3d &acc);

        // Eigen::Vector3d& getPartialDfzOverV(const Eigen::Vector3d vb, );
        // Eigen::Vector3d& getPartialDfzOverA();
        realV3 &getBias();
        realV3  getAeroForceB(const realV3 &vb);
        realM3  getPartialDerivativeOfFbOverVb(const realV3 &vb);
        realV3  getAeroForceAccB(const realV3 &vb);

        real computeAlpha(real h, real gamma);
        void triangleFunctionEquationSolver(const real a, const real b,
                                            const real c, Vector2real &sol);

       public:
        // 1/2 * rho * S
        double mass_, rho_, area_, scale_;
        real   alpha_rad_  = 30 * M_PI / 180;
        real   alpha_prev_ = 0;
        Type   type_;
    };

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
        realV3                    aeroForceB = realV3::Zero();
        realV3                    eta;
        realV3                    xb, yb, zb;
        realV3                    vel;
        realV3                    temp;
        realM3                    attitude;
        Eigen::Quaternion<real>   q;
        real                      alpha;
        real                      alpha_prev;
        realV3                    at;
        real                      h1;
        realV3                    h2;
        Eigen::Matrix<real, 1, 4> N1;
        Eigen::Matrix<real, 3, 4> N2;
        Eigen::Matrix<real, 4, 1> magic;
        bool                      tri_success;
        realV3                    yb_prev  = (realV3() << 1, 0, 0).finished();
        realV3                    zb_fixed = (realV3() << 0, 1, 0).finished();

        // 删除拷贝构造函数和赋值操作符
        Data(const Data &)            = delete;
        Data &operator=(const Data &) = delete;

        // 提供全局访问点
        static Data &getInstance()
        {
            static Data instance;
            return instance;
        }

        void reset(Model &model, bool circled = true)
        {
            auto xb_init_always = (realV3() << 0, 0, 1).finished();
            if (circled)
            {
                yb_prev  = (realV3() << 1, 0, 0).finished();
                zb_fixed = (realV3() << 0, 1, 0).finished();
            }
            else
            {
                yb_prev  = (realV3() << 0, -1, 0).finished();
                zb_fixed = (realV3() << 1, 0, 0).finished();
            }
            // another way to calculate zb
            zb_fixed = xb_init_always.cross(yb_prev);
            zb_fixed.normalize();
            yb_prev.normalize();
            zb_fixed.normalize();
            model.alpha_rad_ = 0 * M_PI / 180;
            alpha            = 0;
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
            auto                    r = q.toRotationMatrix();
            Eigen::Quaternion<real> qq;
            realV3                  xx = r * arr;
            xx.normalize();
            auto yy = xx;
            yy(0)   = yy(0) + 0.2;
            yy      = xx.cross(yy);
            yy.normalize();
            auto zz = xx.cross(yy);
            zz.normalize();
            auto mat = (realM3() << xx, yy, zz).finished();
            qq       = mat;
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

    void        init(VectorXreal &z);
    void        toEigen(const VectorXreal &F, double &acc_thrust,
                        Eigen::Vector3d &omega);
    VectorXreal init(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                     const Eigen::Vector3d &jerk);
    realM3      skew(const realV3 &v);
    Eigen::Vector4<real> computeDotAtAndBodyRates(
        const Eigen::Matrix<real, 1, 4> &N1,
        const Eigen::Matrix<real, 3, 4> &N2, const real &h1, const realV3 &h2);

    // differential flatness forward, returns at_norm and angular rate
    VectorXreal df_forward_nominal(const Vector3real &v, const Vector3real &a,
                                   const Vector3real &j);
    VectorXreal df_forward_singularity_vel(const Vector3real &v,
                                           const Vector3real &a,
                                           const Vector3real &j);
    VectorXreal df_forward(const VectorXreal &z);
    // The 'df_backward_auto' interface automatically handles both forward and
    // backward Jacobian calculations. Return is Jacobian.
    Eigen::MatrixXd df_backward_auto(autodiff::VectorXreal &P, double &thrust,
                                     Eigen::Vector3d &omega);
    Eigen::MatrixXd df_backward(autodiff::VectorXreal &P);

    // Limit penalties in at and omega
    void jacobian_at(Eigen::Vector3d &gradv, Eigen::Vector3d &grada,
                     Eigen::Vector3d &gradj);
    void jacobian_omega(Eigen::Matrix3d &gradv, Eigen::Matrix3d &grada,
                        Eigen::Matrix3d &gradj);

    // TODO reoptimize???
    Eigen::Vector3d &getVelJacobThrAcc();
    Eigen::Vector3d &getAccJacobThrAcc();
    Eigen::Vector3d &getJerJacobThrAcc();

    Eigen::Matrix3d &getVelJacobOmega();
    Eigen::Matrix3d &getAccJacobOmega();
    Eigen::Matrix3d &getJerJacobOmega();
    void             df_test();
}  // namespace tailsitter_df

#endif
