/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <tailsitter_df/tailsitter_df.hpp>
#include <vector>

#include "gcopter/geo_utils.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"

namespace gcopter
{
    struct CostConfig
    {
        double  v_max;
        double  omg_x_max, omg_y_max, omg_z_max;
        double  acc_max;
        double  thrust_min, thrust_max;
        double  pos_weight, vel_weight, acc_weight;
        double  omg_x_weight, omg_y_weight, omg_z_weight;
        double  thrust_weight;
        double  time_weight;
        double  omg_consistent_weight;
        double &rho;
        // singleton acquisition
        static CostConfig &getInstance()
        {
            static CostConfig instance;
            return instance;
        }
        // disbale copy and assign
        CostConfig(const CostConfig &)            = delete;
        CostConfig &operator=(const CostConfig &) = delete;
        // turn it to the predefined speed
        // default constructor
       private:
        CostConfig() : rho(time_weight)
        {
            // all to zero
            v_max         = 0.0;
            omg_x_max     = 0.0;
            omg_y_max     = 0.0;
            omg_z_max     = 0.0;
            omg_x_weight  = 0.0;
            omg_y_weight  = 0.0;
            omg_z_weight  = 0.0;
            thrust_min    = 0.0;
            thrust_max    = 0.0;
            pos_weight    = 0.0;
            vel_weight    = 0.0;
            thrust_weight = 0.0;
            time_weight   = 0.0;
        }
    };

    class GCOPTER_PolytopeSFC
    {
       public:
        using PolyhedronV = Eigen::Matrix3Xd;
        using PolyhedronH = Eigen::MatrixX4d;
        using PolyhedraV  = std::vector<PolyhedronV>;
        using PolyhedraH  = std::vector<PolyhedronH>;

       private:
        minco::MINCO_S3NU minco;

        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;

        PolyhedraV       vPolytopes;
        PolyhedraH       hPolytopes;
        Eigen::Matrix3Xd shortPath;

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;

        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int    integralRes;

        // reference to CostConfig
        CostConfig &config = CostConfig::getInstance();

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::VectorXd  times;
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd  gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd  partialGradByTimes;

       private:
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd       &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                                    : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC              &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) =
                        gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV      &vPolys,
                                    Eigen::Matrix3Xd      &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l        = vIdx(i);
                k        = vPolys[l].cols();
                q        = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr, const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int               n      = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double          sqrNormXi = xi.squaredNorm();
            const double          invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi    = xi * invNormXi;
            const Eigen::VectorXd r         = unitXi.head(n - 1);
            const Eigen::Vector3d delta =
                ovPoly.rightCols(n - 1) * r.cwiseProduct(r) + ovPoly.col(1) -
                ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) =
                (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double       c  = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi  &vIdx,
                                     const PolyhedraV &vPolys, EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double                   minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past           = 0;
            tiny_nls_params.delta          = 1.0e-5;
            tiny_nls_params.g_epsilon      = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                // print l , k
                std::cout << "l: " << l << " k: " << k << std::endl;
                ovPoly.resize(3, k + 1);
                ovPoly.col(0)       = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(
                    x, minSqrD, &GCOPTER_PolytopeSFC::costTinyNLS, nullptr,
                    nullptr, &ovPoly, tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd  &xi,
                                         const Eigen::VectorXi  &vIdx,
                                         const PolyhedraV       &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC               &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double          normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l       = vIdx(i);
                k       = vPolys[l].cols();
                q       = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ   = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) =
                    (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i))
                        .array() *
                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) =
                    (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV      &vPolys,
                                               double &cost, EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double          sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q                = xi.segment(j, k);
                sqrNormQ         = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c  = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        static inline bool smoothedL1(const double &x, const double &mu,
                                      double &f, double &df)
        {
            if (x < 0.0)
            {
                f  = 0;
                df = 0;
                return false;
            }
            else if (x > mu)
            {
                f  = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu    = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2  = mu - 0.5 * x;
                f                    = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        static inline void attachPenaltyFunctional(
            const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
            const double &smoothFactor, const int &integralResolution,
            const CostConfig &config, double &cost, Eigen::VectorXd &gradT,
            Eigen::MatrixX3d &gradC)
        {
            const double velSqrMax  = config.v_max * config.v_max;
            const double omgxSqrMax = config.omg_x_max * config.omg_x_max;
            const double omgySqrMax = config.omg_y_max * config.omg_y_max;
            const double omgzSqrMax = config.omg_z_max * config.omg_z_max;

            const double accSqrMax = config.acc_max * config.acc_max;
            const double thrustMean =
                0.5 * (config.thrust_min + config.thrust_max);
            const double thrustRadi =
                0.5 * fabs(config.thrust_max - config.thrust_min);
            const double thrustSqrRadi = thrustRadi * thrustRadi;

            // const double weightPos = penaltyWeights(0);
            const double    weightVel    = config.vel_weight;
            const double    weightAcc    = config.acc_weight;
            const double    weightThrust = config.thrust_weight;
            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc,
                totalGradJer;
            double          thr;
            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            double          gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradAcc;
            Eigen::Vector3d gradOmg;

            double                      step, alpha;
            double                      s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d             outerNormal;
            double                      violaVel, violaAcc;
            double                      violaThrust;
            double                      violaVelPena, violaAccPena;
            double                      violaThrustPena;
            double                      violaVelPenaD, violaAccPenaD;
            double                      violaThrustPenaD;
            double                      node, pena;
            const int                   pieceNum     = T.size();
            const double                integralFrac = 1.0 / integralResolution;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, 6, 3> &c =
                    coeffs.block<6, 3>(i * 6, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    s1       = j * step;
                    s2       = s1 * s1;
                    s3       = s2 * s1;
                    s4       = s2 * s2;
                    s5       = s4 * s1;
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
                    beta0(4) = s4, beta0(5) = s5;
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1,
                    beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3,
                    beta1(5) = 5.0 * s4;
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0,
                    beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2,
                    beta2(5) = 20.0 * s3;
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0,
                    beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0,
                    beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    // auto z = tailsitter_df::init(vel, acc, jer);
                    tailsitter_df::Data::getInstance().df_backward_auto(
                        vel, acc, jer, thr, omg);

                    violaVel = vel.squaredNorm() - velSqrMax;
                    violaAcc = acc.squaredNorm() - accSqrMax;

                    /*
                     * Additional knowledge about violation costs:
                     *
                     * 1. Angular velocity magnitude constraint:
                     *    - Could use: vio_all_omg = omg.squaredNorm()
                     *
                     * 2. Tilt angle constraint:
                     *    - cos_theta = 1.0 - 2.0 * (quat(1)^2 + quat(2)^2)
                     *    - violaTheta = acos(cos_theta) - thetaMax
                     *
                     * 3. Thrust constraint:
                     *    - Uses range cost: (T-T_max)(T-T_min)
                     *    - Penalizes thrust outside [T_min, T_max]
                     */
                    violaThrust =
                        (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;
                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    totalGradPos.setZero(), totalGradVel.setZero(),
                        totalGradAcc.setZero(), totalGradJer.setZero();
                    // all penalty terms
                    pena = 0.0;

                    if (smoothedL1(violaVel, smoothFactor, violaVelPena,
                                   violaVelPenaD))
                    {
                        // cost type is v^2 - v_max^2
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                        totalGradVel += gradVel;
                    }
                    if (smoothedL1(violaAcc, smoothFactor, violaAccPena,
                                   violaAccPenaD))
                    {
                        // cost type is v^2 - v_max^2
                        gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                        pena += weightAcc * violaVelPena;
                        totalGradAcc += gradAcc;
                    }

                    double violaOmgX     = omg(0) * omg(0) - omgxSqrMax;
                    double violaOmgY     = omg(1) * omg(1) - omgySqrMax;
                    double violaOmgZ     = omg(2) * omg(2) - omgzSqrMax;
                    double violaOmgXPena = 0.0, violaOmgYPena = 0.0,
                           violaOmgZPena  = 0.0;
                    double violaOmgXPenaD = 0.0, violaOmgYPenaD = 0.0,
                           violaOmgZPenaD = 0.0;
                    double gradOmgx = 0, gradOmgy = 0, gradOmgz = 0;
                    double omg_x = omg.x(), omg_y = omg.y(), omg_z = omg.z();
                    smoothedL1(violaOmgX, smoothFactor, violaOmgXPena,
                               violaOmgXPenaD);
                    smoothedL1(violaOmgY, smoothFactor, violaOmgYPena,
                               violaOmgYPenaD);
                    smoothedL1(violaOmgZ, smoothFactor, violaOmgZPena,
                               violaOmgZPenaD);
                    {
                        gradOmgx +=
                            config.omg_x_weight * violaOmgXPenaD * 2.0 * omg_x;
                        gradOmgy +=
                            config.omg_y_weight * violaOmgYPenaD * 2.0 * omg_y;
                        gradOmgz +=
                            config.omg_z_weight * violaOmgZPenaD * 2.0 * omg_z;
                        gradOmg = Eigen::Vector3d(gradOmgx, gradOmgy, gradOmgz);
                        pena += config.omg_x_weight * violaOmgXPena;
                        pena += config.omg_y_weight * violaOmgYPena;
                        pena += config.omg_z_weight * violaOmgZPena;

                        Eigen::Matrix3d Jv, Ja, Jj;
                        tailsitter_df::Data::getInstance().jacobian_omega(
                            Jv, Ja, Jj);
                        totalGradVel += Jv * gradOmg;
                        totalGradAcc += Ja * gradOmg;
                        totalGradJer += Jj * gradOmg;
                    }

                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena,
                                   violaThrustPenaD))
                    {
                        gradThr += weightThrust * violaThrustPenaD * 2.0 *
                                   (thr - thrustMean);
                        pena += weightThrust * violaThrustPena;

                        Eigen::Vector3d tmp_gv, tmp_ga, tmp_gj;
                        tailsitter_df::Data::getInstance().jacobian_at(
                            tmp_gv, tmp_ga, tmp_gj);
                        totalGradVel += tmp_gv * gradThr;
                        totalGradAcc += tmp_ga * gradThr;
                        totalGradJer += tmp_gj * gradThr;
                    }

                    node  = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    gradC.block<6, 3>(i * 6, 0) +=
                        (beta0 * totalGradPos.transpose() +
                         beta1 * totalGradVel.transpose() +
                         beta2 * totalGradAcc.transpose() +
                         beta3 * totalGradJer.transpose()) *
                        node * step;
                    gradT(i) +=
                        (totalGradPos.dot(vel) + totalGradVel.dot(acc) +
                         totalGradAcc.dot(jer) + totalGradJer.dot(sna)) *
                            alpha * node * step +
                        node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr, const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC              &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int                         dimTau  = obj.temporalDim;
            const int                         dimXi   = obj.spatialDim;
            const double                      weightT = obj.config.time_weight;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd>       gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd>       gradXi(g.data() + dimTau, dimXi);

            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);
            // TODO actually, this is not correct.
            tailsitter_df::Data::getInstance().reset();

            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.smoothEps, obj.integralRes, obj.config,
                                    cost, obj.partialGradByTimes,
                                    obj.partialGradByCoeffs);

            obj.minco.propogateGrad(obj.partialGradByCoeffs,
                                    obj.partialGradByTimes, obj.gradByPoints,
                                    obj.gradByTimes);

            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints,
                          gradXi);
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        static inline double costDistance(void *ptr, const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void                 **dataPtrs = (void **)ptr;
            const double          &dEps     = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini =
                *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin =
                *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double    cost     = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d  a, b, d;
            Eigen::VectorXd  r;
            double           smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d                = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double          sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd>       gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ    = q * invNormQ;
                gradQ.head(k - 1) =
                    (vPolys[2 * i + 1].rightCols(k - 1).transpose() *
                     gradP.col(i))
                        .array() *
                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c  = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV      &vPolys,
                                           const double          &smoothD,
                                           Eigen::Matrix3Xd      &path)
        {
            const int       overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void  *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past      = 3;
            shortest_path_params.delta     = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi, minDistance,
                                  &GCOPTER_PolytopeSFC::costDistance, nullptr,
                                  nullptr, dataPtrs, shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>()  = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) =
                    vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                    vPolys[2 * i + 1].col(0);
            }

            return;
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV       &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int         nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    std::cerr << i << " " << hPs[i] << std::endl;
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) =
                    curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows())        = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) =
                    curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) =
                curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double           &speed,
                                      const Eigen::VectorXi  &intervalNs,
                                      Eigen::Matrix3Xd       &innerPoints,
                                      Eigen::VectorXd        &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            std::cerr << "sizeM" << sizeM << std::endl;
            std::cerr << "sizeN" << sizeN << std::endl;

            innerPoints.resize(3, sizeN - 1);
            timeAlloc.resize(sizeN);
            std::cerr << "intervalNs " << intervalNs.transpose() << "\n";
            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

       public:
        inline bool setup_basic_trajectory(
            const Eigen::Matrix3d  &initialPVA,
            const Eigen::Matrix3d  &terminalPVA,
            const Eigen::VectorXd  &initialTime,
            const Eigen::Matrix3Xd &initialPoints,
            const PolyhedraH &sfc_control_points, const double smoothingFactor,
            const int integralResolution)
        {
            headPVA           = initialPVA;
            tailPVA           = terminalPVA;
            smoothEps         = smoothingFactor;
            integralRes       = integralResolution;
            int size_corridor = sfc_control_points.size();
            pieceN            = size_corridor + 1;
            temporalDim       = pieceN;

            minco.setConditions(headPVA, tailPVA, pieceN);

            // Allocate temp variables
            points.resize(3, pieceN - 1);

            hPolytopes = sfc_control_points;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }

            vPolytopes.reserve(size_corridor);
            PolyhedronV curV, cur_v0_and_VB;
            PolyhedronH curH;
            spatialDim = 0;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                if (!geo_utils::enumerateVs(hPolytopes[i], curV))
                {
                    std::cerr << "not enumerateVs" << std::endl;
                    return false;
                }
                int nv = curV.cols();
                spatialDim += nv;
                cur_v0_and_VB.resize(3, nv);
                cur_v0_and_VB.col(0) = curV.col(0);
                cur_v0_and_VB.rightCols(nv - 1) =
                    curV.rightCols(nv - 1).colwise() - curV.col(0);
                vPolytopes.push_back(cur_v0_and_VB);
            }
            vPolyIdx.resize(size_corridor);
            // if we don't constrain the pos error, then the hPolyIdx is useless
            // hPolyIdx.resize(size_corridor);
            for (int i = 0; i < size_corridor; i++)
            {
                // hPolyIdx(i) = i;
                vPolyIdx(i) = i;
            }

            // allocate memory
            minco.setConditions(headPVA, tailPVA, pieceN);
            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(6 * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            times  = initialTime;
            points = initialPoints;

            return true;
        }

        inline double optimize(Trajectory<5> &traj, const double &relCostTol)
        {
            Eigen::VectorXd             x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            // print points and times
            std::cerr << "points before\n" << points << std::endl;
            std::cerr << "times before\n" << times.transpose() << std::endl;
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size       = 256;
            lbfgs_params.past           = 3;
            lbfgs_params.min_step       = 1.0e-20;
            lbfgs_params.max_linesearch = 256;
            lbfgs_params.g_epsilon      = 0.0;
            lbfgs_params.delta          = relCostTol;
            lbfgs_params.s_curv_coeff   = 0.999999;

            int ret = lbfgs::lbfgs_optimize(
                x, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional,
                nullptr, nullptr, this, lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
                // print after
                std::cerr << "points after\n" << points << std::endl;
                std::cerr << "times after\n" << times.transpose() << std::endl;
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cerr << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret) << std::endl;
            }

            return minCostFunctional;
        }
    };

}  // namespace gcopter

#endif
