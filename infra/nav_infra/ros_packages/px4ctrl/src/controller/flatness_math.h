#ifndef __CONTROLLER_FLATNESS_MATH_H
#define __CONTROLLER_FLATNESS_MATH_H

#include <Eigen/Dense>

/**
 * @return True when |value| is below the singularity guard [1e-3] [-]
 */
bool almostZero(double value);

/**
 * Normalize a vector and differentiate the normalization (flatness map).
 *
 * @param[in] x Reference vector, must be non-zero [-]
 * @param[in] xd Time derivative of x [-/s]
 * @param[out] xNor Normalized vector [-]
 * @param[out] xNord Time derivative of the normalized vector [-/s]
 */
void normalizeWithGrad(const Eigen::Vector3d &x, const Eigen::Vector3d &xd,
                       Eigen::Vector3d &xNor, Eigen::Vector3d &xNord);

/**
 * Body-x axis from the yaw-prototype vector, robust to the y_C/z_B
 * collinear singularity by projecting the estimated body-x axis.
 *
 * @param[in] x_B_prototype y_C cross z_B reference [m/s^2]
 * @param[in] x_C Heading-frame x axis [-]
 * @param[in] y_C Heading-frame y axis [-]
 * @param[in] est_q Estimated attitude (world from body) [-]
 * @return Unit body-x axis in world [-]
 */
Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d    &x_B_prototype,
                                       const Eigen::Vector3d    &x_C,
                                       const Eigen::Vector3d    &y_C,
                                       const Eigen::Quaterniond &est_q);

#endif
