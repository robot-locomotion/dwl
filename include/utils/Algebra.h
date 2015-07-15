#ifndef DWL__MATH__ALGEBRA__H
#define DWL__MATH__ALGEBRA__H

#include <utils/macros.h>
#include <Eigen/Dense>
#include <vector>


namespace dwl
{

namespace math
{

//template <typename T, typename D>
//void dampedPseudoInverse(const Eigen::MatrixBase<T>& A, double damping_factor,
//						 Eigen::MatrixBase<D>& Apinv, unsigned int computation_options);

/**
 * @brief Computes the pseudo inverse using Moore Penrose algorithm
 * @param const Eigen::MatrixXd& Matrix
 * @param double Tolerance of the singular value decomposition
 * @return Returns the pseudo-inverse matrix
 */
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1E-9);

/**
 * @brief Computes the skew symmetric matrix from a 3d vector
 * @param Eigen::Vector3d 3d vector
 * @return Returns the skew symmetric matrix
 */
Eigen::Matrix3d skewSymmentricMatrixFrom3DVector(Eigen::Vector3d vector);

} //@namespace utils
} //@namespace dwl

#endif
