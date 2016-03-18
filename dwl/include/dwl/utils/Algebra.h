#ifndef DWL__MATH__ALGEBRA__H
#define DWL__MATH__ALGEBRA__H

#include <dwl/utils/macros.h>
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
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix,
							  double tolerance = 1E-9);

/**
 * @brief Computes the skew symmetric matrix from a 3d vector
 * @param Eigen::Vector3d 3d vector
 * @return Returns the skew symmetric matrix
 */
Eigen::Matrix3d skewSymmentricMatrixFrom3DVector(Eigen::Vector3d vector);

/**
 * @brief Solve a linear system of equations (Ax = b) by applying the Gaussian
 * elimination method
 * @param Eigen::VectorXd& Solution of the system
 * @param Eigen::MatrixXd& A matrix
 * @param Eigen::VectorXd& b column-vector
 */
void GaussianEliminationPivot(Eigen::VectorXd& x,
							  Eigen::MatrixXd& A,
							  Eigen::VectorXd& b);

} //@namespace math
} //@namespace dwl

#endif
