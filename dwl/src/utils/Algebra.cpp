#include <dwl/utils/Algebra.h>


namespace dwl
{

namespace math
{
//template <typename T, typename D>
//void dampedPseudoInverse(const Eigen::MatrixBase<T>& A, double damping_factor,
//						 Eigen::MatrixBase<D>& Apinv, unsigned int computation_options)//TODO Change the arguments
//{
//	using namespace Eigen;
//
//	int m = A.rows(), n = A.cols(), k = m < n ? m : n;
//	JacobiSVD<typename MatrixBase<T>::PlainObject> svd = A.jacobiSvd(computation_options);
//	const typename JacobiSVD<typename T::PlainObject>::SingularValuesType& singular_values = svd.singularValues();
//	MatrixXd sigma_damped = MatrixXd::Zero(k, k);
//
//	double damp = damping_factor * damping_factor;
//	for (int idx = 0; idx < k; idx++) {
//		sigma_damped(idx, idx) = singular_values(idx) / (singular_values(idx) * singular_values(idx) + damp);
//	}
//	Apinv = svd.matrixV() * sigma_damped * svd.matrixU().transpose(); // damped pseudoinverse
//}


Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A, double tolerance)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd pinvA(A.cols(), A.rows());

	svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Build a diagonal matrix with the Inverted Singular values
	// The pseudo inverted singular matrix is easy to compute: is formed by replacing every nonzero
	// entry by its reciprocal (inverting).
	Eigen::Matrix<double, Eigen::Dynamic, 1> singular_values(svd.matrixV().cols(),1);

	singular_values = (svd.singularValues().array()>tolerance).select(svd.singularValues().array().inverse(), 0);

	// Pseudo-Inversion : V * S * U'
	pinvA = (svd.matrixV() * singular_values.asDiagonal() * svd.matrixU().transpose());
	return pinvA;
}


Eigen::Matrix3d skewSymmentricMatrixFrom3DVector(Eigen::Vector3d vector)
{
	Eigen::Matrix3d skew_symmetric_matrix;
	skew_symmetric_matrix.setZero();
	// S = [    0, -w(3),  w(2);
	//       w(3),     0, -w(1);
	//      -w(2),  w(1),   0 ];
	skew_symmetric_matrix(0, 1) = -vector(2);
	skew_symmetric_matrix(0, 2) = vector(1);
	skew_symmetric_matrix(1, 2) = -vector(0);
	skew_symmetric_matrix.bottomLeftCorner<2, 2>() = -skew_symmetric_matrix.topRightCorner<2, 2>().transpose();

	return skew_symmetric_matrix;
}


void GaussianEliminationPivot(Eigen::VectorXd &x,
							  Eigen::MatrixXd& A,
							  Eigen::VectorXd& b)
{
	x = Eigen::VectorXd::Zero(A.cols());

	// We can only solve quadratic systems
	assert (A.rows() == A.cols());

	unsigned int n = A.rows();
	unsigned int pi;

	// the pivots
	size_t *pivot = new size_t[n];

	// temporary result vector which contains the pivoted result
	Eigen::VectorXd px(x);

	unsigned int i,j,k;

	for (i = 0; i < n; i++)
		pivot[i] = i;

	for (j = 0; j < n; j++) {
		pi = j;
		double pv = fabs(A(j, pivot[j]));

		// find the pivot
		for (k = j; k < n; k++) {
			double pt = fabs(A(j, pivot[k]));
			if (pt > pv) {
				pv = pt;
				pi = k;
				unsigned int p_swap = pivot[j];
				pivot[j] = pivot[pi];
				pivot[pi] = p_swap;
			}
		}

		for (i = j + 1; i < n; i++) {
			if (fabs(A(j, pivot[j])) <= std::numeric_limits<double>::epsilon()) {
				std::cerr << "Error: pivoting failed for matrix A = " << std::endl;
				std::cerr << "A = " << std::endl << A << std::endl;
				std::cerr << "b = " << b << std::endl;
			}
			double d = A(i,pivot[j])/A(j,pivot[j]);

			b[i] -= b[j] * d;

			for (k = j; k < n; k++) {
				A(i,pivot[k]) -= A(j,pivot[k]) * d;
			}
		}
	}

	// warning: i is an unsigned int, therefore a for loop of the
	// form "for (i = n - 1; i >= 0; i--)" might end up in getting an invalid
	// value for i!
	i = n;
	do {
		i--;

		for (j = i + 1; j < n; j++) {
			px[i] += A(i,pivot[j]) * px[j];
		}
		px[i] = (b[i] - px[i]) / A(i,pivot[i]);

	} while (i > 0);

	// Unswapping
	for (i = 0; i < n; i++) {
		x[pivot[i]] = px[i];
	}

	delete[] pivot;
}

} //@namespace math
} //@namespace dwl
