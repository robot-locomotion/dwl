#include <utils/Math.h>


namespace dwl
{

namespace math
{

void normalizeAngle(double& angle, AngleRepresentation angle_notation)
{
	switch (angle_notation) {
		case ZeroTo2Pi:
			// Normalizing the angle between [0,2*pi]
			if ((angle >= (2 * M_PI)) || (angle <= 0))
				angle -= floor(angle / (2 * M_PI)) * (2 * M_PI);
			break;

		case MinusPiToPi:
			// Normalizing the angle between [0,2*pi]
			if ((angle >= (2 * M_PI)) || (angle <= 0))
				angle -= floor(angle / (2 * M_PI)) * (2 * M_PI);

			// Normalizing the angle between [0,pi]
			if ((angle > M_PI) && (angle < (2 * M_PI))) {
				angle -= 2 * M_PI;
			}
			break;

		default:
			printf(YELLOW "Warning: it was not normalize the angle because the angle notation is "
					"incoherent \n" COLOR_RESET);
			break;
	}
}


void inRadiiTriangle(double& inradii, double size_a, double size_b, double size_c)
{
	double s = (size_a + size_b + size_c) / 2;
	double a = (s - size_a) * (s - size_b) * (s - size_c) / s;

	if (a > 0)
		inradii = sqrt(a);
	else
		inradii = 0;
}


void computePlaneParameters(Eigen::Vector3d& normal, std::vector<Eigen::Vector3f> points)
{
	if (points.size() <= 3)
		printf(YELLOW "Warning: could not computed the plane parameter with less of 4 points\n" COLOR_RESET);
	else {
		Eigen::Matrix3d covariance;
		Eigen::Vector3d mean;
		double curvature;

		computeMeanAndCovarianceMatrix(points, covariance, mean);
		solvePlaneParameters(normal, curvature, covariance);
	}
}


unsigned int computeMeanAndCovarianceMatrix(std::vector<Eigen::Vector3f> cloud,
											Eigen::Matrix3d &covariance_matrix, Eigen::Vector3d &mean)
{
	// create the buffer on the stack which is much faster than using cloud[indices[i]] and mean as a buffer
	Eigen::VectorXd accu = Eigen::VectorXd::Zero(9, 1);

	for (size_t i = 0; i < cloud.size(); ++i) {
		accu[0] += cloud[i](0) * cloud[i](0);
		accu[1] += cloud[i](0) * cloud[i](1);
		accu[2] += cloud[i](0) * cloud[i](2);
		accu[3] += cloud[i](1) * cloud[i](1);
		accu[4] += cloud[i](1) * cloud[i](2);
		accu[5] += cloud[i](2) * cloud[i](2);
		accu[6] += cloud[i](0);
		accu[7] += cloud[i](1);
		accu[8] += cloud[i](2);
	}

	accu /= (cloud.size());
	if (cloud.size() != 0) {
		mean[0] = accu[6];
		mean[1] = accu[7];
		mean[2] = accu[8];

		covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
		covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
		covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
		covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
		covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
		covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
		covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
		covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
		covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
	}

	return (static_cast<unsigned int> (cloud.size()));
}


void solvePlaneParameters(Eigen::Vector3d &normal_vector, double &curvature, const Eigen::Matrix3d covariance_matrix)
{
	// Extract the smallest eigenvalue and its eigenvector
	EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigenvalue;
	EIGEN_ALIGN16 Eigen::Vector3d eigenvector;

	//typedef typename Eigen::Matrix3f::Scalar Scalar;
	// Scale the matrix so its entries are in [-1,1]. The scaling is applied
	// only when at least one matrix entry has magnitude larger than 1.
	Eigen::Matrix3d::Scalar scale = covariance_matrix.cwiseAbs().maxCoeff();
	if (scale <= std::numeric_limits<Eigen::Matrix3d::Scalar>::min())
		scale = Eigen::Matrix3d::Scalar(1.0);

	Eigen::Matrix3d scaledMat = covariance_matrix / scale;

	Eigen::Vector3d eigenvalues;
	computeRoots(scaledMat, eigenvalues);

	eigenvalue = eigenvalues(0) * scale;

	scaledMat.diagonal().array() -= eigenvalues(0);

	Eigen::Vector3d vec1 = scaledMat.row(0).cross(scaledMat.row(1));
	Eigen::Vector3d vec2 = scaledMat.row(0).cross(scaledMat.row(2));
	Eigen::Vector3d vec3 = scaledMat.row(1).cross(scaledMat.row(2));

	Eigen::Matrix3d::Scalar len1 = vec1.squaredNorm();
	Eigen::Matrix3d::Scalar len2 = vec2.squaredNorm();
	Eigen::Matrix3d::Scalar len3 = vec3.squaredNorm();

	if (len1 >= len2 && len1 >= len3)
		eigenvector = vec1 / std::sqrt(len1);
	else if (len2 >= len1 && len2 >= len3)
		eigenvector = vec2 / std::sqrt(len2);
	else
		eigenvector = vec3 / std::sqrt(len3);


	normal_vector = eigenvector;

	// Compute the curvature surface change
	float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
	if (eig_sum != 0)
		curvature = fabsf(eigenvalue / eig_sum);
	else
		curvature = 0;
}


void computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots)
{
	// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0. The
	// eigenvalues are the roots to this equation, all guaranteed to be
	// real-valued, because the matrix is symmetric.
	Eigen::Matrix3d::Scalar c0 = m(0,0) * m(1,1) * m(2,2) +
			Eigen::Matrix3d::Scalar(2) * m(0,1) * m(0,2) * m(1,2)
	- m(0,0) * m(1,2) * m(1,2) - m(1,1) * m(0,2) * m(0,2)
	- m(2,2) * m(0,1) * m(0,1);

	Eigen::Matrix3d::Scalar c1 = m(0,0) * m(1,1) - m(0,1) * m(0,1) + m(0,0) * m(2,2) -
			m(0,2) * m(0,2) + m(1,1) * m(2,2) - m(1,2) * m(1,2);

	Eigen::Matrix3d::Scalar c2 = m(0,0) + m(1,1) + m(2,2);


	if (fabs (c0) < Eigen::NumTraits<Eigen::Matrix3d::Scalar>::epsilon ())// one root is 0 -> quadratic equation
		computeRoots2(c2, c1, roots);
	else
	{
		const Eigen::Matrix3d::Scalar s_inv3 = Eigen::Matrix3d::Scalar(1.0 / 3.0);
		const Eigen::Matrix3d::Scalar s_sqrt3 = std::sqrt(Eigen::Matrix3d::Scalar (3.0));
		// Construct the parameters used in classifying the roots of the equation
		// and in solving the equation for the roots in closed form.
		Eigen::Matrix3d::Scalar c2_over_3 = c2*s_inv3;
		Eigen::Matrix3d::Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
		if (a_over_3 > Eigen::Matrix3d::Scalar(0))
			a_over_3 = Eigen::Matrix3d::Scalar (0);

		Eigen::Matrix3d::Scalar half_b = Eigen::Matrix3d::Scalar(0.5) *
				(c0 + c2_over_3 * (Eigen::Matrix3d::Scalar(2) * c2_over_3 * c2_over_3 - c1));

		Eigen::Matrix3d::Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
		if (q > Eigen::Matrix3d::Scalar(0))
			q = Eigen::Matrix3d::Scalar(0);

		// Compute the eigenvalues by solving for the roots of the polynomial.
		Eigen::Matrix3d::Scalar rho = sqrt(-a_over_3);
		Eigen::Matrix3d::Scalar theta = atan2(std::sqrt(-q), half_b) * s_inv3;
		Eigen::Matrix3d::Scalar cos_theta = cos(theta);
		Eigen::Matrix3d::Scalar sin_theta = sin(theta);
		roots(0) = c2_over_3 + Eigen::Matrix3d::Scalar(2) * rho * cos_theta;
		roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
		roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		// Sort in increasing order.
		double roots0 = roots(0);
		double roots1 = roots(1);
		double roots2 = roots(2);
		if (roots(0) >= roots(1))
			std::swap(roots0, roots1);
		if (roots(1) >= roots(2)) {
			std::swap(roots1, roots2);
			if (roots(0) >= roots(1))
				std::swap(roots0, roots1);
		}
		roots(0) = roots0;
		roots(1) = roots1;
		roots(2) = roots2;

		if (roots(0) <= 0) // eigenvalue for symmetric positive semi-definite matrix can not be negative! Set it to 0
			computeRoots2(c2, c1, roots);
	}
}


void computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots)
{
	roots(0) = Eigen::Matrix3d::Scalar(0);
	Eigen::Matrix3d::Scalar d = Eigen::Matrix3d::Scalar(b * b - 4.0 * c);
	if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
		d = 0.0;

	Eigen::Matrix3d::Scalar sd = sqrt(d);

	roots(2) = 0.5f * (b + sd);
	roots(1) = 0.5f * (b - sd);
}

} //@namespace math
} //@namespace dwl
