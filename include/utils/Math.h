#ifndef DWL_Math_H
#define DWL_Math_H

#include <Eigen/Dense>
#include <vector>


namespace dwl
{

namespace utils
{


class Math
{
	public:
		Math() {};
		~Math() {};

		unsigned int computeMeanAndCovarianceMatrix(std::vector<Eigen::Vector3f> cloud,
															  Eigen::Matrix3d &covariance_matrix, Eigen::Vector3d &mean);
		void solvePlaneParameters(const Eigen::Matrix3d &covariance_matrix,	Eigen::Vector3d &normal_vector, double &curvature);
		void computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots);

	private:
		void computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots);
};

} //@namespace utils

} //@namespace dwl


#endif
