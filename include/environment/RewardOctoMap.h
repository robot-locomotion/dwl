#ifndef DWL_RewardOctoMap_H
#define DWL_RewardOctoMap_H

#include <environment/RewardMap.h>
#include <octomap/octomap.h>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{


class RewardOctoMap : public RewardMap
{
	public:
		RewardOctoMap();
		~RewardOctoMap();

		void compute(Modeler model);
		bool computeRewards(std::vector<octomap::point3d> cloud, Eigen::Vector3d &normal_vector, float &curvature);

		unsigned int computeMeanAndCovarianceMatrix(std::vector<octomap::point3d> cloud,
														  Eigen::Matrix3d &covariance_matrix, Eigen::Vector3d &mean);
		void solvePlaneParameters(const Eigen::Matrix3d &covariance_matrix,	Eigen::Vector3d &normal_vector, float &curvature);
		void computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots);
		void computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots);

		std::vector<Pose> getNormals();

	private:
		std::vector<Pose> normals_;
};


} //@namespace environment

} //@namespace dwl


#endif
