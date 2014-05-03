#include <environment/RewardMap.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{


struct SearchArea
{
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double grid_size;
};

class RewardOctoMap : public RewardMap
{
	public:
		RewardOctoMap();
		~RewardOctoMap();

		void compute();
		bool computeRewards(std::vector<octomap::point3d> cloud, Eigen::Vector3d &normal_vector, float &curvature);
		unsigned int computeMeanAndCovarianceMatrix(std::vector<octomap::point3d> cloud,
														  Eigen::Matrix3d &covariance_matrix, Eigen::Vector4d &mean);
		void solvePlaneParameters(const Eigen::Matrix3d &covariance_matrix,	Eigen::Vector3d &normal_vector, float &curvature);
		void computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots);
		void computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots);

	private:
		octomap::OcTree* octomap_;
		SearchArea search_area_;

};


} //@namespace environment

} //@namespace dwl
