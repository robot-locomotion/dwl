#ifndef DWL_RewardOctoMap_H
#define DWL_RewardOctoMap_H

#include <environment/RewardMap.h>
#include <octomap/octomap.h>
#include <utils/Math.h>
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

		void compute(Modeler model, Eigen::Vector2d robot_position);
		bool computeFeaturesAndRewards(octomap::OcTree* octomap, octomap::OcTreeKey heightmap_key);
		bool computeRewards(std::vector<Eigen::Vector3f> cloud);

		std::vector<Pose> getNormals(); //TODO

	private:
		std::vector<CellKey> occupied_voxels_;
		dwl::utils::Math math_;
		bool is_first_computation_;
		bool using_cloud_mean_;

		std::vector<Pose> normals_; //TODO

};


} //@namespace environment

} //@namespace dwl


#endif
