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

/**
 * @class RewardOctoMap
 * @brief Class for computing the reward map of the terrain using octomap framework
 */
class RewardOctoMap : public RewardMap
{
	public:
		/** @brief Constructor function */
		RewardOctoMap();

		/** @brief Destructor function */
		~RewardOctoMap();

		/**
		 * @brief Abstract method for computing the reward map according the robot position and octomap model
		 * of the terrain
		 * @param TerrainModel The model of the environment
		 * @param Eigen::Vector4d The position of the robot and the yaw angle
		 */
		void compute(TerrainModel model, Eigen::Vector4d robot_state);

		/**
		 * @brief Computes the features and reward of the terrain given the octomap model and the key
		 * of the topmost cell of a certain position of the grid
		 * @param octomap::OcTree* Pointer to the octomap model of the environment
		 * @param octomap::OcTreeKey The key of the topmost cell of a certain position of the grid
		 */
		void computeRewards(octomap::OcTree* octomap, octomap::OcTreeKey heightmap_key);


	private:
		/** @brief Object of the Math class for the definition of math routines */
		dwl::utils::Math math_;

		/** @brief Indicates if it the first computation */
		bool is_first_computation_;

		/** @brief Defines if it is using the mean of the cloud */
		bool using_cloud_mean_;

		/** @brief Depth of the octomap */
		int depth_;
};

} //@namespace environment
} //@namespace dwl


#endif
