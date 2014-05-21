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
		 * @brief Implemented method for computing the reward map according the robot position and octomap model of the terrain
		 * @param dwl::environment::Modeler model Define the model of the environment
		 * @param Eigen::Vector2d robot_position Define the position of the robot
		 */
		void compute(Modeler model, Eigen::Vector3d robot_position);

		/**
		 * @brief Compute the features and reward of the terrain given the octomap model and the key of the topmost cell of a certain position of the grid
		 * @param octomap::OcTree* octomap Pointer to the octomap model of the environment
		 * @param octomap::OcTreeKey heighmap_key The key of the topmost cell of a certain position of the grid
		 * return bool Indicate if it could be computed the features and reward of the cell
		 */
		bool computeFeaturesAndRewards(octomap::OcTree* octomap, octomap::OcTreeKey heightmap_key);

		/**
		 * @brief Compute the rewards of point cloud of neighboring og the cell
		 */
		bool computeRewards(std::vector<Eigen::Vector3f> cloud);


		std::vector<Pose> getNormals(); //TODO

	private:
		/** @brief Vector of occupied cell's key */
		std::vector<CellKey> occupied_cells_;

		/** @brief Object of the Math class for the definition of math routines */
		dwl::utils::Math math_;

		/** @brief Indicate if it the first computation */
		bool is_first_computation_;

		/** @brief Define if it is using the mean of the cloud */
		bool using_cloud_mean_;

		std::vector<Pose> normals_; //TODO

};


} //@namespace environment

} //@namespace dwl


#endif
