#ifndef DWL_Feature_H
#define DWL_Feature_H

#include <environment/SpaceDiscretization.h>
#include <robot/Robot.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class Feature
 * @brief Abstract class for solving the reward value of a predefined terrain feature
 */
class Feature
{
	public:
		/** @brief Constructor function **/
		Feature();

		/** @brief Destructor function **/
		virtual ~Feature();

		/**
		 * @brief Sets the robot information that is used for body features
		 * @param dwl::robot::Robot* robot Robot properties
		 */
		void reset(robot::Robot* robot);

		/**
		 * @brief Abstract method to compute reward value according some terrain information
		 * @param double& Reference of the reward variable
		 * @param Terrain Information about the terrain, i.e. position, surface and curvature
		 */
		virtual void computeReward(double& reward_value, Terrain terrain_info);

		/**
		 * @brief Abstract method to compute reward value according some robot and terrain information
		 * @param double& Reference of the reward variable
		 * @param RobotAndTerrain Information of the robot and terrain
		 */
		virtual void computeReward(double& reward_value, RobotAndTerrain info);

		/**
		 * @brief Sets the weight of the feature
		 * @param double Weight of the feature
		 */
		void setWeight(double weight);

		/**
		 * @brief Gets the weight of the feature
		 * @param double& Weight of the feature
		 */
		void getWeight(double& weight);

		/**
		 * @brief Sets the neighboring area for computing some feature
		 * @param double Minimum x
		 * @param double Maximum x
		 * @param double Minimum y
		 * @param double Maximum y
		 * @param double resolution Resolution of the neighboring area
		 */
		void setNeighboringArea(double min_x, double max_x, double min_y, double max_y,
				double resolution);

		/**
		 * @brief Gets the name of the feature
		 * @return std::string Return the name of the feature
		 */
		std::string getName();

	protected:
		/** @brief Name of the feature **/
		std::string name_;

		/** @brief Object of the SpaceDiscretization class for defining the space discretization routines */
		SpaceDiscretization space_discretization_;

		/** @brief Pointer to the robot properties */
		robot::Robot* robot_;

		/** @brief Minimum reward */
		double min_reward_;

		/** @brief Weight used for computing the total reward */
		double weight_;

		/** @brief Area for computing the average of the height map */
		SearchArea neightboring_area_;
};

} //@namespace environment
} //@namespace dwl

#endif
