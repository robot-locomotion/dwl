#ifndef DWL__ENVIRONMENT__LEG_COLLISION_FEASURE__H
#define DWL__ENVIRONMENT__LEG_COLLISION_FEASURE__H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class LegCollisionFeature
 * @brief Class for computing the reward value of the leg collision feature
 */
class LegCollisionFeature : public Feature
{
	public:
		/** @brief Constructor function */
		LegCollisionFeature(double clearance, double collision);

		/** @brief Destructor function */
		~LegCollisionFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& Reward value
		 * @param RobotAndTerrain Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Potential clearance of environment */
		double potential_clearance_;

		/** @brief Potential collision with the environment */
		double potential_collision_;
};

} //@namespace environment
} //@namespace dwl

#endif
