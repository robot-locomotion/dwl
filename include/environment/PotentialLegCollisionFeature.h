#ifndef DWL_PotentialLegCollisionFeature_H
#define DWL_PotentialLegCollisionFeature_H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class PotentialLegCollisionFeature
 * @brief Class for computing the reward value of the leg potential collision feature
 */
class PotentialLegCollisionFeature : public Feature
{
	public:
		/** @brief Constructor function */
		PotentialLegCollisionFeature();

		/** @brief Destructor function */
		~PotentialLegCollisionFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::RobotAndTerrain info Information of the robot and terrain
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
