#ifndef DWL_MaximumHeightFeature_H
#define DWL_MaximumHeightFeature_H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class MaximumHeightFeature
 * @brief Class for computing the reward value of the maximun height feature
 */
class MaximumHeightFeature : public Feature
{
	public:
		/** @brief Constructor function */
		MaximumHeightFeature();

		/** @brief Destructor function */
		~MaximumHeightFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::RobotAndTerrain info Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:

};

} //@namespace environment
} //@namespace dwl

#endif
