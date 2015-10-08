#ifndef DWL__ENVIRONMENT__STANCE_POSTURE_FEATURE__H
#define DWL__ENVIRONMENT__STANCE_POSTURE_FEATURE__H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class StancePostureFeature
 * @brief Class for computing the reward value of stance posture feature
 */
class StancePostureFeature : public Feature
{
	public:
		/** @brief Constructor function */
		StancePostureFeature();

		/** @brief Destructor function */
		~StancePostureFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& Reward value
		 * @param RobotAndTerrain Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);


	private:
		double max_distance_;
		bool first_time_;
};

} //@namespace environment
} //@namespace dwl

#endif
