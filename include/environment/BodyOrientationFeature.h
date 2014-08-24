#ifndef DWL_BodyOrientationFeature_H
#define DWL_BodyOrientationFeature_H

#include <environment/Feature.h>
#include <utils/utils.h>
#include <utils/Math.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace environment
{

/**
 * @class BodyOrientationFeature
 * @brief Class for computing the reward value of the certain body orientation feature
 */
class BodyOrientationFeature : public Feature
{
	public:
		/** @brief Constructor function */
		BodyOrientationFeature();

		/** @brief Destructor function */
		~BodyOrientationFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::RobotAndTerrain info Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Threshold that specifies the flat condition */
		double flat_threshold_;

		/** @brief Threshold that indicates a very bad roll condition */
		double roll_threshold_;

		/** @brief Threshold that indicates a very bad pitch condition */
		double pitch_threshold_;
};

} //@namespace environment
} //@namespace dwl

#endif
