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
		BodyOrientationFeature(double max_roll, double max_pitch);

		/** @brief Destructor function */
		~BodyOrientationFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::RobotAndTerrain info Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Threshold that specifies the flat orientation condition */
		double flat_orientation_;

		/** @brief Threshold that indicates a very bad roll condition */
		double max_roll_;

		/** @brief Threshold that indicates a very bad pitch condition */
		double max_pitch_;
};

} //@namespace environment
} //@namespace dwl

#endif
