#ifndef DWL_PotentialBodyOrientationFeature_H
#define DWL_PotentialBodyOrientationFeature_H

#include <environment/Feature.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class PotentialBodyOrientationFeature
 * @brief Class for computing the reward value of the potential body orientation feature
 */
class PotentialBodyOrientationFeature : public Feature
{
	public:
		/** @brief Constructor function */
		PotentialBodyOrientationFeature(double max_roll, double max_pitch);

		/** @brief Destructor function */
		~PotentialBodyOrientationFeature();

		/**
		 * @brief Compute the reward value given a robot and terrain information
		 * @param double& Reward value
		 * @param RobotAndTerrain Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Threshold that specifies the flat condition */
		double flat_orientation_;

		/** @brief Threshold that indicates a very bad roll condition */
		double max_roll_;

		/** @brief Threshold that indicates a very bad pitch condition */
		double max_pitch_;
};

} //@namespace environment
} //@namespace dwl

#endif
