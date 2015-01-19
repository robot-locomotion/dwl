#ifndef DWL_SupportTriangleFeature_H
#define DWL_SupportTriangleFeature_H

#include <environment/Feature.h>
#include <utils/Math.h>


namespace dwl
{

namespace environment
{

/**
 * @class SupportTriangleFeature
 * @brief Class for computing the reward value of the support triangle feature
 */
class SupportTriangleFeature : public Feature
{
	public:
		/** @brief Constructor function */
		SupportTriangleFeature(double stable_inradii, double unstable_inradii);

		/** @brief Destructor function */
		~SupportTriangleFeature();

		/**
		 * @brief Compute the reward value given a terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::RobotAndTerrain info Information of the robot and terrain
		 */
		void computeReward(double& reward_value, RobotAndTerrain info);

	private:
		/** @brief Stable support triangle area */
		double stable_inradii_;

		/** @brief Unstable support triangle area */
		double unstable_inradii_;
};

} //@namespace environment
} //@namespace dwl

#endif
