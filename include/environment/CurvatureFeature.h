#ifndef DWL_CurvatureFeature_H
#define DWL_CurvatureFeature_H

#include <environment/Feature.h>

namespace dwl
{

namespace environment
{

/**
 * @class CurvatureFeature
 * @brief Class for computing the reward value of the curvature feature
 */
class CurvatureFeature : public Feature
{
	public:
		/** @brief Constructor function */
		CurvatureFeature();

		/** @brief Destructor function */
		~CurvatureFeature();

		/**
		 * @brief Compute the reward value given a terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::Terrain terrain_info Information of the terrain
		 */
		void computeReward(double& reward_value, Terrain terrain_info);

	private:
		/** @brief Threshold that specify the positive condition */
		double positive_threshold_;

		/** @brief Threshold that indicates a very (bad) condition */
		double negative_threshold_;
};

} //@namespace environment
} //@namespace dwl

#endif
