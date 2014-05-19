#ifndef DWL_SlopeFeature_H
#define DWL_SlopeFeature_H

#include <environment/Feature.h>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{

/**
 * @class SlopeFeature
 * @brief Class for computing the reward value of the slope feature
 */
class SlopeFeature : public Feature
{
	public:
		/** @brief Constructor function */
		SlopeFeature();

		/** @brief Destructor function */
		~SlopeFeature();

		/**
		 * @brief Compute the reward value given a terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::environment::Terrain terrain_info Information of the terrain
		 */
		void computeReward(double& reward_value, Terrain terrain_info);

	private:
		/** @brief Threshold that specify the flat condition */
		double flat_threshold_;

		/** @brief Threshold that indicates a very (bad) steep condition */
		double steep_threshold_;
};


} //@namespace environment

} //@namespace dwl

#endif
