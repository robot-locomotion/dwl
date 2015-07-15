#ifndef DWL__ENVIRONMENT__HEIGHT_DEVIATION_FEATURE__H
#define DWL__ENVIRONMENT__HEIGHT_DEVIATION_FEATURE__H

#include <environment/Feature.h>


namespace dwl
{

namespace environment
{

/**
 * @class HeightDeviationFeature
 * @brief Class for solving the reward value of a height deviation feature
 */
class HeightDeviationFeature : public Feature
{
	public:
		/** @brief Constructor function */
		HeightDeviationFeature(double flat_height_deviation, double max_height_deviation,
				double min_allowed_height = -std::numeric_limits<double>::max());

		/** @brief Destructor function */
		~HeightDeviationFeature();

		/**
		 * @brief Compute the reward value given a terrain information
		 * @param double& Reward value
		 * @param Terrain Information of the terrain
		 */
		void computeReward(double& reward_value, Terrain terrain_info);


	private:
		/** @brief Flat height deviation */
		double flat_height_deviation_;

		/** @brief Maximum height deviation */
		double max_height_deviation_;

		/** @brief Minimum allowed height */
		double min_allowed_height_;
};

} //@namespace environment
} //@namespace dwl

#endif
