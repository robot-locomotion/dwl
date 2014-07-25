#ifndef DWL_HeightDeviationFeature_H
#define DWL_HeightDeviationFeature_H

#include <environment/Feature.h>
#include <environment/SpaceDiscretization.h>


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
		HeightDeviationFeature();

		/** @brief Destructor function */
		~HeightDeviationFeature();

		/**
		 * @brief Compute the reward value given a terrain information
		 * @param double& reward_value Reward value
		 * @param dwl::Terrain terrain_info Information of the terrain
		 */
		void computeReward(double& reward_value, Terrain terrain_info);


	private:
		/** @brief Area for computing the average of the height map */
		SearchArea average_area_;

		/** @brief Object of the SpaceDiscretization class for defining the space discretiization routines */
		SpaceDiscretization space_discretization_;

};

} //@namespace environment
} //@namespace dwl

#endif
