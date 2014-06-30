#ifndef DWL_Feature_H
#define DWL_Feature_H

#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class Feature
 * @brief Abstract class for solving the reward value of a predefined terrain feature
 */
class Feature
{
	public:
		/** @brief Constructor function **/
		Feature();

		/** @brief Destructor function **/
		virtual ~Feature();

		/**
		 * @brief Abstract method to compute reward value according some terrain information
		 * @param double& reward_value Reference of the reward variable
		 * @param dwl::environment::Terrain terrain_info Information about the terrain, i.e. position, surface and curvature
		 */
		virtual void computeReward(double& reward_value, Terrain terrain_info) = 0;

		/**
		 * @brief Gets the name of the feature
		 * @return std::string Return the name of the feature
		 */
		std::string getName();


	protected:
		/** @brief Name of the feature **/
		std::string name_;

}; //@class Feature


} //@namespace environment
} //@namespace dwl

#endif
