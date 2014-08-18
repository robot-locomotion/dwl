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
		 * @brief Sets the weight of the feature
		 * @param double weight Weight of the feature
		 */
		void setWeight(double weight);

		/**
		 * @brief Gets the weight of the feature
		 * @param double& weight Weight of the feature
		 */
		void getWeight(double& weight);

		/**
		 * @brief Sets the neighboring area for computing some feature
		 * @param double min_x Minimun x
		 * @param double max_x Maximun x
		 * @param double min_y Minimun y
		 * @param double max_y Maximun y
		 * @param double resolution Resolution of the neighboring area
		 */
		void setNeighboringArea(double min_x, double max_x, double min_y, double max_y, double resolution);

		/**
		 * @brief Sets the gain that maps the feature to reward value
		 * @param double gain Value of the gain
		 */
		void setGainFeature(double gain);

		/**
		 * @brief Gets the name of the feature
		 * @return std::string Return the name of the feature
		 */
		std::string getName();


	protected:
		/** @brief Name of the feature **/
		std::string name_;

		/** @brief Weight used for computing the total reward */
		double weight_;

		/** @brief Area for computing the average of the height map */
		SearchArea neightboring_area_;

		/** @brief Gain that maps the feature to reward value */
		double gain_;
};

} //@namespace environment
} //@namespace dwl

#endif
