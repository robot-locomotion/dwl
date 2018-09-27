#ifndef DWL__ENVIRONEMT__FEATURE__H
#define DWL__ENVIRONEMT__FEATURE__H

#include <dwl/environment/SpaceDiscretization.h>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class Feature
 * @brief Abstract class for solving the cost value of a predefined terrain feature
 */
class Feature
{
	public:
		/** @brief Constructor function **/
		Feature();

		/** @brief Destructor function **/
		virtual ~Feature();

		/**
		 * @brief Abstract method to compute the cost value according some
		 * terrain information
		 * @param double& Reference of the cost variable
		 * @param const Terrain& Information about the terrain, i.e. position,
		 * surface and curvature
		 */
		virtual void computeCost(double& cost_value,
								 const Terrain& terrain_info);

		/**
		 * @brief Abstract method to compute the cost value according some robot
		 * and terrain information
		 * @param double& Reference of the reward variable
		 * @param const RobotAndTerrain& Information of the robot and terrain
		 */
		virtual void computeCost(double& cost_value,
								 const RobotAndTerrain& info);

		/**
		 * @brief Sets the weight of the feature
		 * @param double Weight of the feature
		 */
		void setWeight(double weight);

		/**
		 * @brief Gets the weight of the feature
		 * @param double& Weight of the feature
		 */
		void getWeight(double& weight);

		/**
		 * @brief Sets the neighboring area for computing some feature
		 * @param double Minimum x
		 * @param double Maximum x
		 * @param double Minimum y
		 * @param double Maximum y
		 * @param double resolution Resolution of the neighboring area
		 */
		void setNeighboringArea(double min_x, double max_x,
								double min_y, double max_y,
								double resolution);

		/**
		 * @brief Gets the name of the feature
		 * @return std::string Return the name of the feature
		 */
		std::string getName();

	protected:
		/** @brief Name of the feature **/
		std::string name_;

		/** @brief Object of the SpaceDiscretization class for defining the space discretization routines */
		SpaceDiscretization space_discretization_;

		/** @brief Maximum cost */
		double max_cost_;

		/** @brief Weight used for computing the total reward */
		double weight_;

		/** @brief Area for computing the average of the height map */
		SearchArea neightboring_area_;
};

} //@namespace environment
} //@namespace dwl

#endif
