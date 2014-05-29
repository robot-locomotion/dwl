#ifndef DWL_CostMap_H
#define DWL_CostMap_H

#include <planning/Cost.h>
#include <environment/PlaneGrid.h>


namespace dwl
{

namespace planning
{

/**
 * @class CostMAp
 * @brief Class for implementing the cost of the map
 */
class CostMap : public Cost
{
	public:
		/** @brief Constructor function */
		CostMap();

		/** @brief Destructor function */
		virtual ~CostMap();

		/**
		 * @brief Sets the cost map only for Cost that representing the terrain (or map)
		 * @param std::vector<dwl::environment::Cell> reward_map Reward map
		 */
		virtual void setCostMap(std::vector<dwl::environment::Cell> reward_map);

		/**
		 * @brief Abstract method for getting the cost value given a certain state
		 * @param Eigen::VectorXd state State value
		 * @return double Return the cost at defined state
		 */
		virtual void get(AdjacencyMap& state);


	private:
		/** @brief Adjacency map that defines a cost map of the environment */
		AdjacencyMap cost_map_;

		/** @brief Indicates if it the first cost-map message */
		bool is_first_update_;


}; //@class CostMap


} //@namespace planning

} //@namespace dwl


#endif
