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
		 * @brief Gets the cost value given a certain state
		 * @param Eigen::VectorXd adjacency_map State value
		 * @param Eigen::Vector3d robot_state 2D position and yaw orientation of the robot
		 * @param bool terrain_cost Defines if we want to ge the terrain cost or body cost
		 */
		virtual void get(AdjacencyMap& adjacency_map, Eigen::Vector3d robot_state, bool terrain_cost);

		/**
		 * @brief Computes the body cost-map
		 * @param Eigen::Vector3d robot_state 2D position and yaw orientation of the robot
		 */
		void computeBodyCostMap(Eigen::Vector3d robot_state);


	private:
		/**
		 * @brief Adds the cost to adjacent vertexs
		 * @param AdjacencyMap& adjacency_map Adjacency map
		 * @param Vertex vertex_id Vertex id
		 * @param double cost Cost value
		 */
		void addCostToAdjacentVertexs(AdjacencyMap& adjacency_map, Vertex vertex_id, double cost);

		/** @brief Adjacency map that defines a terrain cost map of the environment */
		AdjacencyMap terrain_cost_map_;

		/** @brief Adjacency map that defines a body cost map of the environment */
		AdjacencyMap body_cost_map_;

		/** @brief Gathers the cost values that are mapped using the vertex id */
		std::map<Vertex, Weight> costmap_;

		/** @brief Indicates if it the first cost-map message */
		bool is_first_update_;

		/** @brief Vector of search areas */
		std::vector<environment::SearchArea> stance_areas_;


}; //@class CostMap


} //@namespace planning

} //@namespace dwl


#endif
