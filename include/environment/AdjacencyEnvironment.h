#ifndef DWL_AdjacencyEnvironment_H
#define DWL_AdjacencyEnvironment_H

#include <environment/PlaneGrid.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{


class AdjacencyEnvironment
{
	public:
		/** @brief Constructor function */
		AdjacencyEnvironment();

		/** @brief Destructor function */
		virtual ~AdjacencyEnvironment();

		/**
		 * @brief Sets the terrain cost map from the reward map information
		 * @param std::vector<dwl::Cell> reward_map Reward map
		 */
		void setEnvironmentInformation(std::vector<Cell> reward_map);

		/**
		 * @brief Computes the whole adjancecy map, which is required by some algorithms such as Dijkstrap
		 * @param dwl::AdjacencyMap& adjacency_map Adjacency map
		 * @param Eigen::Vector3d position 2D position and orientation
		 */
		virtual void computeAdjacencyMap(AdjacencyMap& adjacency_map, Eigen::Vector3d position) = 0;


		virtual void getSuccessors(std::list<Edge>& successors, Vertex vertex) = 0;

		void checkStartAndGoalVertex(AdjacencyMap& adjacency_map, Vertex source, Vertex target);

		/**
		 * @brief Gets the name of the adjacency model
		 * @return std::string Return the name of the adjacency model
		 */
		std::string getName();


	protected:
		/** @brief Name of the adjacency model */
		std::string name_;

		/** @brief Object of the PlaneGrid class for defining the grid routines */
		environment::PlaneGrid gridmap_;

		/** @brief Gathers the cost values that are mapped using the vertex id */
		CostMap terrain_cost_map_;

		/** @brief Average cost which is used for unknown areas */
		double average_cost_;

		bool is_there_terrain_information_;


	private:

};


} //@namespace environment

} //@namespace dwl


#endif
