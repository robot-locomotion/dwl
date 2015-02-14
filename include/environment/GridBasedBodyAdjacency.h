#ifndef DWL_GridBasedBodyAdjacency_H
#define DWL_GridBasedBodyAdjacency_H

#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

/**
 * @class GridBasedBodyAdjacency
 * @brief Class for building a grid-based body adjacency map of the environment. This class derives from
 * AdjacencyEnvironment class
 */
class GridBasedBodyAdjacency : public AdjacencyEnvironment
{
	public:
		/** @brief Constructor function */
		GridBasedBodyAdjacency();

		/** @brief Destructor function */
		~GridBasedBodyAdjacency();

		/**
		 * @brief Computes the whole adjacency map
		 * @param AdjacencyMap& Adjacency map
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		void computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target);

		/**
		 * @brief Gets the successors of the current vertex
		 * @param std::list<Edge>& List of successors
		 * @param Vertex Current state vertex
		 */
		void getSuccessors(std::list<Edge>& successors, Vertex state_vertex);


	private:
		/**
		 * @brief Searches the neighbors of a current vertex
		 * @param std::vector<Vertex>& The set of states neighbors
		 * @param Vertex Current state vertex
		 */
		void searchNeighbors(std::vector<Vertex>& neighbor_states, Vertex state_vertex);

		/**
		 * @brief Computes the body cost of a current vertex
		 * @param Vertex Current state vertex
		 */
		void computeBodyCost(double& cost, Vertex state_vertex);

		/** @brief Asks if it is requested a stance adjacency */
		bool isStanceAdjacency();

		/** @brief Indicates it was requested a stance or terrain adjacency */
		bool is_stance_adjacency_;

		/** @brief A Map of search areas */
		SearchAreaMap stance_areas_;

		/** @brief Definition of the neighboring area (number of neighbors per size) */
		int neighboring_definition_;

		/** @brief Number of top reward for computing the stance cost */
		int number_top_reward_;

		/** @brief Uncertainty factor which is applied in non-perceived environment */
		double uncertainty_factor_; // For unknown (non-perceive) areas
};

} //@namespace environment
} //@namespace dwl

#endif
