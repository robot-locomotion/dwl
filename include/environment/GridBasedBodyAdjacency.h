#ifndef DWL_GridBasedBodyAdjacency_H
#define DWL_GridBasedBodyAdjacency_H

#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

/**
 * @class GridBasedBodyAdjacency
 * @brief Class for building a grid-based body adjacency map
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
		 * @param dwl::AdjacencyMap& adjacency_map Adjacency map
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param double orientation Body orientation
		 */
		void computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target, double orientation);

		/**
		 * @brief Gets the successors of the current vertex
		 * @param std::list<Edge>& successors List of successors
		 * @param dwl::Vertex vertex Current vertex
		 * @param double orientation Current orientation
		 */
		void getSuccessors(std::list<Edge>& successors, Vertex vertex, double orientation);


	private:
		/**
		 * @brief Searchs the neighbors of a current vertex
		 * @param std::vector<Vertex>& neighbors The set of neighbors
		 * @param dwl::Vertex vertex_id Current vertex
		 */
		void searchNeighbors(std::vector<Vertex>& neighbors, Vertex vertex_id);

		/**
		 * @brief Computes the body cost of a current vertex
		 * @param dwl::Vertex vertex Current vertex
		 * @param double orientation Current orientation
		 */
		void computeBodyCost(double& cost, Vertex vertex, double orientation);

		/**
		 * @brief Ask if it is requested a stance adjacency
		 */
		bool isStanceAdjacency();

		/** @brief Indicates it was requested a stance or terrain adjacency */
		bool is_stance_adjacency_;

		/** @brief Vector of search areas */
		std::vector<SearchArea> stance_areas_;

		/** @brief Definition of the neighboring area (number of neighbors per size) */
		int neighboring_definition_;

		/** @brief Number of top reward for computing the stance cost */
		int number_top_reward_;

		/** @brief Uncertainty factor which is applicated in un-perceived environment */
		double uncertainty_factor_; // For unknown (non-perceive) areas

};

} //@namespace hyq
} //@namespace dwl

#endif
