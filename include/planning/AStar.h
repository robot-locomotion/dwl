#ifndef DWL_AStar_H
#define DWL_AStar_H

#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

/**
 * @class AStar
 * @brief Class for solving a shortest-search problem using the A* algorithm
 */
class AStar : public Solver
{
	public:
		/** @brief Constructor function */
		AStar();

		/** @brief Destructor function */
		~AStar();

		/**
		 * @brief Initializes the A* algorithm
		 * @return bool Return true if A* algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using A* algorithm
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @return bool Return true if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target);


	private:
		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest A* path //TODO
		 * @param CostMap& min_cost Minimum cost of the vertex
		 * @param PreviousVertex& Previous vertex
		 * @param Vertex source Source vertex
		 * @param Vertex target Target vertex
		 * @param AdjacencyMap adjacency_map Adjacency map
		 */
		void findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target, AdjacencyMap adjacency_map);

		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest A* path //TODO
		 * @param CostMap& min_cost Minimum cost of the vertex
		 * @param PreviousVertex& Previous vertex
		 * @param Vertex source Source vertex
		 * @param Vertex target Target vertex
		 */
		void findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target);

		/** @brief Indicates if it's computed a whole adjacency map before the searching */
		bool compute_whole_adjacency_map_;

};

} //@namespace planning
} //@namespace dwl


#endif
