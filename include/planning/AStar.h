#ifndef DWL_AStar_H
#define DWL_AStar_H

#include <planning/Solver.h>
#include <environment/PlaneGrid.h>


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
		 * @brief Computes the shortest-path according to A* algorithm
		 * @param SolverInterface& solver_interface Interface for the applied solver
		 */
		bool compute(SolverInterface solver_interface);

		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest A* path //TODO
		 * @param Vertex source Source vertex
		 * @param AdjacencyMap adjacency_map Adjacency map
		 * @param CostMap& min_cost Minimum cost of the vertex
		 * @param PreviousVertex& Previous vertex
		 */
		void findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target, double orientation, AdjacencyMap adjacency_map);

		/**
		 * @brief Estimates the heuristic cost from a source to a target vertex
		 * @param Vertex source Source vertex
		 * @param Vertex target Target vertex
		 */
		double heuristicCostEstimate(Vertex source, Vertex target);


	private:


};

} //@namespace planning

} //@namespace dwl


#endif
