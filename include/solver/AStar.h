#ifndef DWL__SOLVER__ASTAR__H
#define DWL__SOLVER__ASTAR__H

#include <solver/Solver.h>


namespace dwl
{

namespace solver
{

/**
 * @class AStar
 * @brief Class for solving a shortest-search problem using the A* algorithm. This class derives
 * from the SearchTreeSolver class
 */
class AStar : public SearchTreeSolver
{
	public:
		/** @brief Constructor function */
		AStar();

		/** @brief Destructor function */
		~AStar();

		/**
		 * @brief Initializes the A* algorithm
		 * @return True if A* algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using A* algorithm
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 * @param double Allowed time for computing a solution (in seconds)
		 * @return True if it was computed a solution
		 */
		bool compute(Vertex source,
					 Vertex target,
					 double computation_time);


	private:
		/**
		 * @brief Computes the minimum cost and previous vertex according to the shortest A* path
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		void findShortestPath(Vertex source,
							  Vertex target);

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace solver
} //@namespace dwl

#endif
