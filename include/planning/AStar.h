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
		 * @param double computation_time Allowed time for computing a solution (in seconds)
		 * @return bool Return true if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target,	double computation_time = std::numeric_limits<double>::max());


	private:
		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest A* path
		 * @param Vertex source Source vertex
		 * @param Vertex target Target vertex
		 */
		void findShortestPath(Vertex source, Vertex target);

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace planning
} //@namespace dwl


#endif
