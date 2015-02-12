#ifndef DWL_AStar_H
#define DWL_AStar_H

#include <locomotion/Solver.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class AStar
 * @brief Class for solving a shortest-search problem using the A* algorithm. This class derives from the Solver
 * class
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
		bool compute(Vertex source, Vertex target,	double computation_time = std::numeric_limits<double>::max());


	private:
		/**
		 * @brief Computes the minimum cost and previous vertex according to the shortest A* path
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 */
		void findShortestPath(Vertex source, Vertex target);

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
