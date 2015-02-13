#ifndef DWL_Dijktra_H
#define DWL_Dijktra_H

#include <solver/Solver.h>


namespace dwl
{

namespace solver
{

/**
 * @class Dijkstrap
 * @brief Class for solving a shortest-search problem using the Dijkstrap algorithm. This class derive from the
 * Solver class
 */
class Dijkstrap : public Solver
{
	public:
		/** @brief Constructor function */
		Dijkstrap();

		/** @brief Destructor function */
		~Dijkstrap();

		/**
		 * @brief Initializes the Dijkstrap algorithm
		 * @return True if Dijkstrap algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using Dijkstrap algorithm
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 * @return True if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target, double computation_time);


	private:
		/**
		 * @brief Computes the minimum cost and previous vertex according to the shortest Dijkstrap path
		 * @param Vertex Source vertex
		 * @param AdjacencyMap Adjacency map
		 */
		void findShortestPath(Vertex source, Vertex target, AdjacencyMap adjacency_map);

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace solver
} //@namespace dwl

#endif
