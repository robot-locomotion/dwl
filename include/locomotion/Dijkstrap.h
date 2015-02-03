#ifndef DWL_Dijktra_H
#define DWL_Dijktra_H

#include <locomotion/Solver.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class Dijkstrap
 * @brief Class for solving a shortest-search problem using the Dijkstrap algorithm
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
		 * @return bool Returns true if Dijkstrap algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using Dijkstrap algorithm
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @return bool Returns true if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target, double computation_time);


	private:
		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest Dijkstrap path
		 * @param Vertex source Source vertex
		 * @param AdjacencyMap adjacency_map Adjacency map
		 */
		void findShortestPath(Vertex source, Vertex target, AdjacencyMap adjacency_map);

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
