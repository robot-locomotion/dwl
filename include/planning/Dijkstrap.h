#ifndef DWL_Dijktra_H
#define DWL_Dijktra_H

#include <planning/Solver.h>


namespace dwl
{

namespace planning
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
		 * @return bool Return true if Dijkstrap algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using Dijkstrap algorithm
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param double orientation Orientation of the body
		 * @return bool Return true if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target, double orientation);


	private:
		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest Dijkstrap path
		 * @param CostMap& min_cost Minimum cost of the vertex
		 * @param PreviousVertex& Previous vertex
		 * @param Vertex source Source vertex
		 * @param AdjacencyMap adjacency_map Adjacency map
		 */
		void findShortestPath(CostMap& min_cost, PreviousVertex& previous, Vertex source, AdjacencyMap adjacency_map);

};

} //@namespace planning

} //@namespace dwl

#endif
