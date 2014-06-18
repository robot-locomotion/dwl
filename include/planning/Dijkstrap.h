#ifndef DWL_Dijktra_H
#define DWL_Dijktra_H

#include <planning/Solver.h>

/*
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>*/


namespace dwl
{

namespace planning
{

/**
 * @class Dijkstrap
 * @brief Class for solving a shortest-search problem using the Dijkstrap algorithm
 */
class Dijkstrap : public dwl::planning::Solver
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
		 * @brief Computes the shortest-path according to Dijkstrap algorithm
		 * @param SolverInterface& solver_interface Interface for the applied solver
		 */
		bool compute(SolverInterface solver_interface);

		/**
		 * @brief Computes the minimun cost and previous vertex according to the shortest Dijkstrap path
		 * @param Vertex source Source vertex
		 * @param AdjacencyMap adjacency_map Adjacency map
		 * @param CostMap& min_cost Minimum cost of the vertex
		 * @param PreviousVertex& Previous vertex
		 */
		void findShortestPath(Vertex source, AdjacencyMap adjacency_map, CostMap& min_cost, PreviousVertex& previous);


	private:

		//std::list<Vertex> path_;

		/*
		typedef boost::adjacency_list <boost::listS, boost::vecS, boost::directedS,
					boost::no_property, boost::property <boost::edge_weight_t, int> > graph;

		typedef boost::graph_traits <graph>::vertex_descriptor vertex_descriptor;

		typedef std::pair<int, int> edge;*/
};

} //@namespace planning

} //@namespace dwl

#endif
