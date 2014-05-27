#ifndef DWL_DijktraAlgorithm_H
#define DWL_DijktraAlgorithm_H

#include <planning/Solver.h>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>


namespace dwl
{

namespace planning
{

/**
 * @brief Template struct that orders vertex
 */
template <typename Weight, typename Vertex>
struct pair_first_less
{
    bool operator()(std::pair<Weight,Vertex> vertex_1, std::pair<Weight,Vertex> vertex_2)
    {
        return vertex_1.first < vertex_2.first;
    }
};

/**
 * @class DijkstrapAlgorithm
 * @brief Class for solving a shortest-search problem using the Dijkstrap algorithm
 */
class DijkstrapAlgorithm : public dwl::planning::Solver
{
	public:
		/** @brief Constructor function */
		DijkstrapAlgorithm();

		/** @brief Destructor function */
		~DijkstrapAlgorithm();

		/**
		 * @brief Initializes the Dijkstrap algorithm
		 * @return bool Return true if Dijkstrap algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Abstract method for computing the shortest-path according to Dijkstrap algorithm
		 * @param SolverInterface& solver_interface Interface for the applied solver
		 */
		virtual bool compute(SolverInterface solver_interface);

		/**
		 * @brief Compute the minimun cost and previous vertex according to Dijkstrap algorithm
		 * @param dwl::planning::Vertex source Source vertex
		 * @param dwl::planning::AdjacencyMap& adjacency_map Adjacency map
		 * @param dwl::VertexCost& min_cost Minimum cost of the vertex
		 * @param dwl::planning::PreviousVertex& Previous vertex
		 */
		void DijkstraComputePath(Vertex source, AdjacencyMap& adjacency_map, VertexCost& min_cost, PreviousVertex& previous);


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
