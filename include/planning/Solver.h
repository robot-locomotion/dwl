#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <list>
#include <set>

#include <utils/utils.h>
//#include <pthread.h>


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
 * @brief Struct that defines the graph-searching interface
 */
struct GraphSearching
{
	Vertex source;
	Vertex target;
	AdjacencyMap adjacency_map;
};

/**
 * @brief Struct that defines the different solver interfaces
 */
struct SolverInterface
{
	GraphSearching searcher; /**< Graph-searching algorithms */
	//Optimization optimizer;
};

/**
 * @class Solver
 * @brief Abstract class for computation of a solution
 */
class Solver
{
	public:
		/** @brief Constructor function */
		Solver();

		/** @brief Destructor function */
		virtual ~Solver();

		/**
		 * @brief Abstract method for initialization of the solver
		 * @return bool Return true if was initialized
		 */
		virtual bool init() = 0;

		/**
		 * @brief Abstract method for computing a solution
		 * @param SolverInterface& solver_interface Interface for the applied solver
		 * @return bool Return true if it was computed a solution
		 */
		virtual bool compute(SolverInterface solver_interface) = 0;

		/**
		 * @brief Get the shortes path only for graph searching algorithms
		 * @param Vertex target Target vertex
		 * @return std::list<Vertex> Returns the path as a list of vertex
		 */
		std::list<Vertex> getShortestPath(Vertex target);

		/**
		 * @brief Gets the minimum cost (total cost) for the computed solution
		 * @return double Return the total cost
		 */
		double getMinimumCost();

		/**
		 * @brief Gets the name of the solver
		 * @return std::string Return the name of the solver
		 */
		std::string getName();


	private:



	protected:
		/** @brief Name of the solver */
		std::string name_;

		/** @brief Indicates if it a graph-searching algorithm */
		bool is_graph_searching_algorithm_;

		PreviousVertex previous_;

		double total_cost_;

//		pthread_mutex_t solver_lock_;
};

} //@namespace planning

} //@namespace dwl


#endif
