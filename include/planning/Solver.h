#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <environment/AdjacencyEnvironment.h>
#include <utils/utils.h>
//#include <pthread.h>


namespace dwl
{

namespace planning
{

/**
 * @class Solver
 * @brief Abstract class for solving graph-searching or optimization problems
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
		 * @brief Sets terrain information, specially in the reward map of the terrain that is converting to a cost map
		 * @param std::vector<Cell> reward_map Reward map to set in the cost map
		 */
		void setTerrainInformation(std::vector<Cell> reward_map);

		/**
		 * @brief Sets the adjacency model that is used for graph-searchin solvers, i.e. path-planning problems
		 * @param dwl::environment::AdjacencyEnvironment* adjacency_model Adjacency model
		 */
		void setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model);

		/**
		 * @brief Abstract method for computing a shortest-path using graph search algorithms such as A* or Dijkstrap
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param double orientation Orientation of the body
		 * @return bool Return true if it was computed a solution
		 */
		virtual bool compute(Vertex source, Vertex target, double orientation);


		/**
		 * @brief Gets the shortest-path only for graph searching algorithms
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

		/** @brief Adjacency model of the environment */
		environment::AdjacencyEnvironment* environment_;

		/** @brief Indicates if it a graph-searching algorithm */
		bool is_graph_searching_algorithm_;

		PreviousVertex previous_;

		double total_cost_;

		bool is_settep_adjacency_model_;

//		pthread_mutex_t solver_lock_;
};

} //@namespace planning

} //@namespace dwl


#endif
