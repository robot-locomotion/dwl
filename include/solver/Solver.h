#ifndef DWL__SOLVER__SOLVER__H
#define DWL__SOLVER__SOLVER__H

#include <robot/Robot.h>
#include <model/OptimizationModel.h>
#include <environment/AdjacencyEnvironment.h>
#include <utils/utils.h>


namespace dwl
{

namespace solver
{

/**
 * @class Solver
 * @brief Abstract class for implementing different solver such as graph searcher or optimizer. For graph
 * searcher requires to define an adjacency model.
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
		 * @return True if was initialized
		 */
		virtual bool init() = 0;

		/**
		 * @brief Defines the environment information
		 * @param Robot* Encapsulated all the robot information
		 * @param EnvironmentInformation* Encapsulates all the information of the environment
		 */
		void reset(robot::Robot* robot, environment::EnvironmentInformation* environment);

		/**
		 * @brief Sets the model that is used for optimization solvers
		 * @param model::OptimizationModel* Encapsulate the constraints and cost functions (model) of
		 * the optimization problem
		 */
		void setModel(model::OptimizationModel* model);

		/**
		 * @brief Sets the adjacency model that is used for graph searching solvers
		 * @param AdjacencyEnvironment* Adjacency model
		 */
		void setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model);

		/**
		 * @brief Abstract method for computing a shortest-path using graph search algorithms such as A*
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 * @param double Allowed time for computing a solution (in seconds)
		 * @return True if it was computed a solution
		 */
		virtual bool compute(Vertex source, Vertex target,
							 double computation_time = std::numeric_limits<double>::max());

		/**
		 * @brief Abstract method for computing a solution of an optimization problem
		 * @return True if it was computed a solution
		 */
		virtual bool compute(double computation_time = std::numeric_limits<double>::max());

		/**
		 * @brief Gets the shortest-path only for graph searching algorithms
		 * @param Vertex target Target vertex
		 * @return The path as a list of vertex
		 */
		std::list<Vertex> getShortestPath(Vertex source, Vertex target);

		/**
		 * @brief Gets the minimum cost (total cost) for the computed solution
		 * @return The total cost of the planned path
		 */
		double getMinimumCost();

		/**
		 * @brief Gets the name of the solver
		 * @return The name of the solver
		 */
		std::string getName();


	protected:
		/** @brief Name of the solver */
		std::string name_;

		/** @brief Robot properties */
		robot::Robot* robot_;

		/** @brief Environment information */
		environment::EnvironmentInformation* environment_;

		/** @brief Optimizer' model which defines cost functions and constraints */
		model::OptimizationModel* model_;

		/** @brief Adjacency model of the environment */
		environment::AdjacencyEnvironment* adjacency_;

		/** @brief Indicates if it is a graph-searching algorithm */
		bool is_graph_searching_algorithm_;

		/** @brief Indicates if it is an optimization problem */
		bool is_optimization_algorithm_;

		/** @brief Shortest previous vertex */
		PreviousVertex policy_;

		/** @brief Total cost of the path */
		double total_cost_;

		/** @brief Initial time of computation */
		clock_t time_started_;

		/** @brief Indicates if it was set a model for the optimization problem */
		bool is_set_model_;

		/** @brief Indicates if it was set an adjacency model */
		bool is_set_adjacency_model_;
};

} //@namespace solver
} //@namespace dwl


#endif
