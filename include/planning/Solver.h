#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <environment/AdjacencyEnvironment.h>
#include <utils/utils.h>


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
		 * @return bool Returns true if was initialized
		 */
		virtual bool init() = 0;

		/**
		 * @brief Specifies the environment information
		 * @param dwl::environment::EnvironmentInformation* environment Encapsulates all the information of the environment
		 */
		void reset(environment::EnvironmentInformation* environment); //TODO virtual method

		void setCurrentPose(Pose current_pose);

		/**
		 * @brief Sets the adjacency model that is used for graph-searchin solvers, i.e. path-planning problems
		 * @param dwl::environment::AdjacencyEnvironment* adjacency_model Adjacency model
		 */
		void setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model);

		/**
		 * @brief Abstract method for computing a shortest-path using graph search algorithms such as A* or Dijkstrap
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param double computation_time Allowed time for computing a solution (in seconds)
		 * @return bool Returns true if it was computed a solution
		 */
		virtual bool compute(Vertex source, Vertex target, double computation_time = std::numeric_limits<double>::max());

		/**
		 * @brief Abstract method for computing a solution of an optimization problem
		 * @param Eigen::MatrixXd hessian Hessian matrix
		 * @param Eigen::VectorXd gradient Gradient vector
		 * @param Eigen::MatrixXd constraint Constraint matrix
		 * @param Eigen::VectorXd low_bound Low bound
		 * @param Eigen::VectorXd upper_bound Upper bound
		 * @param Eigen::VectorXd low_constraint Low constraint
		 * @param Eigen::VectorXd upper_constraint Upper constraint
		 * @return bool Returns true if it was computed a solution
		 */
		virtual bool compute(Eigen::MatrixXd hessian, Eigen::VectorXd gradient, Eigen::MatrixXd constraint, Eigen::VectorXd low_bound,
				Eigen::VectorXd upper_bound, Eigen::VectorXd low_constraint, Eigen::VectorXd upper_constraint); //TODO represents as active, inactive and bound

		/**
		 * @brief Gets the shortest-path only for graph searching algorithms
		 * @param Vertex target Target vertex
		 * @return std::list<Vertex> Returns the path as a list of vertex
		 */
		std::list<Vertex> getShortestPath(Vertex source, Vertex target);

		/**
		 * @brief Gets the minimum cost (total cost) for the computed solution
		 * @return double Returns the total cost
		 */
		double getMinimumCost();

		/**
		 * @brief Gets the name of the solver
		 * @return std::string Returns the name of the solver
		 */
		std::string getName();


	private:


	protected:
		/** @brief Name of the solver */
		std::string name_;

		/** @brief Adjacency model of the environment */
		environment::AdjacencyEnvironment* adjacency_;

		/** @brief Indicates if it is a graph-searching algorithm */
		bool is_graph_searching_algorithm_;

		/** @brief Indicates if it is an optimization problem */
		bool is_optimization_algorithm_;

		/** @brief Shortest previous vertex */
		PreviousVertex previous_;

		/** @brief Initial time of computation */
		clock_t time_started_;

		/** @brief Total cost of the path */
		double total_cost_;

		/** @brief Indicates if it was set an adjacency model */
		bool is_settep_adjacency_model_;
};

} //@namespace planning
} //@namespace dwl


#endif
