#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <planning/Constraint.h>
#include <planning/Cost.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>

//#include <pthread.h>


namespace dwl
{

namespace planning
{

/**
 * @brief Struct that defines the graph-searching interface
 */
struct GraphSearching
{
	Vertex source;
	Vertex target;
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
		 * @brief Adds a constraint in the solver
		 * @param dwl::planning::Constraint* Pointer to the constraint class
		 */
		void addConstraint(Constraint* constraint);

		/**
		 * @brief Removes a constraint in the solver
		 * @param std::string constraint_name Name of the constraint
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost in the solver
		 * @param dwl::planning::Cost* Pointer to the cost class
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost in the solver
		 * @param std::string cost_name Name of the cost
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Get the shortes path only for graph searching algorithms
		 * @param dwl::planning::Vertex target Target vertex
		 * @return std::list<Vertex> Returns the path as a list of vertex
		 */
		std::list<Vertex> getShortestPath(Vertex target);

		/**
		 *
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

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_active_constraint_;

		/** @brief Indicates if it was added an inactive constraint in the solver */
		bool is_added_inactive_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;

		/** @brief Vector of active constraints pointers */
		std::vector<Constraint*> active_constraints_;

		/** @brief Vector of inactive constraints pointers */
		std::vector<Constraint*> inactive_constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		PreviousVertex previous_;

		double total_cost_;

//		pthread_mutex_t solver_lock_;
};

} //@namespace planning

} //@namespace dwl


#endif
