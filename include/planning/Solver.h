#ifndef DWL_Solver_H
#define DWL_Solver_H

#include <planning/Constraint.h>
#include <planning/Cost.h>
#include <Eigen/Dense>
#include <vector>
#include <utils/macros.h>
#include <pthread.h>


namespace dwl
{

namespace planning
{

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
		 * @param Eigen::MatrixXd& solution Solution of the problem
		 * @return bool Return true if it was computed a solution
		 */
		virtual bool compute(Eigen::MatrixXd& solution) = 0;

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
		 * @brief Gets the name of the solver
		 * @return std::string Return the name of the solver
		 */
		std::string getName();

	private:

	protected:
		/** @brief Name of the solver */
		std::string name_;

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


		pthread_mutex_t solver_lock_;
};

} //@namespace planning

} //@namespace dwl


#endif
