#ifndef DWL__OCP__OPTIMAL_CONTROL__H
#define DWL__OCP__OPTIMAL_CONTROL__H

#include <dwl/model/OptimizationModel.h>
#include <dwl/ocp/DynamicalSystem.h>
#include <dwl/ocp/Constraint.h>
#include <dwl/ocp/Cost.h>



namespace dwl
{

namespace ocp
{

/**
 * @class OptimalControl
 * @brief An optimal control problem requires information of constraints (dynamical, active or
 * inactive) and cost functions. Thus, OptimalControlModel is a class that allows us to implement
 * different locomotion (control and planning) models. For instance, we can defined different
 * decision variables (time; base and joint position, velocity and acceleration; contact forces)
 */
class OptimalControl : public model::OptimizationModel
{
	public:
		/** @brief Constructor function */
		OptimalControl();

		/** @brief Destructor function */
		~OptimalControl();

		/** @brief Initializes the optimization model, i.e. the dimensions of the optimization
		 * vectors */
		void init(bool only_soft_constraints);

		/**
		 * @brief Sets the initial trajectory
		 * @param WholeBodyTrajectory& Initial whole-body trajectory
		 */
		void setStartingTrajectory(WholeBodyTrajectory& initial_trajectory);

		/**
		 * @brief Gets the starting point of the problem
		 * @param double* Initial values for the decision variables, $x$
		 * @param int Number of the decision variables
		 */
		void getStartingPoint(double* decision, int decision_dim);

		/**
		 * @brief Evaluates the bounds of the optimal control problem
		 * @param double* Lower bounds $x^L$ for $x$
		 * @param double* Upper bounds $x^L$ for $x$
		 * @param int Number of decision variables (dimension of $x$)
		 * @param double* Lower bounds of the constraints $g^L$ for $x$
		 * @param double* Upper bounds of the constraints $g^L$ for $x$
		 * @param int Number of constraints (dimension of $g(x)$)
		 */
		void evaluateBounds(double* decision_lbound, int decision_dim1,
							double* decision_ubound, int decision_dim2,
							double* constraint_lbound, int constraint_dim1,
							double* constraint_ubound, int constraint_dim2);

		/**
		 * @brief Evaluates the cost function of the optimal control problem
		 * decision state
		 * @param double& Value of the objective function ($f(x)$).
		 * @param const double* Array of the decision variables, $x$, at which the cost functions,
		 * $f(x)$, is evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		void evaluateCosts(double& cost,
						   const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the constraint function given a
		 * current decision state
		 * @param double* Array of constraint function values, $g(x)$
		 * @param int Number of constraint variables (dimension of $g(x)$)
		 * @param const double* Array of the decision variables, $x$, at which the constraint functions,
		 * $g(x)$, are evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		void evaluateConstraints(double* constraint, int constraint_dim,
								 const double* decision, int decision_dim);

		/**
		 * @brief Evaluates the solution from an optimizer
		 * @param const Eigen::Ref<const Eigen::VectorXd>& Solution vector
		 * @return WholeBodyTrajectory& Returns the whole-body trajectory solution
		 */
		WholeBodyTrajectory& evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution);

		/**
		 * @brief Adds the dynamical system (active constraints) to the optimization problem
		 * @param DynamicalSystem* Dynamical system constraint to add it
		 */
		void addDynamicalSystem(DynamicalSystem* dynamical_system);

		/** @brief Removes the current dynamical system constraint */
		void removeDynamicalSystem();

		/**
		 * @brief Adds an active or inactive constraints to the optimization problem
		 * @param Constraint* Constraint to add it
		 */
		void addConstraint(Constraint<WholeBodyState>* constraint);

		/**
		 * @brief Removes an active or inactive constraints to the optimization problem
		 * @param Constraint* Constraint to remove it
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost function for the optimization problem
		 * @param Cost* Cost to add it
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost function for the optimization problem
		 * @param Cost* Cost to remove it
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Sets the horizon steps
		 * @param unsigned int Horizon
		 */
		void setHorizon(unsigned int horizon);

		/** @brief Gets the dynamical system constraint */
		DynamicalSystem* getDynamicalSystem();

		/** @brief Gets the active and inactive constraints, which not include the dynamical one */
		std::vector<Constraint<WholeBodyState>*> getConstraints();

		/** @brief Gets the cost functions */
		std::vector<Cost*> getCosts();

		/** @brief Gets the horizon value of the optimization problem */
		const unsigned int& getHorizon();


	protected:
		/** @brief Dynamical system constraint pointer */
		DynamicalSystem* dynamical_system_;

		/** @brief Vector of active and inactive constraints pointers */
		std::vector<Constraint<WholeBodyState>*> constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_dynamic_system_;

		/** @brief Indicates if it was added a constraint in the solver */
		bool is_added_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;

		/** @brief Dimension of the terminal constraint vector */
		unsigned int terminal_constraint_dimension_;

		/** @brief Horizon of the optimal control problem */
		unsigned int horizon_;

		/** @brief Whole-body solution */
		WholeBodyTrajectory motion_solution_;
};

} //@namespace ocp
} //@namespace dwl


#endif
