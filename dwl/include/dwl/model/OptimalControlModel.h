#ifndef DWL__MODEL__OPTIMAL_CONTROL_MODEL__H
#define DWL__MODEL__OPTIMAL_CONTROL_MODEL__H

#include <dwl/model/OptimizationModel.h>
#include <dwl/model/DynamicalSystem.h>
#include <dwl/model/Constraint.h>
#include <dwl/model/Cost.h>


namespace dwl
{

namespace model
{

/**
 * @class OptimalControlModel
 * @brief An optimal control problem requires information of constraints (dynamical, active or
 * inactive) and cost functions. Thus, OptimalControlModel is a class that allows us to implement
 * different locomotion (control and planning) models. For instance, we can defined different
 * decision variables (time; base and joint position, velocity and acceleration; contact forces)
 */
class OptimalControlModel : public OptimizationModel
{
	public:
		/** @brief Constructor function */
		OptimalControlModel();

		/** @brief Destructor function */
		~OptimalControlModel();

		/** @brief Initializes the optimization model, i.e. the dimensions of the optimization
		 * vectors */
		void init();

		/**
		 * @brief Sets the initial trajectory
		 * @param WholeBodyTrajectory& Initial whole-body trajectory
		 */
		void setStartingTrajectory(WholeBodyTrajectory& initial_trajectory);

		/**
		 * @brief Gets the starting point of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full initial point
		 */
		void getStartingPoint(Eigen::Ref<Eigen::VectorXd> full_initial_point);

		/**
		 * @brief Evaluates the bounds of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full state lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full state upper bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint upper bound
		 */
		void evaluateBounds(Eigen::Ref<Eigen::VectorXd> full_state_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_state_upper_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_upper_bound);
		/**
		 * @brief Evaluates the constraint function given a current decision state
		 * @param Eigen::Ref<Eigen::VectorXd> Constraint vector
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
								 const Eigen::Ref<const Eigen::VectorXd>& decision_var);

		/**
		 * @brief Evaluates the cost function given a current decision state
		 * @param double& Cost value
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateCosts(double& cost,
						   const Eigen::Ref<const Eigen::VectorXd>& decision_var);

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
		void addConstraint(Constraint* constraint);

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
		std::vector<Constraint*> getConstraints();

		/** @brief Gets the cost functions */
		std::vector<Cost*> getCosts();

		/** @brief Gets the horizon value of the optimization problem */
		const unsigned int& getHorizon();


	protected:
		/** @brief Dynamical system constraint pointer */
		DynamicalSystem* dynamical_system_;

		/** @brief Vector of active and inactive constraints pointers */
		std::vector<Constraint*> constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		/** @brief Numerical differentiation mode */
		Eigen::NumericalDiffMode num_diff_mode_;

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_dynamic_system_;

		/** @brief Indicates if it was added a constraint in the solver */
		bool is_added_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;
};

} //@namespace model
} //@namespace dwl


#endif
