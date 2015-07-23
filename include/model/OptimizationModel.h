#ifndef DWL__MODEL__OPTIMIZATION_MODEL__H
#define DWL__MODEL__OPTIMIZATION_MODEL__H

#include <model/DynamicalSystem.h>
#include <model/Constraint.h>
#include <model/Cost.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class OptimizationModel
 * @brief A NLP problem requires information of constraints (dynamical, active or inactive) and
 * cost functions. Thus, OptimizationModel is a class that allows us to implement different locomotion
 * (control and planning) models. For instance, we can defined different decision
 * variables (time; base and joint position, velocity and acceleration; contact forces), i.e.
 * the description of the state vector by implementing convertDecisionVariablesToStateModel method.
 */
class OptimizationModel
{
	public:
		/** @brief Constructor function */
		OptimizationModel();

		/** @brief Destructor function */
		~OptimizationModel();

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

		/** @brief Gets the dimension of the state vector of the optimization problem */
		unsigned int getDimensionOfState();

		/** @brief Gets the dimension of the constraints of the optimization problem */
		unsigned int getDimensionOfConstraints();

		/** @brief Gets the horizon value of the optimization problem */
		unsigned int getHorizon();


	protected:
		/** @brief Dynamical system constraint pointer */
		DynamicalSystem* dynamical_system_;

		/** @brief Vector of active and inactive constraints pointers */
		std::vector<Constraint*> constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		/** @brief Dimension of the state vector */
		unsigned int state_dimension_;

		/** @brief Dimension of the constraint vector */
		unsigned int constraint_dimension_;

		/** @brief Horizon of the optimal control problem */
		unsigned int horizon_;

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
