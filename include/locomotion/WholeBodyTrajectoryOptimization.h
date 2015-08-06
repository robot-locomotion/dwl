#ifndef DWL__LOCOMOTION__WHOLE_BODY_TRAJECTORY_OPTIMIZATION__H
#define DWL__LOCOMOTION__WHOLE_BODY_TRAJECTORY_OPTIMIZATION__H

#include <solver/OptimizationSolver.h>
#include <model/DynamicalSystem.h>
#include <model/Constraint.h>
#include <model/Cost.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class WholeBodyTrajectoryOptimization
 * @brief This class solves whole-body trajectory optimization problem given dynamical system
 * constraint and set of constraints and cost function.
 */
class WholeBodyTrajectoryOptimization
{
	public:
		/** @brief Constructor function */
		WholeBodyTrajectoryOptimization();

		/** @brief Destructor function */
		~WholeBodyTrajectoryOptimization();

		/**
		 * @brief Initializes the whole-body trajectory optimizer
		 * @param solver::OptimizationSolver* Pointer to the optimization solver
		 */
		void init(solver::OptimizationSolver* solver);

		/**
		 * @brief Adds the dynamical system constraint
		 * @param model::DynamicalSystem* Pointer to the dynamical system constraint
		 */
		void addDynamicalSystem(model::DynamicalSystem* system);

		/**
		 * @brief Adds the constraint
		 * @param modelk::Constraint* Pointer to the constraint
		 */
		void addConstraint(model::Constraint* constraint);

		/**
		 * @brief Adds the cost function
		 * @param model::Cost* Pointer to the cost function
		 */
		void addCost(model::Cost* cost);

		/**
		 * @brief Sets the horizon
		 * @param unsigned int Horizon value
		 */
		void setHorizon(unsigned int horizon);

		/**
		 * @brief Computes a whole-body trajectory
		 * @param const LocomotionState& Current whole-body state
		 * @param const LocomotionState& Desired whole-body state
		 * @param double Allowed computation time
		 */
		bool compute(const LocomotionState& current_state,
					 const LocomotionState& desired_state,
					 double computation_time);

		/** @brief Gets the dynamical system constraint */
		model::DynamicalSystem* getDynamicalSystem();

		/**
		 * @brief Gets the whole-body trajectory
		 * @return std::vector<LocomotionState>& Whole-body trajectory
		 */
		std::vector<LocomotionState>& getWholeBodyTrajectory();


	private:
		/** @brief Optimization solver */
		solver::OptimizationSolver* solver_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
