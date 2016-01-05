#ifndef DWL__LOCOMOTION__WHOLE_BODY_TRAJECTORY_OPTIMIZATION__H
#define DWL__LOCOMOTION__WHOLE_BODY_TRAJECTORY_OPTIMIZATION__H

#include <dwl/ocp/OptimalControl.h>
#include <dwl/solver/OptimizationSolver.h>
#include <dwl/utils/SplineInterpolation.h>


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
		 * @param ocp::DynamicalSystem* Pointer to the dynamical system constraint
		 */
		void addDynamicalSystem(ocp::DynamicalSystem* system);

		/** @brief Removes the current dynamical system */
		void removeDynamicalSystem();

		/**
		 * @brief Adds the constraint
		 * @param ocp::Constraint* Pointer to the constraint
		 */
		void addConstraint(ocp::Constraint* constraint);

		/**
		 * @brief Removes the current dynamical system
		 * @param std::string Name constraint
		 */
		void removeConstraint(std::string name);

		/**
		 * @brief Adds the cost function
		 * @param ocp::Cost* Pointer to the cost function
		 */
		void addCost(ocp::Cost* cost);

		/**
		 * @brief Removes the cost function
		 * @param std::string Name cost
		 */
		void removeCost(std::string name);

		/**
		 * @brief Sets the horizon
		 * @param unsigned int Horizon value
		 */
		void setHorizon(unsigned int horizon);

		/**
		 * @brief Sets the integration step-time
		 * @param const double& Step time in seconds
		 */
		void setStepIntegrationTime(const double& step_time);

		/**
		 * @brief Sets the nominal trajectory
		 * @param WholeBodyTrajectory& Nominal whole-body trajectory
		 */
		void setNominalTrajectory(WholeBodyTrajectory& nom_trajectory);

		/**
		 * @brief Computes a whole-body trajectory
		 * @param const WholeBodyState& Current whole-body state
		 * @param const WholeBodyState& Desired whole-body state
		 * @param double Allowed computation time
		 */
		bool compute(const WholeBodyState& current_state,
					 const WholeBodyState& desired_state,
					 double computation_time);

		/** @brief Gets the dynamical system constraint */
		ocp::DynamicalSystem* getDynamicalSystem();

		/**
		 * @brief Gets the whole-body trajectory
		 * @return WholeBodyTrajectory& Whole-body trajectory
		 */
		const WholeBodyTrajectory& getWholeBodyTrajectory();

		/**
		 * @brief Gets the interpolated whole-body trajectory
		 * @param const double Time of interpolation
		 * @return WholeBodyTrajectory& Whole-body trajectory
		 */
		const WholeBodyTrajectory& getInterpolatedWholeBodyTrajectory(const double& interpolation_time);


	private:
		/** @brief Optimization solver */
		solver::OptimizationSolver* solver_;

		/** @brief Optimal control model */
		dwl::ocp::OptimalControl oc_model_;

		/** @brief Interpolated whole-body trajectory */
		WholeBodyTrajectory interpolated_trajectory_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
