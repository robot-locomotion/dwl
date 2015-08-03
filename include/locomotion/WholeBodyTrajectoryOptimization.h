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

class WholeBodyTrajectoryOptimization
{
	public:
		WholeBodyTrajectoryOptimization();
		~WholeBodyTrajectoryOptimization();


		void init(solver::OptimizationSolver* solver);

		void addDynamicalSystem(model::DynamicalSystem* system);
		void addConstraint(model::Constraint* constraint);
		void addCost(model::Cost* cost);

		void setHorizon(unsigned int horizon);

		bool compute(const LocomotionState& current_state,
					 const LocomotionState& desired_state,
					 double computation_time);

//		std::vector<LocomotionState> getWholeBodyTrajectory();


	private:
		solver::OptimizationSolver* solver_;
};

} //@namespace locomotion
} //@namespace dwl

#endif
