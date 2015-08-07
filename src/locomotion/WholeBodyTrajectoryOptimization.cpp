#include <locomotion/WholeBodyTrajectoryOptimization.h>


namespace dwl
{

namespace locomotion
{

WholeBodyTrajectoryOptimization::WholeBodyTrajectoryOptimization() : solver_(NULL)
{

}


WholeBodyTrajectoryOptimization::~WholeBodyTrajectoryOptimization()
{

}

void WholeBodyTrajectoryOptimization::init(solver::OptimizationSolver* solver)
{
	solver_ = solver;
	solver_->init();
}


void WholeBodyTrajectoryOptimization::addDynamicalSystem(model::DynamicalSystem* system)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addDynamicalSystem(system);
	else
		printf(RED "FATAL: there was not defined a solver for setting the %s dynamical constraint"
				COLOR_RESET, system->getName().c_str());
}


void WholeBodyTrajectoryOptimization::addConstraint(model::Constraint* constraint)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addConstraint(constraint);
	else
		printf(RED "FATAL: there was not defined a solver for setting the %s constraint"
				COLOR_RESET, constraint->getName().c_str());
}


void WholeBodyTrajectoryOptimization::addCost(model::Cost* cost)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().addCost(cost);
	else
		printf(RED "FATAL: there is not defined a solver for setting the %s cost"
				COLOR_RESET, cost->getName().c_str());
}


void WholeBodyTrajectoryOptimization::setHorizon(unsigned int horizon)
{
	if (solver_ != NULL)
		solver_->getOptimizationModel().setHorizon(horizon);
	else
		printf(RED "FATAL: there is not defined a solver for setting the horizon" COLOR_RESET);
}


void WholeBodyTrajectoryOptimization::setStepIntegrationTime(const double& step_time)
{
	solver_->getOptimizationModel().getDynamicalSystem()->setStepIntegrationTime(step_time);
}


bool WholeBodyTrajectoryOptimization::compute(const LocomotionState& current_state,
											  const LocomotionState& desired_state,
											  double computation_time)
{
	if (solver_ != NULL) {
		// Setting the current state and the starting state for the optimization
		solver_->getOptimizationModel().getDynamicalSystem()->setInitialState(current_state);
		solver_->getOptimizationModel().getDynamicalSystem()->setStartingState(current_state);

		// Setting the desired state to the cost functions
		unsigned int num_cost = solver_->getOptimizationModel().getCosts().size();
		for (unsigned int i = 0; i < num_cost; i++)
			solver_->getOptimizationModel().getCosts()[i]->setDesiredState(desired_state);

		return solver_->compute(computation_time);
	} else {
		printf(RED "FATAL: there was not defined a solver" COLOR_RESET);
		return false;
	}
}


model::DynamicalSystem* WholeBodyTrajectoryOptimization::getDynamicalSystem()
{
	return solver_->getOptimizationModel().getDynamicalSystem();
}


std::vector<LocomotionState>& WholeBodyTrajectoryOptimization::getWholeBodyTrajectory()
{
	return solver_->getWholeBodyTrajectory();
}


} //@namespace locomotion
} //@namespace dwl
