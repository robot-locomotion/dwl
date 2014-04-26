#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

namespace planning
{


PlanningOfMotionSequences::PlanningOfMotionSequences() : solver_(NULL), is_settep_solver_(false), is_initialized_planning_(false)
{

}


void PlanningOfMotionSequences::reset(Solver* solver)
{
	printf(BLUE "Setting the %s solver\n" COLOR_RESET, solver->getName().c_str());
	is_settep_solver_ = true;
	solver_ = solver;
}


void PlanningOfMotionSequences::addConstraint(Constraint* constraint)
{
	if (is_settep_solver_)
		solver_->addConstraint(constraint);
	else
		printf(YELLOW "Could not add the constraints because it is necessary to set up the solver\n" COLOR_RESET);
}


void PlanningOfMotionSequences::removeConstraint(std::string constraint_name)
{
	if (is_settep_solver_)
		solver_->removeConstraint(constraint_name);
	else
		printf(YELLOW "Could not remove the constraints because it is necessary to set up the solver\n" COLOR_RESET);
}


void PlanningOfMotionSequences::addCost(Cost* cost)
{
	if (is_settep_solver_)
		solver_->addCost(cost);
	else
		printf(YELLOW "Could not add the cost because it is necessary to set up the solver\n" COLOR_RESET);
}


void PlanningOfMotionSequences::removeCost(std::string cost_name)
{
	if (is_settep_solver_)
		solver_->removeCost(cost_name);
	else
		printf(YELLOW "Could not remove the cost because it is necessary to set up the solver\n" COLOR_RESET);
}


bool PlanningOfMotionSequences::initPlan(std::vector<double> start, std::vector<double> goal)
{
	if (is_settep_solver_) {
		if (!init(start, goal)) {
			printf(RED "Could not init the %s planning algorithm\n" COLOR_RESET, name_.c_str());

			return false;
		}
		is_initialized_planning_ = true;
	}
	else {
		printf(YELLOW "Could not initialize the %s planning because has not been setted the solver\n" COLOR_RESET, name_.c_str());

		return false;
	}

	return true;
}


bool PlanningOfMotionSequences::computePlan()
{
	if (is_initialized_planning_) {
		if (is_settep_solver_) {
			if (!compute()) {
				printf(RED "Could not compute the %s planning algorithm\n" COLOR_RESET, name_.c_str());

				return false;
			}
		}
		else {
			printf(YELLOW "Could not execute the %s planning because has not been setted the solver\n" COLOR_RESET, name_.c_str());

			return false;
		}
	}
	else {
		printf(YELLOW "Could not execute the %s planning because has not been initialized\n" COLOR_RESET, name_.c_str());

		return false;
	}

	return true;
}


void PlanningOfMotionSequences::changeGoal(std::vector<double> goal)
{
	printf(GREEN "Changed the goal state\n" COLOR_RESET);
	pthread_mutex_lock(&planning_lock_);
	goal_state_ = goal;
	pthread_mutex_unlock(&planning_lock_);
}


std::string PlanningOfMotionSequences::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
