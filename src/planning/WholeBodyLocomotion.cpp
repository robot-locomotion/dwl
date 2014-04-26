#include <planning/WholeBodyLocomotion.h>



namespace dwl
{


WholeBodyLocomotion::WholeBodyLocomotion() : planner_(NULL), is_settep_planner_(false)
{
	is_learning_ = false;
}


void WholeBodyLocomotion::reset(dwl::planning::PlanningOfMotionSequences* planner)
{
	printf(BLUE "Setting the %s planner\n" COLOR_RESET, planner->getName().c_str());
	is_settep_planner_ = true;
	planner_ = planner;
}


void WholeBodyLocomotion::addConstraint(planning::Constraint* constraint)
{
	if (is_settep_planner_)
		planner_->addConstraint(constraint);
	else {
		printf(YELLOW "Could not add the %s constraint because has not been setted the planner\n" COLOR_RESET, constraint->getName().c_str());
	}
}


void WholeBodyLocomotion::removeConstraint(std::string constraint_name)
{
	if (is_settep_planner_)
		planner_->removeConstraint(constraint_name);
	else {
		printf(YELLOW "Could not remove the %s constraint because has not been setted the planner\n" COLOR_RESET, constraint_name.c_str());
	}
}


void WholeBodyLocomotion::addCost(planning::Cost* cost)
{
	if (is_settep_planner_)
		planner_->addCost(cost);
	else {
		printf(YELLOW "Could not add the %s cost because has not been setted the planner\n" COLOR_RESET, cost->getName().c_str());
	}
}


void WholeBodyLocomotion::removeCost(std::string cost_name)
{
	if (is_settep_planner_)
		planner_->removeCost(cost_name);
	else {
		printf(YELLOW "Could not remove the %s cost because has not been setted the planner\n" COLOR_RESET, cost_name.c_str());
	}
}


bool WholeBodyLocomotion::init(std::vector<double> start, std::vector<double> goal)
{
	if (is_settep_planner_) {
		if (!planner_->initPlan(start, goal)) {
			//printf(RED "Could not initiliazed the %s planner\n" COLOR_RESET, planner_->getName().c_str());
			return false;
		}
		if (is_learning_)
			printf("WholeBodyLocomotion is using a learning component\n");
	}
	else {
		printf(YELLOW "Could not initialize the whole-body locomotor because has not been setted the planner\n" COLOR_RESET);

		return false;
	}

	return true;
}


bool WholeBodyLocomotion::computePlan()
{
	if (is_settep_planner_) {
		if (!planner_->computePlan()) {

			return false;
		}
	}
	else {
		printf(YELLOW "Could not compute the plan because has not been setted the planner\n" COLOR_RESET);
	}

	return true;
}


void WholeBodyLocomotion::changeGoal(std::vector<double> goal)
{
	if (is_settep_planner_)
		planner_->changeGoal(goal);
	else
		printf(YELLOW "Could not change the goal because has not been setted the planner\n" COLOR_RESET);
}


} //@namespace dwl
