#include <planning/WholeBodyLocomotion.h>



namespace dwl
{


WholeBodyLocomotion::WholeBodyLocomotion() : planner_(NULL)
{
	is_learning_ = false;
}


void WholeBodyLocomotion::reset(dwl::planning::PlanningOfMotionSequences* planner)
{
	printf(BLUE "Setting the %s planner\n" COLOR_RESET, planner->getName().c_str());

	planner_ = planner;
}


void WholeBodyLocomotion::addConstraint(planning::Constraint* constraint)
{
	planner_->addConstraint(constraint);
}


void WholeBodyLocomotion::addCost(planning::Cost* cost)
{
	planner_->addCost(cost);
}


bool WholeBodyLocomotion::init()
{
	std::vector<double> start, goal;
	if (!planner_->init(start, goal)) {
		printf(RED "Could not initiliazed the %s planner\n" COLOR_RESET, planner_->getName().c_str());
		return false;
	}

	if (is_learning_)
		printf("WholeBodyLocomotion is using a learning component\n");

	return true;
}


bool WholeBodyLocomotion::computePlan()
{
	if (!planner_->compute()) {
		printf(RED "Could not compute the %s planning algorithm\n" COLOR_RESET, planner_->getName().c_str());
		return false;
	}

	return true;
}


} //@namespace dwl
