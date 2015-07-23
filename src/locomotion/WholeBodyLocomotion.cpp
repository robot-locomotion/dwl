#include <locomotion/WholeBodyLocomotion.h>



namespace dwl
{


WholeBodyLocomotion::WholeBodyLocomotion() : planner_(NULL), is_set_planner_(false)
{
	is_learning_ = false;
}


WholeBodyLocomotion::~WholeBodyLocomotion()
{

}


void WholeBodyLocomotion::reset(locomotion::PlanningOfMotionSequence* planner)
{
	printf(BLUE "Setting the %s planner\n" COLOR_RESET, planner->getName().c_str());
	is_set_planner_ = true;
	planner_ = planner;
}


bool WholeBodyLocomotion::init()
{
	if (!is_set_planner_) {
		printf(YELLOW "Could not initialized the whole-body locomotion because has not been set the planner\n"
				COLOR_RESET);
		return false;
	}

	bool init = planner_->initPlan();

	return init;
}


void WholeBodyLocomotion::resetGoal(Pose goal)
{
	if (is_set_planner_)
		planner_->resetGoal(goal);
	else
		printf(YELLOW "Could not set the reset the goal pose because has not been set the planner\n"
				COLOR_RESET);
}


bool WholeBodyLocomotion::compute(Pose current_pose)
{
	if (!is_set_planner_) {
		printf(YELLOW "Could not computed the plan because has not been set the planner\n" COLOR_RESET);
		return false;
	}

	bool plan = planner_->computePlan(current_pose);

	return plan;
}


void WholeBodyLocomotion::setTerrainInformation(std::vector<RewardCell> reward_map)
{
	if (is_set_planner_) {
		planner_->setEnvironmentInformation(reward_map);
	} else
		printf(YELLOW "Could not set the terrain reward information because has not been set the planner\n"
				COLOR_RESET);
}


void WholeBodyLocomotion::setTerrainInformation(std::vector<Cell> obstacle_map)
{
	if (is_set_planner_) {
		planner_->setEnvironmentInformation(obstacle_map);
	} else
		printf(YELLOW "Could not set the terrain obstacle information because has not been set the planner\n"
				COLOR_RESET);
}


void WholeBodyLocomotion::setComputationTime(double computation_time)
{
	if (is_set_planner_)
		planner_->setComputationTime(computation_time);
	else
		printf(YELLOW "Could not set the allowed computation time because has not been set the planner\n"
				COLOR_RESET);
}


void WholeBodyLocomotion::setComputationTime(double computation_time, TypeOfSolver solver)
{
	if (is_set_planner_)
		planner_->setComputationTime(computation_time, solver);
	else
		printf(YELLOW "Could not set the allowed computation time because has not been set the planner\n"
				COLOR_RESET);
}


std::vector<Pose> WholeBodyLocomotion::getBodyPath()
{
	return planner_->getBodyPath();
}


std::vector<Contact> WholeBodyLocomotion::getContactSequence()
{
	return planner_->getContactSequence();
}

} //@namespace dwl
