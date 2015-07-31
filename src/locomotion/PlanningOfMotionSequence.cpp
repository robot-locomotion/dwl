#include <locomotion/PlanningOfMotionSequence.h>


namespace dwl
{

namespace locomotion
{

PlanningOfMotionSequence::PlanningOfMotionSequence() : motion_planner_(NULL), contact_planner_(NULL),
		robot_(NULL), solver_(NULL), environment_(NULL), computation_time_(std::numeric_limits<double>::max()),
		is_set_solver_(false), is_initialized_planning_(false)// is_added_active_constraint_(false),
		//is_added_inactive_constraint_(false), is_added_cost_(false)
{

}


PlanningOfMotionSequence::~PlanningOfMotionSequence()
{

}


void PlanningOfMotionSequence::reset(robot::Robot* robot, solver::Solver* solver,
									 environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot properties in the %s planner \n" COLOR_RESET, name_.c_str());
	robot_ = robot;

	printf(BLUE "Setting the %s solver in the %s planner\n" COLOR_RESET, solver->getName().c_str(),
			name_.c_str());
	is_set_solver_ = true;
	solver_ = solver;
	solver_->reset(robot, environment);

	printf(BLUE "Setting the environment information in the %s planner\n" COLOR_RESET, name_.c_str());
	environment_ = environment;
}


void PlanningOfMotionSequence::reset(robot::Robot* robot,
									 MotionPlanning* motion_planner,
									 ContactPlanning* contact_planner,
									 environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot properties in the %s planner \n" COLOR_RESET, name_.c_str());
	robot_ = robot;

	printf(BLUE "Setting the environment information in the %s planner\n" COLOR_RESET, name_.c_str());
	environment_ = environment;

	printf(BLUE "Setting the motion planner\n" COLOR_RESET);
	motion_planner_ = motion_planner;
	motion_planner_->reset(robot, environment);

	printf(BLUE "Setting the contact planner\n" COLOR_RESET);
	contact_planner_ = contact_planner;
	contact_planner_->reset(robot, environment);

	is_set_solver_ = true;
}


bool PlanningOfMotionSequence::initPlan()
{
	if (!is_set_solver_) {
		printf(YELLOW "Could not initialized the %s planning because has not been set the solver\n" COLOR_RESET,
				name_.c_str());
		return false;
	}

	if (!init()) {
		printf(RED "Could not initialized the %s planning algorithm\n" COLOR_RESET, name_.c_str());
		return false;
	}
	is_initialized_planning_ = true;

	return is_initialized_planning_;
}


void PlanningOfMotionSequence::resetGoal(Pose goal)
{
	goal_pose_ =  goal;
}


bool PlanningOfMotionSequence::computePlan(Pose robot_state)
{
	if (!is_initialized_planning_) {
		printf(YELLOW "Could not executed the %s planning because has not been initialized\n" COLOR_RESET,
				name_.c_str());
		return false;
	} else if (!is_set_solver_) {
		printf(YELLOW "Could not executed the %s planning because has not been set the solver\n" COLOR_RESET,
				name_.c_str());
		return false;
	}

	bool plan = compute(robot_state);

	return plan;
}


void PlanningOfMotionSequence::setComputationTime(double computation_time)
{
	printf("Setting the allowed computation time of the solver of the coupled planner to %f \n",
			computation_time);
	computation_time_ = computation_time;
}


void PlanningOfMotionSequence::setComputationTime(double computation_time, TypeOfSolver solver)
{
	switch (solver) {
		case BodyPathSolver:
			motion_planner_->setComputationTime(computation_time, true);
			break;
		case BodyPoseSolver:
			motion_planner_->setComputationTime(computation_time, false);
			break;
		case ContactSolver:
			contact_planner_->setComputationTime(computation_time);
			break;
		default:
			printf(YELLOW "Warning: it was not set an allowed computation time because there is not"
					" exist that type of solver \n" COLOR_RESET);
			break;
	}
}


void PlanningOfMotionSequence::setEnvironmentInformation(std::vector<RewardCell> reward_map)
{
	environment_->setEnvironmentInformation(reward_map);
}


void PlanningOfMotionSequence::setEnvironmentInformation(std::vector<Cell> obstacle_map)
{
	environment_->setEnvironmentInformation(obstacle_map);
}


std::vector<Pose> PlanningOfMotionSequence::getBodyPath()
{
	return body_path_;
}


std::vector<Contact> PlanningOfMotionSequence::getContactSequence()
{
	return contacts_sequence_;
}


std::string PlanningOfMotionSequence::getName()
{
	return name_;
}

} //@namespace locomotion
} //@namespace dwl
