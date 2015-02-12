#include <locomotion/PlanningOfMotionSequence.h>


namespace dwl
{

namespace locomotion
{

PlanningOfMotionSequence::PlanningOfMotionSequence() : motion_planner_(NULL), contact_planner_(NULL), robot_(NULL), solver_(NULL),
		environment_(NULL), computation_time_(std::numeric_limits<double>::max()), is_set_solver_(false), is_initialized_planning_(false),
		is_added_active_constraint_(false), is_added_inactive_constraint_(false), is_added_cost_(false)
{

}


PlanningOfMotionSequence::~PlanningOfMotionSequence()
{
	if (is_added_active_constraint_) {
		for (std::vector<Constraint*>::iterator i = active_constraints_.begin(); i != active_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_inactive_constraint_) {
		for (std::vector<Constraint*>::iterator i = inactive_constraints_.begin(); i != inactive_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_cost_) {
		for (std::vector<Cost*>::iterator i = costs_.begin(); i != costs_.end(); i++)
			delete *i;
	}
}


void PlanningOfMotionSequence::reset(robot::Robot* robot, Solver* solver, environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot properties in the %s planner \n" COLOR_RESET, name_.c_str());
	robot_ = robot;

	printf(BLUE "Setting the %s solver in the %s planner\n" COLOR_RESET, solver->getName().c_str(), name_.c_str());
	is_set_solver_ = true;
	solver_ = solver;
	solver_->reset(robot, environment);

	printf(BLUE "Setting the environment information in the %s planner\n" COLOR_RESET, name_.c_str());
	environment_ = environment;
}


void PlanningOfMotionSequence::reset(robot::Robot* robot, MotionPlanning* motion_planner, ContactPlanning* contact_planner,
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


void PlanningOfMotionSequence::addConstraint(Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		active_constraints_.push_back(constraint);

		if (!is_added_active_constraint_)
			is_added_active_constraint_ = true;
	}
	else {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		inactive_constraints_.push_back(constraint);

		if (!is_added_inactive_constraint_)
			is_added_inactive_constraint_ = true;
	}
}


void PlanningOfMotionSequence::removeConstraint(std::string constraint_name)
{
	int max_num_constraints;
	if (is_added_active_constraint_ & is_added_inactive_constraint_)
		max_num_constraints = (active_constraints_.size() > inactive_constraints_.size()) ? active_constraints_.size() : inactive_constraints_.size();
	else if (is_added_active_constraint_)
		max_num_constraints = active_constraints_.size();
	else if (is_added_inactive_constraint_)
		max_num_constraints = inactive_constraints_.size();
	else {
		printf(YELLOW "Could not removed the %s constraint because has not been added an constraint\n" COLOR_RESET, constraint_name.c_str());

		return;
	}

	if (max_num_constraints == 0)
		printf(YELLOW "Could not removed the %s constraint because there is not constraints\n" COLOR_RESET, constraint_name.c_str());
	else {
		for (int i = 0; i < max_num_constraints; i++) {
			if (is_added_active_constraint_) {
				if (i < (int) active_constraints_.size()) {
					if (constraint_name == active_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the active %s constraint\n" COLOR_RESET, active_constraints_[i]->getName().c_str());
						delete active_constraints_.at(i);
						active_constraints_.erase(active_constraints_.begin() + i);

						return;
					}
				}
			}

			if (is_added_inactive_constraint_) {
				if (i < (int) inactive_constraints_.size()) {
					if (constraint_name == inactive_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the inactive %s constraint\n" COLOR_RESET, inactive_constraints_[i]->getName().c_str());
						delete inactive_constraints_.at(i);
						inactive_constraints_.erase(inactive_constraints_.begin() + i);

						return;
					}
				}
			}

			if (i == max_num_constraints - 1) {
				printf(YELLOW "Could not removed the %s constraint\n" COLOR_RESET, constraint_name.c_str());
			}
		}
	}
}


void PlanningOfMotionSequence::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());

	costs_.push_back(cost);
	is_added_cost_ = true;
}


void PlanningOfMotionSequence::removeCost(std::string cost_name)
{
	if (is_added_cost_) {
		if (costs_.size() == 0)
			printf(YELLOW "Could not removed the %s cost because there is not cost\n" COLOR_RESET, cost_name.c_str());
		else {
			unsigned int costs_size = costs_.size();
			for (unsigned int i = 0; i < costs_size; i++) {
				if (cost_name == costs_[i]->getName().c_str()) {
					printf(GREEN "Removing the %s cost\n" COLOR_RESET, costs_[i]->getName().c_str());
					delete costs_.at(i);
					costs_.erase(costs_.begin() + i);

					return;
				}
				else if (i == costs_.size() - 1) {
					printf(YELLOW "Could not removed the %s cost\n" COLOR_RESET, cost_name.c_str());
				}
			}
		}
	}
	else
		printf(YELLOW "Could not removed the %s cost because has not been added an cost\n" COLOR_RESET, cost_name.c_str());
}


bool PlanningOfMotionSequence::initPlan()
{
	if (!is_set_solver_) {
		printf(YELLOW "Could not initialized the %s planning because has not been set the solver\n" COLOR_RESET, name_.c_str());
		return false;
	}

	if (!init()) {
		printf(RED "Could not initialized the %s planning algorithm\n" COLOR_RESET, name_.c_str());
		return false;
	}
	is_initialized_planning_ = true;

	return is_initialized_planning_;
}


bool PlanningOfMotionSequence::computePlan(Pose robot_state)
{
	if (!is_initialized_planning_) {
		printf(YELLOW "Could not executed the %s planning because has not been initialized\n" COLOR_RESET, name_.c_str());
		return false;
	} else if (!is_set_solver_) {
		printf(YELLOW "Could not executed the %s planning because has not been set the solver\n" COLOR_RESET, name_.c_str());
		return false;
	}

	bool plan = compute(robot_state);

	return plan;
}


void PlanningOfMotionSequence::setComputationTime(double computation_time)
{
	printf("Setting the allowed computation time of the solver of the coupled planner to %f \n", computation_time);
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
			printf(YELLOW "Warning: it was not set an allowed computation time because there is not exist that type of solver \n" COLOR_RESET);
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
