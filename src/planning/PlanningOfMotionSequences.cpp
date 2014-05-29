#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

namespace planning
{


PlanningOfMotionSequences::PlanningOfMotionSequences() : solver_(NULL), gridmap_(0.04, 0.02), is_added_active_constraint_(false), is_added_inactive_constraint_(false),
		is_added_cost_(false), is_settep_solver_(false), is_initialized_planning_(false)
{

}


PlanningOfMotionSequences::~PlanningOfMotionSequences()
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


void PlanningOfMotionSequences::reset(Solver* solver)
{
	printf(BLUE "Setting the %s solver\n" COLOR_RESET, solver->getName().c_str());
	is_settep_solver_ = true;
	solver_ = solver;
}


void PlanningOfMotionSequences::addConstraint(Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
//		pthread_mutex_lock(&planning_lock_);
		active_constraints_.push_back(constraint);
//		pthread_mutex_unlock(&planning_lock_);

		if (!is_added_active_constraint_)
			is_added_active_constraint_ = true;
	}
	else {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
//		pthread_mutex_lock(&planning_lock_);
		inactive_constraints_.push_back(constraint);
//		pthread_mutex_unlock(&planning_lock_);

		if (!is_added_inactive_constraint_)
			is_added_inactive_constraint_ = true;
	}
}


void PlanningOfMotionSequences::removeConstraint(std::string constraint_name)
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
				if (i < active_constraints_.size()) {
					if (constraint_name == active_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the active %s constraint\n" COLOR_RESET, active_constraints_[i]->getName().c_str());
//						pthread_mutex_lock(&planning_lock_);
						delete active_constraints_.at(i);
						active_constraints_.erase(active_constraints_.begin() + i);
//						pthread_mutex_unlock(&planning_lock_);

						return;
					}
				}
			}

			if (is_added_inactive_constraint_) {
				if (i < inactive_constraints_.size()) {
					if (constraint_name == inactive_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the inactive %s constraint\n" COLOR_RESET, inactive_constraints_[i]->getName().c_str());
//						pthread_mutex_lock(&planning_lock_);
						delete inactive_constraints_.at(i);
						inactive_constraints_.erase(inactive_constraints_.begin() + i);
//						pthread_mutex_unlock(&planning_lock_);

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


void PlanningOfMotionSequences::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());
//	pthread_mutex_lock(&planning_lock_);
	costs_.push_back(cost);
//	pthread_mutex_lock(&planning_lock_);
	is_added_cost_ = true;
}


void PlanningOfMotionSequences::removeCost(std::string cost_name)
{
	if (is_added_cost_) {
		if (costs_.size() == 0)
			printf(YELLOW "Could not removed the %s cost because there is not cost\n" COLOR_RESET, cost_name.c_str());
		else {
			for (int i = 0; i < costs_.size(); i++) {
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


bool PlanningOfMotionSequences::initPlan()
{
	if (is_settep_solver_) {
		if (!init()) {
			printf(RED "Could not initialized the %s planning algorithm\n" COLOR_RESET, name_.c_str());

			return false;
		}
		is_initialized_planning_ = true;
	}
	else {
		printf(YELLOW "Could not initialized the %s planning because has not been setted the solver\n" COLOR_RESET, name_.c_str());

		return false;
	}

	return true;
}


bool PlanningOfMotionSequences::computePlan()
{
	if (is_initialized_planning_) {
		if (is_settep_solver_) {
			if (!compute()) {
				printf(RED "Could not computed the %s planning algorithm\n" COLOR_RESET, name_.c_str());
				return false;
			}
		}
		else {
			printf(YELLOW "Could not executed the %s planning because has not been setted the solver\n" COLOR_RESET, name_.c_str());
			return false;
		}
	}
	else {
		printf(YELLOW "Could not executed the %s planning because has not been initialized\n" COLOR_RESET, name_.c_str());
		return false;
	}

	return true;
}


void PlanningOfMotionSequences::changeGoal(BodyPose goal)
{
	printf(GREEN "Changed the goal state\n" COLOR_RESET);
	//pthread_mutex_lock(&planning_lock_);
	goal_pose_ = goal;
	//pthread_mutex_unlock(&planning_lock_);
}


void PlanningOfMotionSequences::setGridMapResolution(double resolution)
{
	// Setting the resolution of the gridmap instantiation of this class
	gridmap_.setResolution(resolution, true);
	gridmap_.setResolution(resolution, false);

	// Setting the resolution of the gridmap cost class
	for (int i = 0; i < costs_.size(); i++) {
		if (costs_[i]->isCostMap()) {
			costs_[i]->setGridMapResolution(resolution);

			break;
		}
	}
}


std::string PlanningOfMotionSequences::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
