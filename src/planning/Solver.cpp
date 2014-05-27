#include <planning/Solver.h>


namespace dwl
{

namespace planning
{


Solver::Solver() : is_graph_searching_algorithm_(false), is_added_active_constraint_(false), is_added_inactive_constraint_(false),
		is_added_cost_(false), total_cost_(0)
{

}


Solver::~Solver()
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


void Solver::addConstraint(Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
//		pthread_mutex_lock(&solver_lock_);
		active_constraints_.push_back(constraint);
//		pthread_mutex_unlock(&solver_lock_);

		if (!is_added_active_constraint_)
			is_added_active_constraint_ = true;
	}
	else {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
//		pthread_mutex_lock(&solver_lock_);
		inactive_constraints_.push_back(constraint);
//		pthread_mutex_unlock(&solver_lock_);

		if (!is_added_inactive_constraint_)
			is_added_inactive_constraint_ = true;
	}
}


void Solver::removeConstraint(std::string constraint_name)
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
//						pthread_mutex_lock(&solver_lock_);
						delete active_constraints_.at(i);
						active_constraints_.erase(active_constraints_.begin() + i);
//						pthread_mutex_unlock(&solver_lock_);

						return;
					}
				}
			}

			if (is_added_inactive_constraint_) {
				if (i < inactive_constraints_.size()) {
					if (constraint_name == inactive_constraints_[i]->getName().c_str()) {
						printf(GREEN "Removing the inactive %s constraint\n" COLOR_RESET, inactive_constraints_[i]->getName().c_str());
//						pthread_mutex_lock(&solver_lock_);
						delete inactive_constraints_.at(i);
						inactive_constraints_.erase(inactive_constraints_.begin() + i);
//						pthread_mutex_unlock(&solver_lock_);

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


void Solver::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());
//	pthread_mutex_lock(&solver_lock_);
	costs_.push_back(cost);
//	pthread_mutex_lock(&solver_lock_);
	is_added_cost_ = true;
}


void Solver::removeCost(std::string cost_name)
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


std::list<Vertex> Solver::getShortestPath(Vertex target)
{
	std::list<Vertex> path;
	if (is_graph_searching_algorithm_) {
		PreviousVertex::iterator prev;
		Vertex vertex = target;
		path.push_front(vertex);
		while((prev = previous_.find(vertex)) != previous_.end()) {
			vertex = prev->second;
			path.push_front(vertex);
		}
	} else {
		printf(YELLOW "Could not get the shortest path because the %s is not a graph-searchin algorithm\n" COLOR_RESET, name_.c_str());
	}

	return path;
}


double Solver::getMinimumCost()
{
	return total_cost_;
}


std::string dwl::planning::Solver::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
