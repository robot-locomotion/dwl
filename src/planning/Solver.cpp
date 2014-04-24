#include <planning/Solver.h>


namespace dwl
{

namespace planning
{


void Solver::addConstraint(Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		active_constraints_.push_back(constraint);
	}
	else {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		inactive_constraints_.push_back(constraint);
	}
}


void Solver::removeConstraint(std::string constraint_name)
{
	int max_num_constraints = (active_constraints_.size() > inactive_constraints_.size()) ? active_constraints_.size() : inactive_constraints_.size();
	for (int i = 0; i < max_num_constraints; i++) {
		if (i < active_constraints_.size()) {
			if (constraint_name == active_constraints_[i]->getName().c_str()) {
				printf(GREEN "Removing the active %s constraint\n" COLOR_RESET, active_constraints_[i]->getName().c_str());
				active_constraints_.erase(active_constraints_.begin() + i);
				return;
			}
		}
		if (i < inactive_constraints_.size()) {
			if (constraint_name == inactive_constraints_[i]->getName().c_str()) {
				printf(GREEN "Removing the inactive %s constraint\n" COLOR_RESET, inactive_constraints_[i]->getName().c_str());
				inactive_constraints_.erase(inactive_constraints_.begin() + i);
				return;
			}
		}
		else if (i == max_num_constraints - 1) {
			printf(YELLOW "Could not remove the %s constraint\n" COLOR_RESET, constraint_name.c_str());
		}
	}
}


void Solver::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());
	costs_.push_back(cost);
}


void Solver::removeCost(std::string cost_name)
{
	for (int i = 0; i < costs_.size(); i++) {
		if (cost_name == costs_[i]->getName().c_str()) {
			printf(GREEN "Removing the %s cost\n" COLOR_RESET, costs_[i]->getName().c_str());
			costs_.erase(costs_.begin() + i);

			return;
		}
		else if (i == costs_.size() - 1) {
			printf(YELLOW "Could not remove the %s cost\n" COLOR_RESET, cost_name.c_str());
		}
	}
}


std::string dwl::planning::Solver::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
