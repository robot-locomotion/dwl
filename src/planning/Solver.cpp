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

void Solver::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());
	costs_.push_back(cost);
}


std::string dwl::planning::Solver::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
