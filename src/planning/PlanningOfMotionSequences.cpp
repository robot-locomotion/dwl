#include <planning/PlanningOfMotionSequences.h>


namespace dwl
{

namespace planning
{


PlanningOfMotionSequences::PlanningOfMotionSequences() : solver_(NULL)
{

}


void PlanningOfMotionSequences::reset(Solver* solver)
{
	printf(BLUE "Setting the %s solver\n" COLOR_RESET, solver->getName().c_str());

	solver_ = solver;
}


void PlanningOfMotionSequences::addConstraint(Constraint* constraint)
{
	solver_->addConstraint(constraint);
}


void PlanningOfMotionSequences::removeConstraint(std::string constraint_name)
{
	solver_->removeConstraint(constraint_name);
}


void PlanningOfMotionSequences::addCost(Cost* cost)
{
	solver_->addCost(cost);
}


void PlanningOfMotionSequences::removeCost(std::string cost_name)
{
	solver_->removeCost(cost_name);
}


std::string dwl::planning::PlanningOfMotionSequences::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
