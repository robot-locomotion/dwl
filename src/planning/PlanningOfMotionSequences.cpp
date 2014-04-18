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

void PlanningOfMotionSequences::addCost(Cost* cost)
{
	solver_->addCost(cost);
}


} //@namespace planning

} //@namespace dwl
