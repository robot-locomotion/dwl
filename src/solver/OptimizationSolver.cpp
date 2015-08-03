#include <solver/OptimizationSolver.h>


namespace dwl
{

namespace solver
{

OptimizationSolver::OptimizationSolver()
{

}


OptimizationSolver::~OptimizationSolver()
{

}


model::OptimizationModel& OptimizationSolver::getOptimizationModel()
{
	return model_;
}


std::vector<LocomotionState>& OptimizationSolver::getWholeBodyTrajectory()
{
	return locomotion_trajectory_;
}


std::string OptimizationSolver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
