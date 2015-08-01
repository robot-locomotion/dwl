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


std::string OptimizationSolver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
