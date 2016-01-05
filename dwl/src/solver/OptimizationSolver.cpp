#include <dwl/solver/OptimizationSolver.h>


namespace dwl
{

namespace solver
{

OptimizationSolver::OptimizationSolver() : model_(NULL)
{

}


OptimizationSolver::~OptimizationSolver()
{

}


void OptimizationSolver::setOptimizationModel(model::OptimizationModel* model)
{
	model_ = model;
}


model::OptimizationModel* OptimizationSolver::getOptimizationModel()
{
	return model_;
}


const WholeBodyTrajectory& OptimizationSolver::getWholeBodyTrajectory()
{
	return locomotion_trajectory_;
}


std::string OptimizationSolver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
