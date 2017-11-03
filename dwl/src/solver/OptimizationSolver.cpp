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


void OptimizationSolver::setFromConfigFile(std::string filename)
{

}


void OptimizationSolver::setOptimizationModel(model::OptimizationModel* model)
{
	// This routine allows us to understand if the Jacobian and Hessian were implemented
	// It's important to evaluate if we have to computed them numerically
	double *values = NULL;
	int *rind = NULL, *cind = NULL;
	const double *x = NULL, *lambda = NULL;
	double obj_factor = 0.;
	int n = model->getDimensionOfState();
	int m = model->getDimensionOfConstraints();
	int nnz_jac = model->getNumberOfNonzeroJacobian();
	int nnz_hess = model->getNumberOfNonzeroHessian();
	model->evaluateCostGradient(values, n, x, n);
	model->evaluateConstraintJacobian(values, nnz_jac,
									  rind, nnz_jac,
									  cind, nnz_jac,
									  x, n, false);
	model->evaluateLagrangianHessian(values, nnz_hess,
									 rind, nnz_hess,
									 cind, nnz_hess,
									 obj_factor,
									 lambda, m, x, n, false);

	model_ = model;
}


bool OptimizationSolver::init()
{
	return true;
}


bool OptimizationSolver::compute(double computation_time)
{
	printf(RED "Error: you have to define a solver.\n" COLOR_RESET);
	return true;
}


model::OptimizationModel* OptimizationSolver::getOptimizationModel()
{
	return model_;
}


const Eigen::VectorXd& OptimizationSolver::getSolution()
{
	return solution_;
}


std::string OptimizationSolver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
