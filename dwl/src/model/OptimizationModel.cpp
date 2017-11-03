#include <dwl/model/OptimizationModel.h>


namespace dwl
{

namespace model
{

OptimizationModel::OptimizationModel() : solution_(NULL), state_dimension_(0),
		constraint_dimension_(0), nonzero_jacobian_(0), nonzero_hessian_(0), gradient_(true),
		jacobian_(true), hessian_(true), first_time_(true), cost_function_(this),
		num_diff_mode_(Eigen::Central),	epsilon_(1E-06)
{

}


OptimizationModel::~OptimizationModel()
{

}

void OptimizationModel::init(bool only_soft_constraints)
{

}


void OptimizationModel::setDimensionOfState(unsigned int dimension)
{
	state_dimension_ = dimension;
}


void OptimizationModel::setDimensionOfConstraints(unsigned int dimension)
{
	constraint_dimension_ = dimension;
}


void OptimizationModel::setNumberOfNonzeroJacobian(unsigned int nonzero)
{
	nonzero_jacobian_ = nonzero;
}


void OptimizationModel::setNumberOfNonzeroHessian(unsigned int nonzero)
{
	nonzero_hessian_ = nonzero;
}


void OptimizationModel::getStartingPoint(double* decision, int decision_dim)
{
	printf(YELLOW "Warning: there is not defined the warm point, default in the origin\n" COLOR_RESET);

	// Defined the warm point in the origin
	Eigen::Map<Eigen::VectorXd> warm_point(decision, decision_dim);
	warm_point = Eigen::VectorXd::Zero(decision_dim);
}


void OptimizationModel::evaluateBounds(double* decision_lbound, int decision_dim1,
									   double* decision_ubound, int decision_dim2,
									   double* constraint_lbound, int constraint_dim1,
									   double* constraint_ubound, int constraint_dim2)
{
	if (first_time_) {
		printf(YELLOW "Warning: there is not defined the bounds, default as boundless\n" COLOR_RESET);
		first_time_ = false;
	}

	// Mapping the raw buffer to Eigen vectors
	Eigen::Map<Eigen::VectorXd> x_lb(decision_lbound, decision_dim1);
	Eigen::Map<Eigen::VectorXd> x_ub(decision_ubound, decision_dim2);
	Eigen::Map<Eigen::VectorXd> g_lb(constraint_lbound, constraint_dim1);
	Eigen::Map<Eigen::VectorXd> g_ub(constraint_ubound, constraint_dim2);

	// Defined the problem boundless
	x_lb = -NO_BOUND * Eigen::VectorXd::Ones(decision_dim1);
	x_ub = NO_BOUND * Eigen::VectorXd::Ones(decision_dim2);
	g_lb = -NO_BOUND * Eigen::VectorXd::Ones(constraint_dim1);
	g_ub = NO_BOUND * Eigen::VectorXd::Ones(constraint_dim2);
}


void OptimizationModel::evaluateCosts(double& cost,
									  const double* decision, int decision_dim)
{
	printf(YELLOW "Warning: No cost function is implemented\n" COLOR_RESET);
	cost = 0.;
}


void OptimizationModel::evaluateCostGradient(double* gradient, int grad_dim,
											 const double* decision, int decision_dim)
{
	// Indicates that the gradient is computed using numerical differenciation
	gradient_ = false;

	// Eigen interfacing to raw buffers
	Eigen::Map<Eigen::VectorXd> full_gradient(gradient, grad_dim);
	const Eigen::Map<const Eigen::VectorXd> decision_var(decision, decision_dim);

	Eigen::MatrixXd grad(1, decision_dim);

	switch (num_diff_mode_) {
		case Eigen::Forward: {
			Eigen::NumericalDiff<CostFunction,Eigen::Forward> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, grad);
			break;
		} case Eigen::Central: {
			Eigen::NumericalDiff<CostFunction,Eigen::Central> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, grad);
			break;
		} default: {
			Eigen::NumericalDiff<CostFunction,Eigen::Central> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, grad);
			break;
		}
	}

	full_gradient = grad;
}


void OptimizationModel::evaluateConstraints(double* constraint, int constraint_dim,
								 	 		const double* decision, int decision_dim)
{

}


void OptimizationModel::evaluateConstraintJacobian(double* jacobian_values, int nonzero_dim1,
												   int* row_entries, int nonzero_dim2,
												   int* col_entries, int nonzero_dim3,
												   const double* decision, int decision_dim, bool flag)
{
	jacobian_ = false;
}


void OptimizationModel::evaluateLagrangianHessian(double* hessian_values, int nonzero_dim1,
												  int* row_entries, int nonzero_dim2,
												  int* col_entries, int nonzero_dim3,
												  double obj_factor,
												  const double* lagrange, int constraint_dim,
												  const double* decision, int decision_dim,
												  bool flag)
{
	hessian_ = false;
}



unsigned int OptimizationModel::getDimensionOfState()
{
	return state_dimension_;
}


unsigned int OptimizationModel::getDimensionOfConstraints()
{
	return constraint_dimension_;
}


unsigned int OptimizationModel::getNumberOfNonzeroJacobian()
{
	return nonzero_jacobian_;
}


unsigned int OptimizationModel::getNumberOfNonzeroHessian()
{
	return nonzero_hessian_;
}


bool OptimizationModel::isCostGradientImplemented()
{
	return gradient_;
}


bool OptimizationModel::isConstraintJacobianImplemented()
{
	return jacobian_;
}

		
bool OptimizationModel::isLagrangianHessianImplemented()
{
	return hessian_;
}

} //@namespace model
} //@namespace dwl