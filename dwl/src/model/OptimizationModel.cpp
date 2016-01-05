#include <dwl/model/OptimizationModel.h>


namespace dwl
{

namespace model
{

OptimizationModel::OptimizationModel() : constraint_function_(this), cost_function_(this),
		num_diff_mode_(Eigen::Central),	state_dimension_(0), constraint_dimension_(0),
		terminal_constraint_dimension_(0), horizon_(1), epsilon_(1E-06)
{

}


OptimizationModel::~OptimizationModel()
{

}


void OptimizationModel::evaluateConstraintJacobian(Eigen::MatrixXd& jacobian,
												   const Eigen::VectorXd& decision_var)
{
	switch (num_diff_mode_) {
		case Eigen::Forward: {
			Eigen::NumericalDiff<ConstraintFunction,Eigen::Forward> num_diff(constraint_function_, epsilon_);
			num_diff.df(decision_var, jacobian);
			break;
		} case Eigen::Central: {
			Eigen::NumericalDiff<ConstraintFunction,Eigen::Central> num_diff(constraint_function_, epsilon_);
			num_diff.df(decision_var, jacobian);
			break;
		} default: {
			Eigen::NumericalDiff<ConstraintFunction,Eigen::Central> num_diff(constraint_function_, epsilon_);
			num_diff.df(decision_var, jacobian);
			break;
		}
	}
}


void OptimizationModel::evaluateCostGradient(Eigen::MatrixXd& gradient,
											 const Eigen::VectorXd& decision_var)
{
	switch (num_diff_mode_) {
		case Eigen::Forward: {
			Eigen::NumericalDiff<CostFunction,Eigen::Forward> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, gradient);
			break;
		} case Eigen::Central: {
			Eigen::NumericalDiff<CostFunction,Eigen::Central> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, gradient);
			break;
		} default: {
			Eigen::NumericalDiff<CostFunction,Eigen::Central> num_diff(cost_function_, epsilon_);
			num_diff.df(decision_var, gradient);
			break;
		}
	}
}


void OptimizationModel::configureNumericalDifferentiation(enum Eigen::NumericalDiffMode mode,
														  double epsilon)
{
	num_diff_mode_ = mode;
	epsilon_ = epsilon;
}


unsigned int OptimizationModel::getDimensionOfState()
{
	return state_dimension_ * horizon_;
}


unsigned int OptimizationModel::getDimensionOfConstraints()
{
	return constraint_dimension_ * horizon_ + terminal_constraint_dimension_;
}

} //@namespace model
} //@namespace dwl
