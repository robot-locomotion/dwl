#include <model/OptimizationModel.h>


namespace dwl
{

namespace model
{

OptimizationModel::OptimizationModel() : dynamical_system_(NULL),
		constraint_function_(this), cost_function_(this), num_diff_mode_(Eigen::Central),
		state_dimension_(0), constraint_dimension_(0), horizon_(1), epsilon_(1E-06),
		is_added_dynamic_system_(false), is_added_constraint_(false), is_added_cost_(false)
{

}


OptimizationModel::~OptimizationModel()
{
	delete dynamical_system_;

	typedef std::vector<Constraint*>::iterator ConstraintItr;
	typedef std::vector<Cost*>::iterator CostItr;
	if (is_added_constraint_) {
		for (ConstraintItr i = constraints_.begin(); i != constraints_.end(); i++)
			delete *i;
	}

	if (is_added_cost_) {
		for (CostItr i = costs_.begin(); i != costs_.end(); i++)
			delete *i;
	}
}


void OptimizationModel::evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
											const Eigen::Ref<const Eigen::VectorXd>& decision_var)
{
	if (state_dimension_ != (decision_var.size() / horizon_)) {
		printf(RED "FATAL: the state and decision dimensions are not consistent\n" COLOR_RESET);
		exit(EXIT_FAILURE);
	}

	// Setting the initial state
	LocomotionState locomotion_state;
	Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dimension_);
	dynamical_system_->toLocomotionState(locomotion_state, decision_state);
	unsigned int num_constraints = constraints_.size();
	for (unsigned int j = 0; j < num_constraints + 1; j++) {
		if (j == 0) // dynamic system constraint
			dynamical_system_->setLastState(locomotion_state);
		else
			constraints_[j-1]->setLastState(locomotion_state);
	}

	// Computing the active and inactive constraints for a predefined horizon
	unsigned int index = 0;
	for (unsigned int i = 0; i < horizon_; i++) {
		// Converting the decision variable for a certain time to a robot state
		decision_state = decision_var.segment(i * state_dimension_, state_dimension_);
		dynamical_system_->toLocomotionState(locomotion_state, decision_state);

		// Computing the constraints for a certain time
		Eigen::VectorXd constraint;
		for (unsigned int j = 0; j < num_constraints + 1; j++) {
			if (j == 0) {// dynamic system constraint
				dynamical_system_->compute(constraint, locomotion_state);
				dynamical_system_->setLastState(locomotion_state);
			} else {
				constraints_[j-1]->compute(constraint, locomotion_state);
				constraints_[j-1]->setLastState(locomotion_state);
			}

			unsigned int constraint_size = constraint.size();
			full_constraint.segment(index, constraint_size) = constraint;

			index += constraint_size;
		}
	}
}


void OptimizationModel::evaluateCosts(double& cost,
									  const Eigen::Ref<const Eigen::VectorXd>& decision_var)
{
	if (state_dimension_ != (decision_var.size() / horizon_)) {
		printf(RED "FATAL: the state and decision dimensions are not consistent\n" COLOR_RESET);
		exit(EXIT_FAILURE);
	}

	// Initializing the cost value
	cost = 0;

	// Computing the cost for predefined horizon
	LocomotionState locomotion_state;
	for (unsigned int i = 0; i < horizon_; i++) {
		// Converting the decision variable for a certain time to a robot state
		Eigen::VectorXd decision_state = decision_var.segment(i * state_dimension_, state_dimension_);
		dynamical_system_->toLocomotionState(locomotion_state, decision_state);

		// Computing the function cost for a certain time
		double simple_cost;
		unsigned int num_cost_functions = costs_.size();
		for (unsigned int j = 0; j < num_cost_functions; j++) {
			costs_[j]->compute(simple_cost, locomotion_state);
			cost += simple_cost;
		}
	}
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


void OptimizationModel::addDynamicalSystem(DynamicalSystem* dynamical_system)
{
	if (is_added_dynamic_system_) {
		printf(YELLOW "Could not added two dynamical systems\n" COLOR_RESET);
		return;
	}

	printf(GREEN "Adding the %s dynamical system\n" COLOR_RESET, dynamical_system->getName().c_str());
	dynamical_system_ = dynamical_system;

	// Reading the state dimension
	state_dimension_ = getDynamicalSystem()->getDimensionOfState();

	// Updating the constraint dimension
	constraint_dimension_ += dynamical_system->getConstraintDimension();
}


void OptimizationModel::removeDynamicalSystem()
{
	if (is_added_dynamic_system_) {
		dynamical_system_ = NULL;

		// Updating the constraint dimension
		constraint_dimension_ -= dynamical_system_->getConstraintDimension();
	} else
		printf(YELLOW "There was not added a dynamical system\n" COLOR_RESET);
}


void OptimizationModel::addConstraint(Constraint* constraint)
{
	printf(GREEN "Adding the %s dynamical system constraint\n" COLOR_RESET, constraint->getName().c_str());
	constraints_.push_back(constraint);

	if (!is_added_constraint_)
		is_added_constraint_ = true;

	// Updating the constraint dimension
	constraint_dimension_ += constraint->getConstraintDimension();
}


void OptimizationModel::removeConstraint(std::string constraint_name)
{
	if (is_added_constraint_) {
		unsigned int num_constraints = constraints_.size();
		for (unsigned int i = 0; i < num_constraints; i++) {
			if (constraint_name == constraints_[i]->getName().c_str()) {
				printf(GREEN "Removing the %s constraint\n" COLOR_RESET, constraints_[i]->getName().c_str());

				// Updating the constraint dimension
				constraint_dimension_ -= constraints_[i]->getConstraintDimension();

				// Deleting the constraint
				delete constraints_.at(i);
				constraints_.erase(constraints_.begin() + i);

				return;
			}
		}
	} else {
		printf(YELLOW "Could not removed the %s constraint because has not been added an constraint\n" COLOR_RESET,
				constraint_name.c_str());
		return;
	}
}


void OptimizationModel::addCost(Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());

	costs_.push_back(cost);
	is_added_cost_ = true;
}


void OptimizationModel::removeCost(std::string cost_name)
{
	if (is_added_cost_) {
		if (costs_.size() == 0)
			printf(YELLOW "Could not removed the %s cost because there is not cost\n" COLOR_RESET,
					cost_name.c_str());
		else {
			unsigned int costs_size = costs_.size();
			for (unsigned int i = 0; i < costs_size; i++) {
				if (cost_name == costs_[i]->getName().c_str()) {
					printf(GREEN "Removing the %s cost\n" COLOR_RESET, costs_[i]->getName().c_str());

					// Deleting the cost
					delete costs_.at(i);
					costs_.erase(costs_.begin() + i);

					return;
				}
				else if (i == costs_.size() - 1) {
					printf(YELLOW "Could not removed the %s cost\n" COLOR_RESET, cost_name.c_str());
				}
			}
		}
	}
	else
		printf(YELLOW "Could not removed the %s cost because has not been added an cost\n" COLOR_RESET,
				cost_name.c_str());
}


void OptimizationModel::setHorizon(unsigned int horizon)
{
	if (horizon == 0)
		horizon_ = 1;
	else
		horizon_ = horizon;
}


void OptimizationModel::configureNumericalDifferentiation(enum Eigen::NumericalDiffMode mode,
														  double epsilon)
{
	num_diff_mode_ = mode;
	epsilon_ = epsilon;
}


DynamicalSystem* OptimizationModel::getDynamicalSystem()
{
	return dynamical_system_;
}


std::vector<Constraint*> OptimizationModel::getConstraints()
{
	return constraints_;
}


std::vector<Cost*> OptimizationModel::getCosts()
{
	return costs_;
}


unsigned int OptimizationModel::getDimensionOfState()
{
	return state_dimension_ * horizon_;
}


unsigned int OptimizationModel::getDimensionOfConstraints()
{
	return constraint_dimension_ * horizon_;
}


unsigned int OptimizationModel::getHorizon()
{
	return horizon_;
}

} //@namespace model
} //@namespace dwl
