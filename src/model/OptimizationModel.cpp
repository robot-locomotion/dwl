#include <model/OptimizationModel.h>


namespace dwl
{

namespace model
{

OptimizationModel::OptimizationModel() : dynamical_system_(NULL),
		constraint_function_(this), cost_function_(this), num_diff_mode_(Eigen::Central),
		state_dimension_(0), constraint_dimension_(0), terminal_constraint_dimension_(0),
		horizon_(1), epsilon_(1E-06), is_added_dynamic_system_(false), is_added_constraint_(false),
		is_added_cost_(false)
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


void OptimizationModel::getStartingPoint(Eigen::Ref<Eigen::VectorXd> full_initial_point)
{
	if (locomotion_solution_.size() == 0) {
		// Getting the initial and ending locomotion state
		LocomotionState starting_locomotion_state = dynamical_system_->getInitialState();
		LocomotionState ending_locomotion_state = dynamical_system_->getTerminalState();

		// Defining splines //TODO for the time being only cubic interpolation is OK
		unsigned int num_joints = dynamical_system_->getFloatingBaseSystem().getJointDoF();
		std::vector<math::CubicSpline> base_spline, joint_spline;
		base_spline.resize(6);
		joint_spline.resize(num_joints);

		// Computing a starting point from interpolation of the starting and ending states
		LocomotionState current_locomotion_state = starting_locomotion_state;
		math::Spline::Point current_point;
		current_locomotion_state.duration = 1. / horizon_;
		for (unsigned int k = 0; k < horizon_; k++) {
			current_locomotion_state.time = k * current_locomotion_state.duration;

			// Base interpolation
			for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
				if (k == 0) {
					// Initialization of the base spline
					math::Spline::Point starting_base(starting_locomotion_state.base_pos(base_idx));
					math::Spline::Point ending_base(ending_locomotion_state.base_pos(base_idx));
					base_spline[base_idx].setBoundary(0, 1, starting_base, ending_base);
				} else {
					// Getting and setting the interpolated point
					base_spline[base_idx].getPoint(current_locomotion_state.time, current_point);
					current_locomotion_state.base_pos(base_idx) = current_point.x;
				}
			}

			// Joint interpolations
			for (unsigned int joint_idx = 0; joint_idx < num_joints; joint_idx++) {
				if (k == 0) {
					// Initialization of the joint spline
					math::Spline::Point starting_joint(starting_locomotion_state.joint_pos(joint_idx));
					math::Spline::Point ending_joint(ending_locomotion_state.joint_pos(joint_idx));
					joint_spline[joint_idx].setBoundary(0, 1, starting_joint, ending_joint);
				} else {
					// Getting and setting the interpolated point
					joint_spline[joint_idx].getPoint(current_locomotion_state.time, current_point);
					current_locomotion_state.joint_pos(joint_idx) = current_point.x;
				}
			}

			// Getting the current state vector
			Eigen::VectorXd current_state;
			dynamical_system_->fromLocomotionState(current_state, current_locomotion_state);

			// Adding the current state vector
			full_initial_point.segment(k * state_dimension_, state_dimension_) = current_state;
		}
	} else {
		// Defining the current locomotion solution as starting point
		unsigned int state_dimension = dynamical_system_->getDimensionOfState();
		for (unsigned int k = 0; k < horizon_; k++) {
			Eigen::VectorXd current_state;
			dynamical_system_->fromLocomotionState(current_state, locomotion_solution_[k]);

			full_initial_point.segment(k * state_dimension, state_dimension) = current_state;
		}
	}
}


void OptimizationModel::evaluateBounds(Eigen::Ref<Eigen::VectorXd> full_state_lower_bound,
									   Eigen::Ref<Eigen::VectorXd> full_state_upper_bound,
									   Eigen::Ref<Eigen::VectorXd> full_constraint_lower_bound,
									   Eigen::Ref<Eigen::VectorXd> full_constraint_upper_bound)
{
	// Getting the lower and upper bound of the locomotion state
	LocomotionState locomotion_lower_bound, locomotion_upper_bound;
	dynamical_system_->getStateBounds(locomotion_lower_bound, locomotion_upper_bound);

	// Converting locomotion state bounds to state bounds
	Eigen::VectorXd state_lower_bound, state_upper_bound;
	dynamical_system_->fromLocomotionState(state_lower_bound, locomotion_lower_bound);
	dynamical_system_->fromLocomotionState(state_upper_bound, locomotion_upper_bound);

	// Getting the lower and upper constraint bounds for a certain time
	unsigned int index = 0;
	unsigned int num_constraints = constraints_.size();
	Eigen::VectorXd constraint_lower_bound(constraint_dimension_),
			constraint_upper_bound(constraint_dimension_);
	for (unsigned int i = 0; i < num_constraints + 1; i++) {
		Eigen::VectorXd lower_bound, upper_bound;
		unsigned int current_bound_dim;
		if (i == 0) {
			// Getting the dynamical constraint bounds
			dynamical_system_->getBounds(lower_bound, upper_bound);

			// Checking the bound dimension
			current_bound_dim = dynamical_system_->getConstraintDimension();
			if (current_bound_dim != lower_bound.size()) {
				printf(RED "FATAL: the bound dimension at %s constraint is not consistent\n"
						COLOR_RESET, dynamical_system_->getName().c_str());
				exit(EXIT_FAILURE);
			}
		} else {
			// Getting the set of constraint bounds
			constraints_[i-1]->getBounds(lower_bound, upper_bound);

			// Checking the bound dimension
			current_bound_dim = constraints_[i-1]->getConstraintDimension();
			if (current_bound_dim != lower_bound.size()) {
				printf(RED "FATAL: the bound dimension at %s constraint is not consistent\n"
						COLOR_RESET, constraints_[i-1]->getName().c_str());
				exit(EXIT_FAILURE);
			}
		}

		// Setting the full constraint vector
		constraint_lower_bound.segment(index, current_bound_dim) = lower_bound;
		constraint_upper_bound.segment(index, current_bound_dim) = upper_bound;

		index += current_bound_dim;
	}

	// Setting the full (state and constraint) lower and upper bounds for the predefined horizon
	for (unsigned int k = 0; k < horizon_; k++) {
		// Setting state bounds
		full_state_lower_bound.segment(k * state_dimension_, state_dimension_) = state_lower_bound;
		full_state_upper_bound.segment(k * state_dimension_, state_dimension_) = state_upper_bound;

		// Setting dynamic system bounds
		full_constraint_lower_bound.segment(k * constraint_dimension_,
											constraint_dimension_) = constraint_lower_bound;
		full_constraint_upper_bound.segment(k * constraint_dimension_,
											constraint_dimension_) = constraint_upper_bound;
	}

	// Computing the terminal bounds in case of full trajectory optimization
	if (dynamical_system_->isFullTrajectoryOptimization()) {
		// Computing the terminal bounds
		Eigen::VectorXd terminal_lower_bound, terminal_upper_bound;
		dynamical_system_->getTerminalBounds(terminal_lower_bound, terminal_upper_bound);

		// Checking the terminal bound dimension
		if (terminal_constraint_dimension_ != terminal_lower_bound.size()) {
			printf(RED "FATAL: the terminal bound dimension is not consistent\n");
			exit(EXIT_FAILURE);
		}

		// Setting the terminal bounds
		full_constraint_lower_bound.segment(horizon_ * constraint_dimension_,
											terminal_constraint_dimension_) = terminal_lower_bound;
		full_constraint_upper_bound.segment(horizon_ * constraint_dimension_,
											terminal_constraint_dimension_) = terminal_upper_bound;
	}
}


void OptimizationModel::evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
											const Eigen::Ref<const Eigen::VectorXd>& decision_var)
{
	if (state_dimension_ != (decision_var.size() / horizon_)) {
		printf(RED "FATAL: the state and decision dimensions are not consistent\n" COLOR_RESET);
		exit(EXIT_FAILURE);
	}

	// Getting the initial conditions of the locomotion state
	LocomotionState locomotion_initial_cond = dynamical_system_->getInitialState();

	// Setting the initial state
	unsigned int num_constraints = constraints_.size();
	for (unsigned int j = 0; j < num_constraints + 1; j++) {
		if (j == 0) // dynamic system constraint
			dynamical_system_->setLastState(locomotion_initial_cond);
		else
			constraints_[j-1]->setLastState(locomotion_initial_cond);
	}

	// Computing the active and inactive constraints for a predefined horizon
	LocomotionState locomotion_state(dynamical_system_->getFloatingBaseSystem().getJointDoF());
	Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dimension_);
	unsigned int index = 0;
	for (unsigned int k = 0; k < horizon_; k++) {
		// Converting the decision variable for a certain time to a robot state
		decision_state = decision_var.segment(k * state_dimension_, state_dimension_);
		dynamical_system_->toLocomotionState(locomotion_state, decision_state);

		// Adding the time information in cases that time is not a decision variable
		if (dynamical_system_->isFixedStepIntegration())
			locomotion_state.duration = dynamical_system_->getFixedStepTime();
		locomotion_state.time += locomotion_state.duration;

		// Computing the constraints for a certain time
		for (unsigned int j = 0; j < num_constraints + 1; j++) {
			Eigen::VectorXd constraint;
			unsigned int current_constraint_dim;
			if (j == 0) {// dynamic system constraint
				// Evaluating the dynamical constraint
				dynamical_system_->compute(constraint, locomotion_state);
				dynamical_system_->setLastState(locomotion_state);

				// Checking the constraint dimension
				current_constraint_dim = dynamical_system_->getConstraintDimension();
				if (current_constraint_dim != constraint.size()) {
					printf(RED "FATAL: the constraint dimension at %s constraint is not consistent\n"
							COLOR_RESET, dynamical_system_->getName().c_str());
					exit(EXIT_FAILURE);
				}
			} else {
				constraints_[j-1]->compute(constraint, locomotion_state);
				constraints_[j-1]->setLastState(locomotion_state);

				// Checking the constraint dimension
				current_constraint_dim = constraints_[j-1]->getConstraintDimension();
				if (current_constraint_dim != constraint.size()) {
					printf(RED "FATAL: the constraint dimension at %s constraint is not consistent\n"
							COLOR_RESET, constraints_[j-1]->getName().c_str());
					exit(EXIT_FAILURE);
				}
			}

			// Setting in the full constraint vector
			full_constraint.segment(index, current_constraint_dim) = constraint;

			index += current_constraint_dim;
		}

		// Computing the terminal constraint in case of full trajectory optimization
		if (dynamical_system_->isFullTrajectoryOptimization()) {
			if (k == horizon_ - 1) {
				Eigen::VectorXd constraint;
				dynamical_system_->computeTerminalConstraint(constraint, locomotion_state);

				// Setting in the full constraint vector
				full_constraint.segment(index, terminal_constraint_dimension_) = constraint;

				index += terminal_constraint_dimension_;
			}
		}
	}

	// Resetting the state buffer
	for (unsigned int j = 0; j < num_constraints + 1; j++) {
		if (j == 0) // dynamic system constraint
			dynamical_system_->resetStateBuffer();
		else
			constraints_[j-1]->resetStateBuffer();
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
	LocomotionState locomotion_state(dynamical_system_->getFloatingBaseSystem().getJointDoF());
	for (unsigned int k = 0; k < horizon_; k++) {
		// Converting the decision variable for a certain time to a robot state
		Eigen::VectorXd decision_state = decision_var.segment(k * state_dimension_, state_dimension_);
		dynamical_system_->toLocomotionState(locomotion_state, decision_state);

		// Adding the time information in cases that time is not a decision variable
		if (dynamical_system_->isFixedStepIntegration())
			locomotion_state.duration = dynamical_system_->getFixedStepTime();
		locomotion_state.time += locomotion_state.duration;

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


std::vector<LocomotionState>& OptimizationModel::evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution)
{
	// Getting the state dimension
	unsigned int state_dim = dynamical_system_->getDimensionOfState();

	// Recording the solution
	locomotion_solution_.clear();
	locomotion_solution_.push_back(dynamical_system_->getInitialState());
	Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dim);
	double current_time = dynamical_system_->getInitialState().time;
	for (unsigned int k = 0; k < horizon_; k++) {
		// Converting the decision variable for a certain time to a robot state
		LocomotionState locomotion_state;
		decision_state = solution.segment(k * state_dim, state_dim);
		dynamical_system_->toLocomotionState(locomotion_state, decision_state);

		// Setting the time information in cases where time is not a decision variable
		if (dynamical_system_->isFixedStepIntegration())
			locomotion_state.duration = dynamical_system_->getFixedStepTime();
		current_time += locomotion_state.duration;
		locomotion_state.time = current_time;


		// Setting the acceleration information in cases where the accelerations are not decision
		// variables
		LocomotionState last_locomotion_state;
		if (k == 0)
			last_locomotion_state = dynamical_system_->getInitialState();
		else {
			Eigen::VectorXd last_decision_state = solution.segment((k - 1) * state_dim, state_dim);
			dynamical_system_->toLocomotionState(last_locomotion_state, last_decision_state);
		}

		if (locomotion_state.base_acc.isZero() && locomotion_state.joint_acc.isZero()) {
			// Computing (estimating) the accelerations
			locomotion_state.base_acc = (locomotion_state.base_vel - last_locomotion_state.base_vel) /
					locomotion_state.duration;
			locomotion_state.joint_acc = (locomotion_state.joint_vel - last_locomotion_state.joint_vel) /
					locomotion_state.duration;
		}


		// Setting the contact information in cases where time is not a decision variable
		// Checking if there are contact information
		if (locomotion_state.contacts.size() == 0)
			locomotion_state.contacts.resize(dynamical_system_->getFloatingBaseSystem().getNumberOfEndEffectors());

		urdf_model::LinkID contacts = dynamical_system_->getFloatingBaseSystem().getEndEffectors();
		rbd::BodySelector contact_names;
		for (urdf_model::LinkID::const_iterator contact_it = contacts.begin();
				contact_it != contacts.end(); contact_it++) {
			// Getting and setting the end-effector names
			std::string name = contact_it->first;
			contact_names.push_back(name);
		}

		if (locomotion_state.contacts[0].position.isZero()) {
			rbd::BodyVector contact_pos;
			dynamical_system_->getKinematics().computeForwardKinematics(contact_pos,
																		locomotion_state.base_pos,
																		locomotion_state.joint_pos,
																		contact_names, rbd::Linear);
			for (unsigned int i = 0; i < contact_names.size(); i++)
				locomotion_state.contacts[i].position = contact_pos.find(contact_names[i])->second;
		}

		if (locomotion_state.contacts[0].velocity.isZero()) {
			rbd::BodyVector contact_vel;
			dynamical_system_->getKinematics().computeVelocity(contact_vel,
													   	   	   locomotion_state.base_pos,
															   locomotion_state.joint_pos,
															   locomotion_state.base_vel,
															   locomotion_state.joint_vel,
															   contact_names, rbd::Linear);
			for (unsigned int i = 0; i < contact_names.size(); i++)
				locomotion_state.contacts[i].velocity = contact_vel.find(contact_names[i])->second;
		}

		if (locomotion_state.contacts[0].acceleration.isZero()) {
			rbd::BodyVector contact_acc;
			dynamical_system_->getKinematics().computeAcceleration(contact_acc,
													   	   	   	   locomotion_state.base_pos,
																   locomotion_state.joint_pos,
																   locomotion_state.base_vel,
																   locomotion_state.joint_vel,
																   locomotion_state.base_acc,
																   locomotion_state.joint_acc,
																   contact_names, rbd::Linear);
			for (unsigned int i = 0; i < contact_names.size(); i++)
				locomotion_state.contacts[i].acceleration = contact_acc.find(contact_names[i])->second;
		}

		// Pushing the current state
		locomotion_solution_.push_back(locomotion_state);



		std::cout << "-------------------------------------" << std::endl;
		std::cout << "x = " << decision_state.transpose() << std::endl;
		std::cout << "time = " << locomotion_state.time << std::endl;
		std::cout << "duration = " << locomotion_state.duration << std::endl;
		std::cout << "base_pos = " << locomotion_state.base_pos.transpose() << std::endl;
		std::cout << "joint_pos = " << locomotion_state.joint_pos.transpose() << std::endl;
		std::cout << "base_vel = " << locomotion_state.base_vel.transpose() << std::endl;
		std::cout << "joint_vel = " << locomotion_state.joint_vel.transpose() << std::endl;
		std::cout << "base_acc = " << locomotion_state.base_acc.transpose() << std::endl;
		std::cout << "joint_acc = " << locomotion_state.joint_acc.transpose() << std::endl;
		std::cout << "joint_eff = " << locomotion_state.joint_eff.transpose() << std::endl;
		std::cout << "contact_pos = " << locomotion_state.contacts[0].position.transpose() << std::endl;
		std::cout << "contact_vel = " << locomotion_state.contacts[0].velocity.transpose() << std::endl;
		std::cout << "contact_acc = " << locomotion_state.contacts[0].acceleration.transpose() << std::endl;
		std::cout << "contact_for = " << locomotion_state.contacts[0].force.transpose() << std::endl;
		std::cout << "-------------------------------------" << std::endl;
	}

	return locomotion_solution_;
}


void OptimizationModel::addDynamicalSystem(DynamicalSystem* dynamical_system)
{
	if (is_added_dynamic_system_) {
		printf(YELLOW "Could not added two dynamical systems\n" COLOR_RESET);
		return;
	}

	printf(GREEN "Adding the %s dynamical system\n" COLOR_RESET, dynamical_system->getName().c_str());
	dynamical_system_ = dynamical_system;
	is_added_dynamic_system_ = true;

	// Reading the state dimension
	state_dimension_ = dynamical_system_->getDimensionOfState();

	// Updating the constraint dimension
	constraint_dimension_ += dynamical_system_->getConstraintDimension();

	// Updating the terminal constraint dimension
	if (dynamical_system_->isFullTrajectoryOptimization())
		terminal_constraint_dimension_ = dynamical_system_->getTerminalConstraintDimension();
}


void OptimizationModel::removeDynamicalSystem()
{
	if (is_added_dynamic_system_) {
		is_added_dynamic_system_ = false;

		// Updating the constraint dimension
		constraint_dimension_ -= dynamical_system_->getConstraintDimension();
	} else
		printf(YELLOW "There was not added a dynamical system\n" COLOR_RESET);
}


void OptimizationModel::addConstraint(Constraint* constraint)
{
	printf(GREEN "Adding the %s constraint\n" COLOR_RESET, constraint->getName().c_str());
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
		printf(YELLOW "Could not removed the %s constraint because has not been added an "
				"constraint\n" COLOR_RESET,	constraint_name.c_str());
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
		printf(YELLOW "Could not removed the %s cost because has not been added an cost\n"
				COLOR_RESET, cost_name.c_str());
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
	return constraint_dimension_ * horizon_ + terminal_constraint_dimension_;
}


const unsigned int& OptimizationModel::getHorizon()
{
	return horizon_;
}

} //@namespace model
} //@namespace dwl
