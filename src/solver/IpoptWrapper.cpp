#include <solver/IpoptWrapper.h>


namespace dwl
{

namespace solver
{

IpoptWrapper::IpoptWrapper() : opt_model_(NULL)
{

}


IpoptWrapper::~IpoptWrapper()
{

}


void IpoptWrapper::reset(model::OptimizationModel* model)
{
	opt_model_ = model;
}


bool IpoptWrapper::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
								Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	// Getting the dimension of decision variables for every knots
	n = opt_model_->getDimensionOfState();

	// Getting the dimension of constraints for every knots
	m = opt_model_->getDimensionOfConstraints();

    // Assuming that Jacobian and Hessian are dense
    nnz_jac_g = n * m;
    nnz_h_lag = n * (n + 1) * 0.5;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

	return true;
}


bool IpoptWrapper::get_bounds_info(Index n, Number* x_l, Number* x_u,
								   Index m, Number* g_l, Number* g_u)
{
	// Eigen interfacing to raw buffers
	Eigen::Map<Eigen::VectorXd> full_state_lower_bound(x_l, n);
	Eigen::Map<Eigen::VectorXd> full_state_upper_bound(x_u, n);
	Eigen::Map<Eigen::VectorXd> full_constraint_lower_bound(g_l, m);
	Eigen::Map<Eigen::VectorXd> full_constraint_upper_bound(g_u, m);

	// Getting the lower and upper bound of the locomotion state
	LocomotionState locomotion_lower_bound, locomotion_upper_bound;
	opt_model_->getDynamicalSystem()->getStateBounds(locomotion_lower_bound, locomotion_upper_bound);

	// Converting locomotion state bounds to state bounds
	Eigen::VectorXd state_lower_bound, state_upper_bound;
	opt_model_->getDynamicalSystem()->fromLocomotionState(state_lower_bound, locomotion_lower_bound);
	opt_model_->getDynamicalSystem()->fromLocomotionState(state_upper_bound, locomotion_upper_bound);

	// Getting the lower and upper constraint bounds for a certain time
	unsigned int index = 0;
	unsigned int num_constraints = opt_model_->getConstraints().size();
	Eigen::VectorXd constraint_lower_bound(m), constraint_upper_bound(m);
	for (unsigned int i = 0; i < num_constraints + 1; i++) {
		Eigen::VectorXd lower_bound, upper_bound;
		if (i == 0)
			opt_model_->getDynamicalSystem()->getBounds(lower_bound, upper_bound);
		else
			opt_model_->getConstraints()[i]->getBounds(lower_bound, upper_bound);

		unsigned int bound_size = lower_bound.size();
		constraint_lower_bound.segment(index, bound_size) = lower_bound;
		constraint_upper_bound.segment(index, bound_size) = upper_bound;

		index += bound_size;
	}

	// Setting the full (state and constraint) lower and upper bounds for the predefined horizon
	unsigned int state_dim = n / opt_model_->getHorizon();
	unsigned int constraint_dim = m / opt_model_->getHorizon();
	for (unsigned int i = 0; i < opt_model_->getHorizon(); i++) {
		// Setting state bounds
		full_state_lower_bound.segment(i * state_dim, state_dim) = state_lower_bound;
		full_state_upper_bound.segment(i * state_dim, state_dim) = state_upper_bound;

		// Setting dynamic system bounds
		full_constraint_lower_bound.segment(i * constraint_dim, constraint_dim) = constraint_lower_bound;
		full_constraint_upper_bound.segment(i * constraint_dim, constraint_dim) = constraint_upper_bound;
	}

	return true;
}


bool IpoptWrapper::get_starting_point(Index n, bool init_x, Number* x,
									  bool init_z, Number* z_L, Number* z_U,
									  Index m, bool init_lambda, Number* lambda)
{
	// Eigen interfacing to raw buffers
	Eigen::Map<Eigen::VectorXd> full_initial_state(x, n);

	// Getting the initial locomotion state
	LocomotionState starting_locomotion_state;
	opt_model_->getDynamicalSystem()->getStartingState(starting_locomotion_state);

	// Getting the initial state vector
	Eigen::VectorXd starting_state;
	opt_model_->getDynamicalSystem()->fromLocomotionState(starting_state, starting_locomotion_state);

	// Setting the full starting state for the predefined horizon
	unsigned int state_dim = n / opt_model_->getHorizon();
	for (unsigned int i = 0; i < opt_model_->getHorizon(); i++)
		full_initial_state.segment(i * state_dim, state_dim) = starting_state;

	return true;
}


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	obj_value = 0;

	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	// Computing the cost for predefined horizon
	LocomotionState locomotion_state;
	unsigned int state_dim = n / opt_model_->getHorizon();
	for (unsigned int i = 0; i < opt_model_->getHorizon(); i++) {
		// Converting the decision variable for a certain time to a robot state
		Eigen::VectorXd decision_state = decision_var.segment(i * state_dim, state_dim);
		opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);

		// Computing the function cost for a certain time
		double cost;
		unsigned int num_cost_functions = opt_model_->getCosts().size();
		for (unsigned int j = 0; j < num_cost_functions; j++) {
			opt_model_->getCosts()[j]->compute(cost, locomotion_state);
			obj_value += cost;
		}
	}

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::Map<Eigen::VectorXd> total_gradient(grad_f, n);

	// Computing the gradient of the cost function for a predefined horizon
	LocomotionState locomotion_state;
	unsigned int state_dim = n / opt_model_->getHorizon();
	for (unsigned int i = 0; i < opt_model_->getHorizon(); i++) {
		// Converting the decision variable for a certain time to a robot state
		Eigen::VectorXd decision_state = decision_var.segment(i * state_dim, state_dim);
		opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);

		// Computing the gradient for each defined cost function
		Eigen::VectorXd gradient(state_dim);
		total_gradient.setZero();
		unsigned int num_cost_functions = opt_model_->getCosts().size();
		for (unsigned int j = 0; j < num_cost_functions; j++) {
			opt_model_->getCosts()[j]->computeGradient(gradient, locomotion_state);
			total_gradient.segment(i * state_dim, state_dim) += gradient;
		}
	}

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::Map<Eigen::VectorXd> full_constraint(g, m);
	full_constraint.setZero();

	// Computing state dimension
	unsigned int state_dim = n / opt_model_->getHorizon();

	// Setting the initial state
	LocomotionState locomotion_state;
	Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dim);
	opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);
	unsigned int num_constraints = opt_model_->getConstraints().size();
	for (unsigned int j = 0; j < num_constraints + 1; j++) {
		if (j == 0) // dynamic system constraint
			opt_model_->getDynamicalSystem()->setLastState(locomotion_state);
		else
			opt_model_->getConstraints()[j-1]->setLastState(locomotion_state);
	}

	// Computing the active and inactive constraints for a predefined horizon
	unsigned int index = 0;
	for (unsigned int i = 0; i < opt_model_->getHorizon(); i++) {
		// Converting the decision variable for a certain time to a robot state
		decision_state = decision_var.segment(i * state_dim, state_dim);
		opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);

		// Computing the constraints for a certain time
		Eigen::VectorXd constraint;
		for (unsigned int j = 0; j < num_constraints + 1; j++) {
			if (j == 0) {// dynamic system constraint
				opt_model_->getDynamicalSystem()->compute(constraint, locomotion_state);
				opt_model_->getDynamicalSystem()->setLastState(locomotion_state);
			} else {
				opt_model_->getConstraints()[j-1]->compute(constraint, locomotion_state);
				opt_model_->getConstraints()[j-1]->setLastState(locomotion_state);
			}

			unsigned int constraint_size = constraint.size();
			full_constraint.segment(index, constraint_size) = constraint;

			index += constraint_size;
		}
	}

	return true;
}


bool IpoptWrapper::eval_jac_g(Index n, const Number* x, bool new_x,
							  Index m, Index nele_jac, Index* row_entries, Index* col_entries,
							  Number* values)
{
	if (values == NULL) {
		// Returns the structure of the Jacobian assuming it's dense
		int idx = 0;
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				row_entries[idx] = i;
				col_entries[idx] = j;
				idx++;
			}
		}
	} else {
		// Returns the values of the Jacobian of the constraints
		// Eigen interfacing to raw buffers
		Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
		Eigen::Map<MatrixRXd> full_jacobian(values, m, n);
		full_jacobian.setZero();
		Eigen::MatrixXd jacobian;

		// Computing state dimension
		unsigned int state_dim = n / opt_model_->getHorizon();

		// Setting the initial state
		LocomotionState locomotion_state;
		Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dim);
		opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);
		unsigned int num_constraints = opt_model_->getConstraints().size();
		for (unsigned int j = 0; j < num_constraints + 1; j++) {
			if (j == 0) // dynamic system constraint
				opt_model_->getDynamicalSystem()->setLastState(locomotion_state);
			else
				opt_model_->getConstraints()[j-1]->setLastState(locomotion_state);
		}

		// Computing the Jacobian for a predefined horizon
		unsigned int row_index = 0, col_index = 0;
		for (unsigned int i = 0; i < opt_model_->getHorizon(); i++) {
			// Converting the decision variable for a certain time to a robot state
			decision_state = decision_var.segment(i * state_dim, state_dim);
			opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);

			// Computing the Jacobian of the constraints for a certain time
			for (unsigned int j = 0; j < num_constraints + 1; j++) {
				if (j == 0)
					opt_model_->getDynamicalSystem()->computeJacobian(jacobian, locomotion_state);
				else
					opt_model_->getConstraints()[j-1]->computeJacobian(jacobian, locomotion_state);

				unsigned int constraint_size = jacobian.rows();
				unsigned int num_variables = jacobian.cols();
				full_jacobian.block(row_index, col_index, constraint_size, num_variables) = jacobian;

				row_index += constraint_size;
				col_index += num_variables;
			}
		}
	}

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
						  Index m, const Number* lambda, bool new_lambda,
						  Index nele_hess, Index* row_entries, Index* col_entries, Number* values) //TODO clean it!
{
	if (values == NULL) {
		// Returns the structure. This is a symmetric matrix, fill the lower left
		// triangle only. Assume the Hessian is dense
		Index idx = 0;
		for (Index row = 0; row < n; row++) {
			for (Index col = 0; col <= row; col++) {
				row_entries[idx] = row;
				col_entries[idx] = col;
				idx++;
			}
		}

		assert(idx == nele_hess);
	} else {
//		Eigen::MatrixXd full_hessian(m, n);
//		full_hessian.setZero();
//		values = full_hessian.data();

		// return the values. This is a symmetric matrix, fill the lower left
		// triangle only

		// fill the objective portion
		values[0] = obj_factor * (2*x[3]); // 0,0
		values[1] = obj_factor * (x[3]);   // 1,0
		values[2] = 0.;                    // 1,1
		values[3] = obj_factor * (x[3]);   // 2,0
		values[4] = 0.;                    // 2,1
		values[5] = 0.;                    // 2,2
		values[6] = obj_factor * (2*x[0] + x[1] + x[2]); // 3,0
		values[7] = obj_factor * (x[0]);                 // 3,1
		values[8] = obj_factor * (x[0]);                 // 3,2
		values[9] = 0.;                                  // 3,3

		// add the portion for the first constraint
		values[1] += lambda[0] * (x[2] * x[3]); // 1,0
		values[3] += lambda[0] * (x[1] * x[3]); // 2,0
		values[4] += lambda[0] * (x[0] * x[3]); // 2,1
		values[6] += lambda[0] * (x[1] * x[2]); // 3,0
		values[7] += lambda[0] * (x[0] * x[2]); // 3,1
		values[8] += lambda[0] * (x[0] * x[1]); // 3,2

		// add the portion for the second constraint
		values[0] += lambda[1] * 2; // 0,0
		values[2] += lambda[1] * 2; // 1,1
		values[5] += lambda[1] * 2; // 2,2
		values[9] += lambda[1] * 2; // 3,3
//		return false;// use limited memory approx http://www.coin-or.org/Ipopt/documentation/node31.html
	}

	return true;
}


void IpoptWrapper::finalize_solution(Ipopt::SolverReturn status,
									 Index n, const Number* x, const Number* z_L, const Number* z_U,
									 Index m, const Number* g, const Number* lambda,
									 Number obj_value, const Ipopt::IpoptData* ip_data,
									 Ipopt::IpoptCalculatedQuantities* ip_cq)
{
	// here is where we would store the solution to variables, or write to a file, etc
	// so we could use the solution.

	// For this example, we write the solution to the console
	printf("\n\nSolution of the primal variables, x\n");
	for (Index i=0; i<n; i++)
		printf("x[%d] = %e\n", i, x[i]);

	printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
	for (Index i=0; i<n; i++)
		printf("z_L[%d] = %e\n", i, z_L[i]);

	for (Index i=0; i<n; i++)
		printf("z_U[%d] = %e\n", i, z_U[i]);

	printf("\n\nObjective value\n");
	printf("f(x*) = %e\n", obj_value);

	printf("\nFinal value of the constraints:\n");
	for (Index i=0; i<m ;i++)
		printf("g(%d) = %e\n", i, g[i]);
}

} //@namespace solver
} //@namespace dwl
