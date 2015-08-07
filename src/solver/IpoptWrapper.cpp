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


void IpoptWrapper::setOptimizationModel(model::OptimizationModel* model)
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

	// Evaluating the bounds
	opt_model_->evaluateBounds(full_state_lower_bound, full_state_upper_bound,
							   full_constraint_lower_bound, full_constraint_upper_bound);

	return true;
}


bool IpoptWrapper::get_starting_point(Index n, bool init_x, Number* x,
									  bool init_z, Number* z_L, Number* z_U,
									  Index m, bool init_lambda, Number* lambda)
{
	// Here, we assume we only have starting values for x, if you code your own NLP, you can
	// provide starting values for the dual variables if you wish to use a warmstart option

	// Eigen interfacing to raw buffers
	Eigen::Map<Eigen::VectorXd> full_initial_state(x, n);

	// Setting the full starting state for the predefined horizon
	opt_model_->getStartingPoint(full_initial_state);

	return true;
}


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	// Numerical evaluation of the cost function
	opt_model_->evaluateCosts(obj_value, decision_var);

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::Map<Eigen::VectorXd> full_gradient(grad_f, n);

	// Computing the gradient of the cost function for a predefined horizon
	Eigen::MatrixXd grad(1,n);
	opt_model_->evaluateCostGradient(grad, decision_var);

	full_gradient = grad;

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::Map<Eigen::VectorXd> full_constraint(g, m);
	full_constraint.setZero();

	// Numerical evaluation of the constraint function
	opt_model_->evaluateConstraints(full_constraint, decision_var);

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

		// Computing the Jacobian for a predefined horizon
		Eigen::MatrixXd jac(m,n);
		opt_model_->evaluateConstraintJacobian(jac, decision_var);

		full_jacobian = jac;
	}

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
						  Index m, const Number* lambda, bool new_lambda,
						  Index nele_hess, Index* row_entries, Index* col_entries, Number* values)
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
		Eigen::MatrixXd full_hessian(m, n);
		full_hessian.setZero();
		values = full_hessian.data();

		return false;// use limited memory approx http://www.coin-or.org/Ipopt/documentation/node31.html
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
#ifdef DEBUG
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
#endif

	// Eigen interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> solution(x, n);
	unsigned int state_dim = solution.size() / opt_model_->getHorizon();

	// Recording the solution
	locomotion_solution_.clear();
	LocomotionState locomotion_state;
	Eigen::VectorXd decision_state = Eigen::VectorXd::Zero(state_dim);
	for (unsigned int k = 0; k < opt_model_->getHorizon(); k++) {
		// Converting the decision variable for a certain time to a robot state
		decision_state = solution.segment(k * state_dim, state_dim);
		opt_model_->getDynamicalSystem()->toLocomotionState(locomotion_state, decision_state);

		// Setting the time information in cases where time is not a decision variable
		if (opt_model_->getDynamicalSystem()->isFixedStepIntegration())
			locomotion_state.time = opt_model_->getDynamicalSystem()->getFixedStepTime() * k;

		// Pushing the current state
		locomotion_solution_.push_back(locomotion_state);
	}
}


std::vector<LocomotionState>& IpoptWrapper::getSolution()
{
	return locomotion_solution_;
}

} //@namespace solver
} //@namespace dwl
