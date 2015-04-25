#include <solver/IpoptWrapper.h>


namespace dwl
{

namespace solver
{


IpoptWrapper::IpoptWrapper() : model_(NULL)
{

}


IpoptWrapper::~IpoptWrapper()
{

}


void IpoptWrapper::reset(model::Model* model)
{
	model_ = model;
}


bool IpoptWrapper::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
								Index& nnz_h_lag, IndexStyleEnum& index_style) //TODO make general
{
//	model_
//	int num_knots = 0;

	// The problem described in HS071_NLP.hpp has 4 variables, x[0] through x[3]
	n = model_->getDimensionOfDecisionVariables();

	// one equality constraint and one inequality constraint
	m = model_->getDimensionOfConstraints();

    // Assuming that Jacobian and Hessian are dense
    nnz_jac_g = n * m;
    nnz_h_lag = n * (n + 1) * 0.5;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

	return true;
}


bool IpoptWrapper::get_bounds_info(Index n, Number* x_l, Number* x_u,
								   Index m, Number* g_l, Number* g_u) //TODO make general
{
	// here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
	// If desired, we could assert to make sure they are what we think they are.
	assert(n == 4);
	assert(m == 2);

	// the variables have lower bounds of 1
	for (Index i=0; i<4; i++)
		x_l[i] = 1.0;

	// the variables have upper bounds of 5
	for (Index i=0; i<4; i++)
		x_u[i] = 5.0;

	// the first constraint g1 has a lower bound of 25
	g_l[0] = 25;
	// the first constraint g1 has NO upper bound, here we set it to 2e19.
	// Ipopt interprets any number greater than nlp_upper_bound_inf as
	// infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
	// is 1e19 and can be changed through ipopt options.
	g_u[0] = 2e19;

	// the second constraint g2 is an equality constraint, so we set the
	// upper and lower bound to the same value
	g_l[1] = g_u[1] = 40.0;

	return true;
}


bool IpoptWrapper::get_starting_point(Index n, bool init_x, Number* x,
									  bool init_z, Number* z_L, Number* z_U,
									  Index m, bool init_lambda, Number* lambda) //TODO make general
{
	// Here, we assume we only have starting values for x, if you code
	// your own NLP, you can provide starting values for the dual variables
	// if you wish
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

	// initialize to the given starting point
	x[0] = 1.0;
	x[1] = 5.0;
	x[2] = 5.0;
	x[3] = 1.0;

	return true;
}


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	assert(n == 4);

	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);

	StateModel state;
	model_->convertDecisionVariablesToStateModel(state, decision_var);

	double cost;
	obj_value = 0;
	int num_cost_functions = model_->getCosts().size();
	for (int i = 0; i < num_cost_functions; i++) {
		model_->getCosts()[i]->compute(cost, state);
		obj_value += cost;
	}

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
//	assert(n == 4);

	// Eigen Interfacing to raw buffers
	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::Map<Eigen::VectorXd> total_gradient(grad_f, n);

	StateModel state;
	model_->convertDecisionVariablesToStateModel(state, decision_var);

	// Computing the gradient for each defined cost function
	Eigen::VectorXd gradient(n);
	total_gradient.setZero();
	int num_cost_functions = model_->getCosts().size();
	for (int i = 0; i < num_cost_functions; i++) {
		model_->getCosts()[i]->computeGradient(gradient, state);
		total_gradient += gradient;
	}

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
//	assert(n == 4);
//	assert(m == 2);

	const Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
	Eigen::VectorXd constraint;
	Eigen::Map<Eigen::VectorXd> full_constraint(g, m);
	full_constraint.setZero();

	StateModel state;
	model_->convertDecisionVariablesToStateModel(state, decision_var);


	int index = 0, num_constraint = 0;
	int num_active_constraints = model_->getActiveConstraints().size();
	std::cout << "Number of active constraints " << num_active_constraints << std::endl;
	for (int i = 0; i < num_active_constraints; i++) {
		model_->getActiveConstraints()[i]->compute(constraint, state);

		num_constraint = constraint.size();
		full_constraint.segment(index, num_constraint) = constraint;

		index += num_constraint;
	}

	int num_inactive_constraints = model_->getInactiveConstraints().size();
	std::cout << "Number of inactive constraints " << num_inactive_constraints << std::endl;
	for (int i = 0; i < num_inactive_constraints; i++) {
		model_->getInactiveConstraints()[i]->compute(constraint, state);

		num_constraint = constraint.size();
		full_constraint.segment(index, num_constraint) = constraint;

		index += num_constraint;
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
		Eigen::Map<const Eigen::VectorXd> decision_var(x, n);
		Eigen::Map<MatrixRXd> full_jacobian(values, m, n);
		full_jacobian.setZero();
		Eigen::MatrixXd jacobian;

		StateModel state;
		model_->convertDecisionVariablesToStateModel(state, decision_var);

		int row_index = 0, col_index = 0;
		int num_constraints = 0, num_variables = 0;

		int num_active_constraints = model_->getActiveConstraints().size();
		for (int i = 0; i < num_active_constraints; i++) {
			model_->getActiveConstraints()[i]->computeJacobian(jacobian, state);

			num_constraints = jacobian.rows();
			num_variables = jacobian.cols();
			full_jacobian.block(row_index, col_index, num_constraints, num_variables) = jacobian;

			row_index += num_constraints;
			col_index += num_variables;
		}

		int num_inactive_constraints = model_->getInactiveConstraints().size();
		for (int i = 0; num_inactive_constraints; i++) {
			model_->getInactiveConstraints()[i]->computeJacobian(jacobian, state);

			num_constraints = jacobian.rows();
			num_variables = jacobian.cols();
			full_jacobian.block(row_index, col_index, num_constraints, num_variables) = jacobian;

			row_index += num_constraints;
			col_index += num_variables;
		}
	}

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
						  Index m, const Number* lambda, bool new_lambda,
						  Index nele_hess, Index* row_entries, Index* col_entries, Number* values) //TODO test it!
{
	if (values == NULL) {
		// return the structure. This is a symmetric matrix, fill the lower left
		// triangle only.

		// the hessian for this problem is actually dense
		Index idx=0;
		for (Index row = 0; row < 4; row++) {
			for (Index col = 0; col <= row; col++) {
				row_entries[idx] = row;
				col_entries[idx] = col;
				idx++;
			}
		}

		assert(idx == nele_hess);
	} else {
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
	}



	//TODO fix it
//	if (values == NULL) {
//		// Returns the structure. This is a symmetric matrix, fill the lower left
//		// triangle only. Assume the Hessian is dense
//		int idx = 0;
//		for (int i = 0; i < n; i++){
//			for (int j = 0; j <= i; j++){
//				row_entries[idx] = i;
//				col_entries[idx] = j;
//				idx++;
//			}
//		}
//		assert(idx == nele_hess);
//	} else {
//		Eigen::MatrixXd full_hessian(m, n);
//		full_hessian.setZero();
//
//
//		//TODO make general
//
//
//
////		values = full_hessian.data();
//
//		// return the values. This is a symmetric matrix, fill the lower left
//	    // triangle only
//
//	    // fill the objective portion
//	    values[0] = obj_factor * (2*x[3]); // 0,0
//	    values[1] = obj_factor * (x[3]);   // 1,0
//	    values[2] = 0.;                    // 1,1
//	    values[3] = obj_factor * (x[3]);   // 2,0
//	    values[4] = 0.;                    // 2,1
//	    values[5] = 0.;                    // 2,2
//	    values[6] = obj_factor * (2*x[0] + x[1] + x[2]); // 3,0
//	    values[7] = obj_factor * (x[0]);                 // 3,1
//	    values[8] = obj_factor * (x[0]);                 // 3,2
//	    values[9] = 0.;                                  // 3,3
//
//
//	    // add the portion for the first constraint
//	    values[1] += lambda[0] * (x[2] * x[3]); // 1,0
//	    values[3] += lambda[0] * (x[1] * x[3]); // 2,0
//	    values[4] += lambda[0] * (x[0] * x[3]); // 2,1
//	    values[6] += lambda[0] * (x[1] * x[2]); // 3,0
//	    values[7] += lambda[0] * (x[0] * x[2]); // 3,1
//	    values[8] += lambda[0] * (x[0] * x[1]); // 3,2
//
//	    // add the portion for the second constraint
//	    values[0] += lambda[1] * 2; // 0,0
//	    values[2] += lambda[1] * 2; // 1,1
//	    values[5] += lambda[1] * 2; // 2,2
//	    values[9] += lambda[1] * 2; // 3,3
//
////		return false;// use limited memory approx http://www.coin-or.org/Ipopt/documentation/node31.html
//	}

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
