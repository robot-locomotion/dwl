#include <solver/IpoptWrapper.h>


namespace dwl
{

namespace solver
{


IpoptWrapper::IpoptWrapper() : is_added_cost_(false), is_added_active_constraint_(false),
		is_added_inactive_constraint_(false)
{

}


IpoptWrapper::~IpoptWrapper()
{
	typedef std::vector<model::Constraint*>::iterator ConstraintItr;
	typedef std::vector<model::Cost*>::iterator CostItr;
	if (is_added_active_constraint_) {
		for (ConstraintItr i = active_constraints_.begin(); i != active_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_inactive_constraint_) {
		for (ConstraintItr i = inactive_constraints_.begin(); i != inactive_constraints_.end(); i++)
			delete *i;
	}
	if (is_added_cost_) {
		for (CostItr i = costs_.begin(); i != costs_.end(); i++)
			delete *i;
	}
}


void IpoptWrapper::addConstraint(model::Constraint* constraint)
{
	if (constraint->isActive()) {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		active_constraints_.push_back(constraint);

		if (!is_added_active_constraint_)
			is_added_active_constraint_ = true;
	}
	else {
		printf(GREEN "Adding the active %s constraint\n" COLOR_RESET, constraint->getName().c_str());
		inactive_constraints_.push_back(constraint);

		if (!is_added_inactive_constraint_)
			is_added_inactive_constraint_ = true;
	}
}


void IpoptWrapper::addCost(model::Cost* cost)
{
	printf(GREEN "Adding the %s cost\n" COLOR_RESET, cost->getName().c_str());

	costs_.push_back(cost);
	is_added_cost_ = true;
}


bool IpoptWrapper::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
								Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    // Assuming that Jacobian and Hessin are dense
    nnz_jac_g = n * m;
    nnz_h_lag = n * (n + 1) * 0.5;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

	return true;
}


bool IpoptWrapper::get_bounds_info(Index n, Number* x_l, Number* x_u,
								   Index m, Number* g_l, Number* g_u)
{

	return true;
}


bool IpoptWrapper::get_starting_point(Index n, bool init_x, Number* x,
									  bool init_z, Number* z_L, Number* z_U,
									  Index m, bool init_lambda, Number* lambda)
{

	return true;
}


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) //TODO test it!
{
	Eigen::Map<const Eigen::VectorXd> state(x, n);

	int num_cost_functions = costs_.size();
	for (int i = 0; num_cost_functions; i++)
		obj_value += costs_[i]->get((Eigen::VectorXd) state);

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) //TODO test it!
{
	Eigen::Map<const Eigen::VectorXd> state(x, n);

	Eigen::VectorXd total_gradient(n), gradient(n);
	total_gradient.setZero();

	int num_cost_functions = costs_.size();
	for (int i = 0; num_cost_functions; i++)
		costs_[i]->getGradient(gradient, (Eigen::VectorXd) state);
		total_gradient += gradient;

	grad_f = total_gradient.data();

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) //TODO test it!
{
	Eigen::Map<const Eigen::VectorXd> state(x, n);

	Eigen::VectorXd full_constraint(m);
	full_constraint.setZero();

	Eigen::VectorXd constraint;
	int index = 0, num_constraint = 0;
	if (is_added_active_constraint_) {
		int num_active_constraints = active_constraints_.size();
		for (int i = 0; num_active_constraints; i++) {
			active_constraints_[i]->get(constraint, (Eigen::VectorXd) state);

			num_constraint = constraint.size();
			full_constraint.segment(index, num_constraint) = constraint;

			index += num_constraint;
		}
	}

	if (is_added_inactive_constraint_) {
		int num_inactive_constraints = inactive_constraints_.size();
		for (int i = 0; num_inactive_constraints; i++) {
			inactive_constraints_[i]->get(constraint, (Eigen::VectorXd) state);

			num_constraint = constraint.size();
			full_constraint.segment(index, num_constraint) = constraint;

			index += num_constraint;
		}
	}

	g = full_constraint.data();

	return true;
}


bool IpoptWrapper::eval_jac_g(Index n, const Number* x, bool new_x,
							  Index m, Index nele_jac, Index* row_entries, Index* col_entries,
							  Number* values) //TODO test it!
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
		Eigen::Map<const Eigen::VectorXd> state(x, n);

		Eigen::MatrixXd full_jacobian(m, n);
		full_jacobian.setZero();

		Eigen::MatrixXd jacobian;
		int row_index = 0, col_index = 0;
		int num_constraints = 0, num_variables = 0;

		if (is_added_active_constraint_) {
			int num_active_constraints = active_constraints_.size();
			for (int i = 0; num_active_constraints; i++) {
				active_constraints_[i]->getJacobian(jacobian, (Eigen::VectorXd) state);

				num_constraints = jacobian.rows();
				num_variables = jacobian.cols();
				full_jacobian.block(row_index, col_index, num_constraints, num_variables) = jacobian;

				row_index += num_constraints;
				col_index += num_variables;
			}
		}

		if (is_added_inactive_constraint_) {
			int num_inactive_constraints = inactive_constraints_.size();

			for (int i = 0; num_inactive_constraints; i++) {
				inactive_constraints_[i]->getJacobian(jacobian, (Eigen::VectorXd) state);

				num_constraints = jacobian.rows();
				num_variables = jacobian.cols();
				full_jacobian.block(row_index, col_index, num_constraints, num_variables) = jacobian;

				row_index += num_constraints;
				col_index += num_variables;
			}
		}

		values = full_jacobian.data();
	}

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
						  Index m, const Number* lambda, bool new_lambda,
						  Index nele_hess, Index* row_entries, Index* col_entries, Number* values) //TODO test it!
{
	if (values == NULL) {
		// Returns the structure. This is a symmetric matrix, fill the lower left
		// triangle only. Assume the Hessian is dense
		int idx = 0;
		for (int i = 0; i < n; i++){
			for (int j = 0; j <= i; j++){
				row_entries[idx] = i;
				col_entries[idx] = j;
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

}

} //@namespace solver
} //@namespace dwl
