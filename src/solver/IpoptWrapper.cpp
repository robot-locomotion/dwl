#include <solver/IpoptWrapper.h>
#include <Eigen/Core>//TODO

namespace dwl
{

namespace solver
{


IpoptWrapper::IpoptWrapper()
{

}


IpoptWrapper::~IpoptWrapper()
{

}


bool IpoptWrapper::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
									Index& nnz_h_lag, IndexStyleEnum& index_style)
{


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


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	Eigen::VectorXd full_constraint(m);
	full_constraint.setZero();

	Eigen::Map<const Eigen::VectorXd> state(x, n);

	Eigen::VectorXd constraint;
	int index = 0, num_constraint = 0;
	int num_active_contraints = active_constraints_.size();
	int num_inactive_contraints = inactive_constraints_.size();
	for (int i = 0; num_active_contraints; i++) {
		active_constraints_[i]->get(constraint, (Eigen::VectorXd) state);

		num_constraint = constraint.size();
		full_constraint.segment(index, num_constraint) = constraint;

		index += num_constraint;
	}
	for (int i = 0; num_inactive_contraints; i++) {
		active_constraints_[i]->get(constraint, (Eigen::VectorXd) state);

		num_constraint = constraint.size();
		full_constraint.segment(index, num_constraint) = constraint;

		index += num_constraint;
	}

	g = full_constraint.data();

	return true;
}


bool IpoptWrapper::eval_jac_g(Index n, const Number* x, bool new_x,
								  Index m, Index nele_jac, Index* iRow, Index *jCol,
								  Number* values)
{

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
							 Index m, const Number* lambda, bool new_lambda,
							 Index nele_hess, Index* iRow, Index* jCol, Number* values)
{

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
