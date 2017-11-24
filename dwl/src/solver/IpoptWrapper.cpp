#include <dwl/solver/IpoptWrapper.h>


namespace dwl
{

namespace solver
{

IpoptWrapper::IpoptWrapper() : opt_model_(NULL), jacobian_(false), hessian_(false)
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
	// Initializing the optimization model
	opt_model_->init();

	// Getting the dimension of decision variables for every knots
	n = opt_model_->getDimensionOfState();

	// Getting the dimension of constraints for every knots
	m = opt_model_->getDimensionOfConstraints();

	// Getting the number of nonzero values of the Jacobian
	unsigned int nnz_jac = opt_model_->getNumberOfNonzeroJacobian();
	jacobian_ = opt_model_->isConstraintJacobianImplemented();
	if (nnz_jac == 0 || !jacobian_) // Assume that the Jacobian is dense
		nnz_jac_g = n * m;
	else
		nnz_jac_g = nnz_jac;

	// Getting the number of nonzero values of the Hessian
	unsigned int nnz_hess = opt_model_->getNumberOfNonzeroHessian();
	hessian_ = opt_model_->isLagrangianHessianImplemented();
	if (nnz_hess == 0 || !hessian_) // Assume that the Hessian is dense
		nnz_h_lag = n * (n + 1) * 0.5;
	else
		nnz_h_lag = nnz_hess;

	// use the C style indexing (0-based)
	index_style = TNLP::C_STYLE;

	return true;
}


bool IpoptWrapper::get_bounds_info(Index n, Number* x_l, Number* x_u,
								   Index m, Number* g_l, Number* g_u)
{
	// Evaluating the bounds
	opt_model_->evaluateBounds(x_l, n, x_u, n,
							   g_l, m, g_u, m);

	return true;
}


bool IpoptWrapper::get_starting_point(Index n, bool init_x, Number* x,
									  bool init_z, Number* z_L, Number* z_U,
									  Index m, bool init_lambda, Number* lambda)
{
	// Here, we assume we only have starting values for x, if you code your own NLP, you can
	// provide starting values for the dual variables if you wish to use a warmstart option
	opt_model_->getStartingPoint(x, n);

	return true;
}


bool IpoptWrapper::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	// Numerical evaluation of the cost function
	opt_model_->evaluateCosts(obj_value, x, n);

	return true;
}


bool IpoptWrapper::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	// Computing the gradient of the cost function
	opt_model_->evaluateCostGradient(grad_f, n, x, n);

	return true;
}


bool IpoptWrapper::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	// Numerical evaluation of the constraint function
	opt_model_->evaluateConstraints(g, m, x, n);

	return true;
}


bool IpoptWrapper::eval_jac_g(Index n, const Number* x, bool new_x,
							  Index m, Index nele_jac, Index* row_entries, Index* col_entries,
							  Number* values)
{
	bool flag = false;
	if (values == NULL) {
		flag = true;
	}

	if (!jacobian_) {
		if (flag) {
			// Returns the structure of the Jacobian assuming it's dense
			int idx = 0;
			for (int i = 0; i < m; ++i) {
				for (int j = 0; j < n; ++j) {
					row_entries[idx] = i;
					col_entries[idx] = j;
					idx++;
				}
			}
		}
	} else {
		// Computing the Jacobian and its sparsity structure
		opt_model_->evaluateConstraintJacobian(values, nele_jac,
											   row_entries, nele_jac,
											   col_entries, nele_jac,
											   x, n, flag);
	}

	return true;
}


bool IpoptWrapper::eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
						  Index m, const Number* lambda, bool new_lambda,
						  Index nele_hess, Index* row_entries, Index* col_entries, Number* values)
{
	bool flag = false;
	if (values == NULL) {
		flag = true;
	}

	if (!hessian_) {
		if (flag) {
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
		}
	} else {
		// Computing the Hessian and its sparsity structure
		opt_model_->evaluateLagrangianHessian(values, nele_hess,
											  row_entries, nele_hess,
											  col_entries, nele_hess,
											  obj_factor,
											  lambda, m, x, n, flag);
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

	// Evaluating the solution
	solution_ = solution;
}


const Eigen::VectorXd& IpoptWrapper::getSolution()
{
	return solution_;
}

} //@namespace solver
} //@namespace dwl
