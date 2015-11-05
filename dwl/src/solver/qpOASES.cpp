#include <dwl/solver/qpOASES.h>


USING_NAMESPACE_QPOASES

namespace dwl
{

namespace solver
{

qpOASES::qpOASES() : solver_(NULL), qpOASES_solution_(NULL), num_wsr_(10)
{

}


qpOASES::~qpOASES()
{
	delete solver_;
}


bool qpOASES::init(unsigned int num_variables,
				   unsigned int num_constraints)
{
	// Initializing the number of variables and constraints
	variables_ = num_variables;
	constraints_ = num_constraints;

	// Initializing the qpOASES solution global variable
	qpOASES_solution_ = new double[variables_];

	// Initializing the SQP solver of qpOASES
	solver_ = new SQProblem(variables_, constraints_);
	
	// Setting the options of the SQP solver
	Options my_options;
	my_options.setToReliable();
//	my_options.setToMPC();
//	my_options.enableFlippingBounds = BT_TRUE;
//	my_options.printLevel = PL_LOW;
	solver_->setOptions(my_options);

	
	printf("qpOASES solver class successfully initialized.");
	return true;
}


bool qpOASES::compute(const Eigen::MatrixXd& hessian,
		 	 	 	  const Eigen::VectorXd& gradient,
		 	 	 	  const Eigen::MatrixXd& constraint_mat,
		 	 	 	  const Eigen::VectorXd& lower_bound,
		 	 	 	  const Eigen::VectorXd& upper_bound,
		 	 	 	  const Eigen::VectorXd& lower_constraint,
		 	 	 	  const Eigen::VectorXd& upper_constraint,
		 	 	 	  double cputime)
{
	// Initializing the solution vector
	solution_ = Eigen::VectorXd::Zero(variables_);

	// Ensuring the hessian matrix is row-major storage
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hessian_rowmajor = hessian;
	
	// Solving first QP
	returnValue retval;
	if (!initialized_solver_) {
		retval = solver_->init(hessian_rowmajor.data(),
							   gradient.data(),
							   constraint_mat.data(),
							   lower_bound.data(), upper_bound.data(),
							   lower_constraint.data(), upper_constraint.data(),
							   num_wsr_, &cputime);
		if (retval == SUCCESSFUL_RETURN) {
			printf("qpOASES problem successfully initialized");
			initialized_solver_ = true;
		}
	} else
		retval = solver_->hotstart(hessian_rowmajor.data(),
				   	   	   	   	   gradient.data(),
				   	   	   	   	   constraint_mat.data(),
				   	   	   	   	   lower_bound.data(), upper_bound.data(),
				   	   	   	   	   lower_constraint.data(), upper_constraint.data(),
				   	   	   	   	   num_wsr_, &cputime);

	if (solver_->isInfeasible())
		printf("Warning: the quadratic programming is infeasible");
		
	if (retval == SUCCESSFUL_RETURN) {
		solver_->getPrimalSolution(qpOASES_solution_);
		Eigen::Map<Eigen::VectorXd> sol(qpOASES_solution_, variables_, 1);
		solution_ = sol;
	} else if (retval == RET_MAX_NWSR_REACHED) {
		printf("The QP could not solve because the maximun number of WSR was reached");
		return false;
	} else {
		printf("The QP could not find the solution");
		return false;
	}	
	
	return true;
}


void qpOASES::setNumberOfWorkingSetRecalculations(double num_wsr)
{
	num_wsr_ = num_wsr;

	printf("Setting the number of working set recalculations to %d", num_wsr_);
}

} //@namespace solver
} //@namespace dwl

