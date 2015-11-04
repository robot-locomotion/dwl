#include <dwl/solver/qpOASES.h>


USING_NAMESPACE_QPOASES

namespace dwl
{

namespace solver
{

qpOASES::qpOASES() : solver_(NULL), num_wsr_(10)
{

}


qpOASES::~qpOASES()
{
	delete solver_;
}


bool qpOASES::init()
{
	// reading the parameters required for the solver
	if (nh_.getParam("optimizer/number_constraints", constraints_)) {
		ROS_INFO("Got param: number of constraints = %d", constraints_);
	}
	
	if (variables_ == 0 || constraints_ == 0 || horizon_ == 0)
		return false;
	
	solver_ = new SQProblem(variables_, constraints_ * horizon_);
	Options myOptions;
	myOptions.setToReliable();
//	myOptions.setToMPC();
//	myOptions.enableFlippingBounds = BT_TRUE;
//	myOptions.printLevel = PL_LOW;
	solver_->setOptions(myOptions);
	
	optimal_solution_ = new double[variables_];
	
	
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
			printf("qpOASES problem successfully initialized.");
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
		printf("Warning: the quadratic programming is infeasible.");
		
	if (retval == SUCCESSFUL_RETURN)
		solver_->getPrimalSolution(optimal_solution_);
	else if (retval == RET_MAX_NWSR_REACHED) {
		printf("The QP couldn't solve because the maximun number of WSR was reached.");
		return false;
	} else {
		printf("The QP could not find the solution.");
		return false;
	}	
	
	return true;
}


double* qpOASES::getOptimalSolution()
{
	return optimal_solution_;
}


void qpOASES::setNumberOfWorkingSetRecalculations(double num_wsr)
{
	num_wsr_ = num_wsr;

	printf("Setting the number of working set recalculations to %d", num_wsr_);
}

} //@namespace solver
} //@namespace dwl

