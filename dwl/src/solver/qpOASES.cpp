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


bool qpOASES::compute(double *H,
					  double *g,
					  double *G,
					  double *lb,
					  double *ub,
					  double *lbG,
					  double *ubG,
					  double cputime)
{
	// solve first QP.
	double cpu_time;
	
	returnValue retval;
	if (!initialized_solver_) {
		retval = solver_->init(H, g, G, lb, ub, lbG, ubG, num_wsr_, &cputime);
		if (retval == SUCCESSFUL_RETURN) {
			printf("qpOASES problem successfully initialized.");
			initialized_solver_ = true;
		}
	} else
		retval = solver_->hotstart(H, g, G, lb, ub, lbG, ubG, num_wsr_, &cputime);

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

