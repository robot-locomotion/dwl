#include <dwl/solver/qpOASES.h>
#include <iostream>
#include <vector>


USING_NAMESPACE_QPOASES

namespace dwl
{

namespace solver
{

qpOASES::qpOASES()
{
	variables_ = 0;
	constraints_ = 0;
	horizon_ = 0;
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
	
	nh_.param<int>("optimizer/working_set_recalculations", nWSR_, 10);
	ROS_INFO("Got param: number of working set recalculations = %d", nWSR_);
	
	if (variables_ == 0 || constraints_ == 0 || horizon_ == 0)
		return false;
	
	qpOASES_initialized_ = false;
	solver_ = new SQProblem(variables_, constraints_ * horizon_);
	Options myOptions;
	myOptions.setToReliable();
//	myOptions.setToMPC();
//	myOptions.enableFlippingBounds = BT_TRUE;
//	myOptions.printLevel = PL_LOW;
	solver_->setOptions(myOptions);
	
	optimal_solution_ = new double[variables_];
	
	
	ROS_INFO("qpOASES solver class successfully initialized.");
	return true;
}


bool qpOASES::computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime)
{
	// solve first QP.
	int nWSR = nWSR_;
	double cpu_time;
	
	returnValue retval;
	if (!qpOASES_initialized_) {
		cpu_time = cputime;
		retval = solver_->init(H, g, G, lb, ub, lbG, ubG, nWSR, &cpu_time);
		if (retval == SUCCESSFUL_RETURN) {
			ROS_INFO("qpOASES problem successfully initialized.");
			qpOASES_initialized_ = true;
		}
	}
	else {
		cpu_time = cputime;
		retval = solver_->hotstart(H, g, G, lb, ub, lbG, ubG, nWSR, &cpu_time);
	}
	if (solver_->isInfeasible())
		ROS_WARN("The quadratic programming is infeasible.");
		
	if (retval == SUCCESSFUL_RETURN) {
		solver_->getPrimalSolution(optimal_solution_);
	}
	else if (retval == RET_MAX_NWSR_REACHED) {
		ROS_WARN("The QP couldn't solve because the maximun number of WSR was reached.");
		return false;
	}
	else { 
		ROS_WARN("The QP couldn't find the solution.");
		return false;
	}	
	
	//std::cout << "cputime = " << cpu_time << std::endl;
	return true;
}


double* qpOASES::getOptimalSolution()
{
	return optimal_solution_;
}

} //@namespace solver
} //@namespace dwl

