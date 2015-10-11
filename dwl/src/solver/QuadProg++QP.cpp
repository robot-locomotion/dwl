#include <solver/QuadProg++QP.h>


namespace dwl
{

namespace solver
{

QuadProgQP::QuadProgQP()
{

}


QuadProgQP::~QuadProgQP()
{

}


bool QuadProgQP::init()
{
	return true;
}


bool QuadProgQP::compute(double allocated_time_secs)
{
	// Evaluating constraints and bounds
//	model_.evaluateConstraints();
//	model_.evaluateBounds();


	/*
	double cost = solve_quadprog(Eigen::MatrixXd& G, Eigen::VectorXd& g0,
			 	 	 	  const Eigen::MatrixXd& CE, const Eigen::VectorXd& ce0,
			 	 	 	  const Eigen::MatrixXd& CI, const Eigen::VectorXd& ci0,
			 	 	 	  Eigen::VectorXd& x);

*/
	return true;
}

} //@namespace solver
} //@namespace dwl
