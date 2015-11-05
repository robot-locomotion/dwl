#ifndef DWL__SOLVER__QUADPROGPP_QP__H
#define DWL__SOLVER__QUADPROGPP_QP__H

#include <dwl/solver/QuadraticProgram.h>
#include <dwl/solver/QuadProg++.h>
#include <time.h>


namespace dwl
{

namespace solver
{

class QuadProgQP : public QuadraticProgram
{
	public:
		/** @brief Constructor function */
		QuadProgQP();

		/** @brief Destructor function */
		~QuadProgQP();

		/**
		 * @brief Initialization of the NLP solver using Ipopt
		 * @param unsigned int Number of variables of the QP problem
	 	 * @param unsigned int Number of constraints of the QP problem
		 * @return True if was initialized
		 */
		bool init(unsigned int num_variables,
		  	  	  unsigned int num_constraints);

		/**
	 	 * @brief Function to compute the QP solution
	 	 * @param const Eigen::MatrixXd& Hessian matrix
	 	 * @param const Eigen::VectorXd Gradient vector
	 	 * @param const Eigen::MatrixXd& Constraint matrix
	 	 * @param const Eigen::VectorXd Low bound vector
	 	 * @param const Eigen::VectorXd Upper bound vector
	 	 * @param const Eigen::VectorXd Low constraint vector
	 	 * @param const Eigen::VectorXd Upper constraint vector
	 	 * @param double CPU-time for computing the optimization. If NULL, it provides on output
	 	 * the actual calculation time of the optimization problem.
	 	 * @return bool Label that indicates if the computation of the optimization is successful
		 */
		bool compute(const Eigen::MatrixXd& hessian,
					 const Eigen::VectorXd& gradient,
					 const Eigen::MatrixXd& constraint_mat,
					 const Eigen::VectorXd& lower_bound,
					 const Eigen::VectorXd& upper_bound,
					 const Eigen::VectorXd& lower_constraint,
					 const Eigen::VectorXd& upper_constraint,
					 double cputime);
};

} //@namespace solver
} //@namespace dwl

#endif
