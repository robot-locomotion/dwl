#ifndef DWL__SOLVER__QPOASES__H
#define DWL__SOLVER__QPOASES__H

#include <dwl/solver/QuadraticProgram.h>
#include <qpOASES.hpp>


USING_NAMESPACE_QPOASES

namespace dwl
{

namespace solver
{

/**
 * @class qpOASES
 * @brief Class to interface the qpOASES library
 * This class gives an interface with qpOASES library in order to implement a quadratic program using
 * online active set strategy of Ferreau et at. (2008: "An online active set strategy to overcome
 * the limitations of explicit MPC" such as used in MPC controllers", 2014: "{qpOASES}: A parametric
 * active-set algorithm for quadratic programming"). qpOASES solve a convex optimization
 * class of the following form
 * \f[
 * 	\min_{\mathbf{x}} \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{x}^T\mathbf{g(x_0)}
 * \f]
 * suject to
 * \f{eqnarray*}{
 *	lbG(\mathbf{x_0}) \leq &\mathbf{Gx}& \leq ubG(\mathbf{x_0}) \\
 *	lb(\mathbf{x_0})   \leq &\mathbf{x}&  \leq ub(\mathbf{x_0})
 * \f}
 */
class qpOASES : public QuadraticProgram
{
	public:
		/** @brief Constructor function */
		qpOASES();

		/** @brief Destructor function */
		~qpOASES();

		/**
	 	 * @brief Function to define the initialization of qpOASES optimizer
	 	 * @param unsigned int Number of variables of the QP problem
	 	 * @param unsigned int Number of constraints of the QP problem
		 * @return bool Label that indicates if the initialization of the optimizer is successful
		 */
		bool init(unsigned int num_variables,
				  unsigned int num_constraints);

		/**
 	 	 * @brief Function to solve the QP solution
	 	 * @param const Eigen::MatrixXd& Hessian matrix
	 	 * @param const Eigen::VectorXd Gradient vector
	 	 * @param const Eigen::MatrixXd& Constraint matrix
	 	 * @param const Eigen::VectorXd Low bound vector
	 	 * @param const Eigen::VectorXd Upper bound vector
	 	 * @param const Eigen::VectorXd Low constraint vector
	 	 * @param const Eigen::VectorXd Upper constraint vector
	 	 * @param double CPU-time for computing the optimization
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

		/**
		 * @brief Sets the number of working set recalculations used by qpOASES
		 * @param double Number of working set recalculations
		 */
		void setNumberOfWorkingSetRecalculations(double num_wsr);


	private:
		/** @brief SQProblem object which is used to solve the quadratic problem */
		SQProblem* solver_;

		/** @brief Optimal solution obtained with the implementation of qpOASES */
		double* qpOASES_solution_;

		/** @brief Number of Working Set Recalculations */
		int num_wsr_;
};

} // @namespace solver
} // @namespace dwl



#endif
