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
 @class qpOASES
 @brief Class to interface the qpOASES library
 This class gives an interface with qpOASES library in order to implement a quadratic program using
 online active set strategy for MPC controller. qpOASES solve a convex optimization class of the
 following form
 \f[
	\min_{\mathbf{x}} \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{x}^T\mathbf{g(x_0)}
 \f]
 suject to
 \f{eqnarray*}{
   	lbG(\mathbf{x_0}) \leq &\mathbf{Gx}& \leq ubG(\mathbf{x_0}) \\
	lb(\mathbf{x_0})   \leq &\mathbf{x}&  \leq ub(\mathbf{x_0})
 \f}
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
		 * @return bool Label that indicates if the initialization of the optimizer is successful
		 */
		bool init();

		/**
 	 	 * @brief Function to solve the optimization problem formulated in the MPC
	 	 * @param double* Hessian matrix
	 	 * @param double* Gradient vector
	 	 * @param double* Constraint matrix
	 	 * @param double* Low bound vector
	 	 * @param double* Upper bound vector
	 	 * @param double* Low constraint vector
	 	 * @param double* Upper constraint vector
	 	 * @param double CPU-time for computing the optimization
	 	 * @return bool Label that indicates if the computation of the optimization is successful
		 */
		bool compute(double *H,
					 double *g,
					 double *G,
					 double *lb,
					 double *ub,
					 double *lbG,
					 double *ubG,
					 double cputime);

		/**
	 	 * @brief Get the vector of optimal solutions calculated by qpOASES
	 	 * @return double* Optimal solution
		 */
		double* getOptimalSolution();

		/**
		 * @brief Sets the number of working set recalculations used by qpOASES
		 * @param double Number of working set recalculations
		 */
		void setNumberOfWorkingSetRecalculations(double num_wsr);


	protected:
		/** @brief Optimal solution obtained with the implementation of qpOASES */
		double* optimal_solution_;
				
				
	private:
		/** @brief SQProblem object which is used to solve the quadratic problem */
		SQProblem* solver_;

		/** @brief Number of Working Set Recalculations */
		int num_wsr_;
};

} // @namespace solver
} // @namespace dwl



#endif
