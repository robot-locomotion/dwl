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
class qpOASES : public dwl::solver::QuadraticProgram
{
	public:
		/** @brief Constructor function */
		qpOASES();

		/** @brief Destructor function */
		~qpOASES();

		/**
	 	 @brief Function to define the initialization of qpOASES optimizer
		 @return bool Label that indicates if the initialization of the optimizer is successful
		 */
		virtual bool init();

		/**
 	 	 @brief Function to solve the optimization problem formulated in the MPC
	 	 @param double* H	Hessian matrix
	 	 @param double* g	Gradient vector
	 	 @param double* G	Constraint matrix
	 	 @param double* lb	Low bound vector
	 	 @param double* ub	Upper bound vector
	 	 @param double* lbG	Low constraint vector
	 	 @param double* lbG	Upper constraint vector
	 	 @param double cputime	CPU-time for computing the optimization
	 	 @return bool Label that indicates if the computation of the optimization is successful
		 */
		virtual bool computeOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime);

		/**
	 	 @brief Get the vector of optimal solutions calculated by qpOASES
	 	 @return double* Optimal solution
		 */
		double* getOptimalSolution();


	protected:
		/** @brief Optimal solution obtained with the implementation of qpOASES */
		double* optimal_solution_;
				
				
	private:
		/** @brief SQProblem object which is used to solve the quadratic problem */
		SQProblem *solver_;
				
		/** @brief Label that indicates if qpOASES had been initialized */
		bool qpOASES_initialized_;
				
		/** @brief Number of Working Set Recalculations */
		int nWSR_;
};

} // @namespace solver
} // @namespace dwl



#endif
