#ifndef DWL__SOLVER__QUADRATIC_PROGRAM__H
#define DWL__SOLVER__QUADRATIC_PROGRAM__H


namespace dwl
{

namespace solver
{

/**
 @class QuadraticProgram
 @brief Abstract class to define the optimization algorithm for Model Predictive Control. This class
 acts as an interface to use a defined optimization solver software as a part of this library in
 order to provide different solver options for the end user to solve the basic optimization problem
 that rises in MPC. As more solvers are adapted to this library with this class, more options to
 try different optimization methods are available to select the most suitable one depending on
 each case. The basic
 \f{eqnarray*}{
	\mbox{Minimize} \; F(x) && \\
	\mbox{subject to} \; G(x) & = & 0 \\
	H(x) & \geq & 0
\f}
 As more solvers are adapted to this library with this class, more options to try different
 optimization methods are available to select the most suitable one depending on each case.
*/
class QuadraticProgram
{
	public:
		/** @brief Constructor function */
		QuadraticProgram();

		/** @brief Destructor function */
		virtual ~QuadraticProgram();

		/**
	 	 * @brief Function to perform the initialization of optimizer, if this applies.
	 	 * @return Label that indicates if the initialization of the optimizer is successful
		 */
		virtual bool init() = 0;
				
		/**
	 	 * @brief Function to compute the optimization algorithm associated to the MPC problem.
	 	 * @param double* H Hessian matrix
	 	 * @param double* g Gradient vector
	 	 * @param double* G	Constraint matrix
	 	 * @param double* lb	Low bound vector
	 	 * @param double* ub	Upper bound vector
	 	 * @param double* lbG	Low constraint vector
	 	 * @param double* lbG	Upper constraint vector
	 	 * @param double cputime	CPU-time for computing the optimization. If NULL, it provides on output
	 	 * the actual calculation time of the optimization problem.
	 	 * @return bool Label that indicates if the computation of the optimization is successful
		 */
		virtual bool compute(double *H,
							 double *g,
							 double *G,
							 double *lb,
							 double *ub,
							 double *lbG,
							 double *ubG,
							 double cputime) = 0;
				
		/**
	 	 * @brief Get the vector of optimal or sub-optimal solutions calculated by the
	 	 * dwl::solver::QuadraticProgram::computeOpt() function (optimality of the function is defined
	 	 * by the solver that is adapted).
	 	 * @return double* Optimal solution
		 */
		virtual double* getOptimalSolution() = 0;

		/** @brief Set the horizon of the MPC */
		void setHorizon(int horizon); //TODO remove it

		/** @brief Set the number of variables, i.e inputs * horizon */
		void setNumberOfVariables(int variables);

		/**
	 	 * @brief Get the number of variables, i.e inputs * horizon
	 	 * @return int Number of variables
		 */
		int getNumberOfVariables() const;

		/**
	 	 * @brief Get the number of constraints
	 	 * @return int Number of constraints
		 */
		int getNumberOfConstraints() const;


	protected:
		/** @brief Label that indicates if QP solver had been initialized */
		bool initialized_solver_;

		/** @brief Horizon of MPC */
		int horizon_;

		/** @brief Number of variables, i.e inputs * horizon */
		int variables_;

		/** @brief Number of constraints */
		int constraints_;
};

} //@namepace solver
} //@namespace dwl

#endif

