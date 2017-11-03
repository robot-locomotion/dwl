#ifndef DWL__MODEL__OPTIMIZATION_MODEL__H
#define DWL__MODEL__OPTIMIZATION_MODEL__H

#include <dwl/utils/utils.h>
#include <unsupported/Eigen/NumericalDiff>

#define NO_BOUND 2e19



namespace dwl
{

namespace model
{

/**
 * @class OptimizationModel
 * @brief A NLP problem requires information of constraints (dynamical, active or inactive) and
 * cost functions. Thus, OptimizationModelInterface is a class that allows us to implement
 * different optimization-based problems.
 */
class OptimizationModel
{
	public:
		/**
		 * @brief Functor struct requires for computing numerical differentiation
		 * (Jacobian computation) with Eigen
		 */
		template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
		struct Functor
		{
			typedef _Scalar Scalar;
			enum {
				InputsAtCompileTime = NX,
				ValuesAtCompileTime = NY
			};
			typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
			typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
			typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

			int m_inputs, m_values;

			Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
			Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

			int inputs() const { return m_inputs; }
			int values() const { return m_values; }
		};

		/**
		 * @brief Describes the cost function requires for computing numerical
		 * differentiation (gradient computation) with Eigen
		 */
		struct CostFunction : Functor<double>
		{
			CostFunction(OptimizationModel* model) : Functor<double>(0, 0), model_(model) {}
			int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& f) const
			{
				f.resize(1);
				double cost;
				model_->evaluateCosts(cost, x.data(), x.size());
				f(0) = cost;

				return 0;
			}

			OptimizationModel* const model_;
		};

		/** @brief Constructor function */
		OptimizationModel();

		/** @brief Destructor function */
		virtual ~OptimizationModel();

		/** @brief Initializes the optimization model, i.e. the dimensions of the optimization
		 * vectors */
		virtual void init(bool only_soft_constraints = false);

		/** @brief Sets the dimension of the decision variables, here called state */
		void setDimensionOfState(unsigned int dim);

		/** @brief Sets the dimension of the constraints variables */
		void setDimensionOfConstraints(unsigned int dim);

		/** @brief Sets the number of nonzero values of the Jacobian function */
		void setNumberOfNonzeroJacobian(unsigned int dim);

		/** @brief Sets the number of nonzero values of the Hessian function */
		void setNumberOfNonzeroHessian(unsigned int dim);

		/**
		 * @brief Gets the starting point of the problem
		 * @param double* Initial values for the decision variables, $x$
		 * @param int Number of the decision variables
		 */
		virtual void getStartingPoint(double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the bounds of the problem
		 * @param double* Lower bounds $x^L$ for $x$
		 * @param double* Upper bounds $x^L$ for $x$
		 * @param int Number of decision variables (dimension of $x$)
		 * @param double* Lower bounds of the constraints $g^L$ for $x$
		 * @param double* Upper bounds of the constraints $g^L$ for $x$
		 * @param int Number of constraints (dimension of $g(x)$)
		 */
		virtual void evaluateBounds(double* decision_lbound, int decision_dim1,
									double* decision_ubound, int decision_dim2,
									double* constraint_lbound, int constraint_dim1,
									double* constraint_ubound, int constraint_dim2);

		/**
		 * @brief Abstract method for evaluating the cost function given a current
		 * decision state
		 * @param double& Value of the objective function ($f(x)$).
		 * @param const double* Array of the decision variables, $x$, at which the cost functions,
		 * $f(x)$, is evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateCosts(double& cost,
								   const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the gradient of the cost function given a
		 * current decision state
		 * @param double* Array of values for the gradient of the objective function ($\nabla f(x)$)
		 * @param int Number of decision variables (dimension of $x$)
		 * @param const double* Array for the decision variables, $x$, at which $\nabla f(x)$ is
		 * evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateCostGradient(double* gradient, int grad_dim,
										  const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the constraint function given a
		 * current decision state
		 * @param double* Array of constraint function values, $g(x)$
		 * @param int Number of constraint variables (dimension of $g(x)$)
		 * @param const double* Array of the decision variables, $x$, at which the constraint functions,
		 * $g(x)$, are evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateConstraints(double* constraint, int constraint_dim,
								 	 	 const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the jacobian of the constraint function given a
		 * current decision state
		 * @param double* Values of the entries in the Jacobian of the constraints
		 * @param double* Row indices of entries in the Jacobian of the constraints
		 * @param double* Column indices of entries in the Jacobian of the constraints
		 * @param int Number of nonzero elements in the Jacobian (dimension of row_entries,
		 * col_entries, and values)
		 * @param double* Array for the decision variables, $x$, at which $\nabla g(x)^T$ is evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateConstraintJacobian(double* jacobian_values, int nonzero_dim1,
												int* row_entries, int nonzero_dim2,
												int* col_entries, int nonzero_dim3,
												const double* decision, int decision_dim, bool flag);

		/**
		 * @brief This method returns the structure of the Hessian of the Lagrangian (if "values" is
		 *  NULL) and the values of the hessian of the Lagrangian (if "values" is not NULL)
		 * @param Index Number of decision variables (dimension of $x$)
		 * @param const Number* Values for the primal variables, $x$, at which the Hessian is to be
		 * evaluated
		 * @param bool False if any evaluation method was previously called with the same values
		 * in $x$, true otherwise
		 * @param Number Factor in front of the objective term in the Hessian, $\sigma_f$
		 * @param Index Number of constraint variables (dimension of $g(x)$)
		 * @param const Number* Values for the constraint multipliers, $\lambda$ , at which the
		 * Hessian is to be evaluated
		 * @param bool False if any evaluation method was previously called with the same values in
		 * lambda, true otherwise
		 * @param Index Number of nonzero elements in the Hessian (dimension of row_entries,
		 * col_entries and values)
		 * @param Index* Row indices of entries in the Hessian
		 * @param Index* Column indices of entries in the Hessian
		 * @param Number* Values of the entries in the Hessian
		 */
		virtual void evaluateLagrangianHessian(double* hessian_values, int nonzero_dim1,
											   int* row_entries, int nonzero_dim2,
											   int* col_entries, int nonzero_dim3,
											   double obj_factor,
											   const double* lagrange, int constraint_dim,
											   const double* decision, int decision_dim,
											   bool flag);

		/** @brief Gets the dimension of the state vector of the optimization problem */
		unsigned int getDimensionOfState();

		/** @brief Gets the dimension of the constraints of the optimization problem */
		unsigned int getDimensionOfConstraints();

		/** @brief Gets the number of nonzero values of the Jacobian */
		unsigned int getNumberOfNonzeroJacobian();

		/** @brief Gets the number of nonzero values of the Hessian */
		unsigned int getNumberOfNonzeroHessian();

		/** @brief Returns true if the cost Gradient is implemented */
		bool isCostGradientImplemented();

		/** @brief Returns true if the constraint Jacobian is implemented */
		bool isConstraintJacobianImplemented();

		/** @brief Returns true if the Lagrangian Hessian is implemented */
		bool isLagrangianHessianImplemented();


	protected:
		/**@brief The solution vector */
		double* solution_;

		/** @brief Dimension of the state vector at instantaneous point */
		unsigned int state_dimension_;

		/** @brief Dimension of the constraint vector at instantaneous point */
		unsigned int constraint_dimension_;

		/** @brief Number of nonzero values of the Jacobian */
		unsigned int nonzero_jacobian_;

		/** @brief Number of nonzero values of the Hessian */
		unsigned int nonzero_hessian_;


	private:
		/** @brief True if the gradient of the cost function is implemented */
		bool gradient_;

		/** @brief True if the constraint Jacobian is implemented */
		bool jacobian_;

		/** @brief True if the Lagrangian Hessian is implemented */
		bool hessian_;

		bool first_time_;

		/** @brief Cost functor for numerical differentiation */
		CostFunction cost_function_;

		/** @brief Numerical differentiation mode */
		Eigen::NumericalDiffMode num_diff_mode_;

		/** @brief Machine epsilon constant which gives an upper bound on the relative error
		    due to rounding in floating point arithmetic */
		double epsilon_;
};

} //@namespace model
} //@namespace dwl


#endif
