#ifndef DWL__MODEL__OPTIMIZATION_MODEL__H
#define DWL__MODEL__OPTIMIZATION_MODEL__H

#include <dwl/utils/utils.h>
#include <unsupported/Eigen/NumericalDiff>

#define NO_BOUND 2e19



namespace dwl
{

namespace model
{

enum SoftConstraintFamily {QUADRATIC, UNWEIGHTED};

struct SoftConstraintProperties
{
	SoftConstraintProperties() : weight(0.), threshold(0.), offset(0.),
			family(QUADRATIC) {}
	SoftConstraintProperties(double _weight,
							 double _threshold,
							 double _offset,
							 enum SoftConstraintFamily _family = QUADRATIC) :
								 weight(_weight),
								 threshold(_threshold),
								 offset(_offset),
								 family(_family) {}

	double weight;
	double threshold;
	double offset;
	enum SoftConstraintFamily family;
};

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
 * @brief The OptimizationModel class
 * It models NonLinear Programming (NLP) problem that can be solved with
 * the different optimization solvers available in dwl. The information often
 * needed are constraints (dynamical, active or inactive) and cost functions.
 * Note that this class is an interface for solving easily different
 * optimization-based problems. In case the derivatives are not provided, there
 * is an optional NumDiff method that runs automatically.
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class OptimizationModel
{
	public:
		/**
		 * @brief Describes the cost function requires for computing numerical
		 * differentiation (gradient computation) with Eigen
		 */
		struct CostFunction : Functor<double>
		{
			CostFunction(OptimizationModel* model) : Functor<double>(0, 0),
					model_(model) {}
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

		/** @brief Initializes the optimization model, i.e. the dimensions of
		 * the optimization vectors */
		virtual void init(bool only_soft_constraints = false);

		/** @brief Sets the dimension of the decision variables, here called
		 * state */
		void setDimensionOfState(unsigned int dim);

		/** @brief Sets the dimension of the constraints variables */
		void setDimensionOfConstraints(unsigned int dim);

		/** @brief Sets the number of nonzero values of the Jacobian function */
		void setNumberOfNonzeroJacobian(unsigned int dim);

		/** @brief Sets the number of nonzero values of the Hessian function */
		void setNumberOfNonzeroHessian(unsigned int dim);

		/** @brief Sets the constraint as soft constraint, i.e. inside the
		 * cost function */
		void defineAsSoftConstraint();

		/**
		 * @brief Sets the weight for computing the soft-constraint, i.e.
		 * the associated cost
		 * @param const SoftConstraintProperties& Soft-constraints properties
		 */
		void setSoftProperties(const SoftConstraintProperties& properties);

		/**
		 * @brief Gets the starting point of the problem
		 * @param[out] decision Initial values for the decision variables, $x$
		 * @param[in] decision_dim Number of the decision variables
		 */
		virtual void getStartingPoint(double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the bounds of the problem
		 * @param[out] decision_lbound Lower bounds $x^L$ for $x$
		 * @param[in] decision_dim1 Number of decision variables (dimension of $x$)
		 * @param[out] decision_ubound Upper bounds $x^L$ for $x$
		 * @param[in] decision_dim2 Number of decision variables (dimension of $x$)
		 * @param[out] constraint_lbound Lower bounds of the constraints $g^L$ for $x$
		 * @param[in] constraint_dim1 Number of constraints (dimension of $g(x)$)
		 * @param[out] constraint_ubound Upper bounds of the constraints $g^L$ for $x$
		 * @param[in] constraint_dim2 Number of constraints (dimension of $g(x)$)
		 */
		virtual void evaluateBounds(double* decision_lbound, int decision_dim1,
									double* decision_ubound, int decision_dim2,
									double* constraint_lbound, int constraint_dim1,
									double* constraint_ubound, int constraint_dim2);

		/**
		 * @brief Abstract method for evaluating the cost function given a
		 * current decision state
		 * @param[out] cost Value of the objective function ($f(x)$).
		 * @param[in] decision Array of the decision variables, $x$, at which
		 * the cost functions, $f(x)$, is evaluated
		 * @param[in] decision_dim Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateCosts(double& cost,
								   const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the gradient of the cost
		 * function given a current decision state
		 * @param[out] gradient Array of values for the gradient of the
		 * objective function ($\nabla f(x)$)
		 * @param[in] grad_dim Number of decision variables (dimension of $x$)
		 * @param[in] decision Array for the decision variables, $x$, at which
		 * $\nabla f(x)$ is evaluated
		 * @param[in] decision_dim Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateCostGradient(double* gradient, int grad_dim,
										  const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the constraint function given a
		 * current decision state
		 * @param[out] constraint Array of constraint function values, $g(x)$
		 * @param[in] constraint_dim Number of constraint variables (dimension
		 * of $g(x)$)
		 * @param[in] decision Array of the decision variables, $x$, at which
		 * the constraint functions, $g(x)$, are evaluated
		 * @param[in] decision_dim Number of decision variables (dimension of $x$)
		 */
		virtual void evaluateConstraints(double* constraint, int constraint_dim,
								 	 	 const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the constraint function as soft
		 * one
		 * @param[in] decision Array of the decision variables, $x$, at which
		 * the constraint functions, $g(x)$, are evaluated
		 * @param[in] decision_dim Number of decision variables (dimension of $x$)
		 * @param return The soft-cost value
		 */
		virtual double evaluateAsSoftConstraints(const double* decision, int decision_dim);

		/**
		 * @brief Abstract method for evaluating the jacobian of the constraint
		 * function given a current decision state
		 * @param[out] jacobian Values of the entries in the Jacobian of the
		 * constraints
		 * @param[in] nonzero_dim1 Number of nonzero elements in the Jacobian
		 * (dimension of row_entries, col_entries, and values)
		 * @param[out] row_entries Row indices of entries in the Jacobian of
		 * the constraints
		 * @param[in] nonzero_dim2 Number of nonzero elements in the Jacobian
		 * (dimension of row_entries, col_entries, and values)
		 * @param[out] col_entries Column indices of entries in the Jacobian
		 * of the constraints
		 * @param[in] nonzero_dim3 Number of nonzero elements in the Jacobian
		 * (dimension of row_entries, col_entries, and values)
		 * @param[in] decision Array for the decision variables, $x$, at which
		 * $\nabla g(x)^T$ is evaluated
		 * @param[in] decision_dim Number of decision variables (dimension of $x$)
		 * @param[in] flag Indicate when we have to compute the constraint
		 */
		virtual void evaluateConstraintJacobian(double* jacobian_values, int nonzero_dim1,
												int* row_entries, int nonzero_dim2,
												int* col_entries, int nonzero_dim3,
												const double* decision, int decision_dim, bool flag);

		/**
		 * @brief This method returns the structure of the Hessian of the
		 * Lagrangian (if "values" is  NULL) and the values of the Hessian of
		 * the Lagrangian (if "values" is not NULL)
		 * @param[out] hessian_values Values of the entries in the Hessian
		 * @param[in] nonzero_dim1 Number of decision variables (dimension of $x$)
		 * @param[out] row_entries Row indices of entries in the Hessian
		 * @param[in] nonzero_dim2 Number of decision variables (dimension of $x$)
		 * @param[out] col_entries Column indices of entries in the Hessian
		 * @param[in] nonzero_dim3 Number of decision variables (dimension of $x$)
		 * @param[in] obj_factor Factor in front of the objective term in the
		 * Hessian, $\sigma_f$
		 * @param[in] decision Values for the primal variables, $x$, at which
		 * the Hessian is to be evaluated
		 * @param[in] lagrange Values for the constraint multipliers, $\lambda$,
		 * at which the Hessian is to be evaluated
		 * @param[in] constraint_dim Number of constraint variables (dimension of
		 * $g(x)$)
		 * @param[in] False if any evaluation method was previously called with
		 * the same values in $x$, true otherwise
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

		/** @brief Indicates is the constraint is implemented as soft-constraint */
		bool isSoftConstraint();


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

		/** @brief True if the bounds are evaluated */
		bool bounds_;

		/** @brief True if the constraint are defined as soft */
		bool soft_constraints_;

		bool first_time_;

		/** @brief Cost functor for numerical differentiation */
		CostFunction cost_function_;

		/** @brief Numerical differentiation mode */
		Eigen::NumericalDiffMode num_diff_mode_;

		/** @brief Machine epsilon constant which gives an upper bound on the relative error
		    due to rounding in floating point arithmetic */
		double epsilon_;

		/** @brief Lower and upper bound of the constraints */
		Eigen::VectorXd g_lbound_, g_ubound_;

		/** @brief Soft-constraints properties */
		SoftConstraintProperties soft_properties_;
};

} //@namespace model
} //@namespace dwl


#endif
