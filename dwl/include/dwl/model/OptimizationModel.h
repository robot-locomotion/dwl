#ifndef DWL__MODEL__OPTIMIZATION_MODEL__H
#define DWL__MODEL__OPTIMIZATION_MODEL__H

#include <unsupported/Eigen/NumericalDiff>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class OptimizationModel
 * @brief A NLP problem requires information of constraints (dynamical, active or inactive) and
 * cost functions. Thus, OptimizationModel is a class that allows us to implement different locomotion
 * (control and planning) models. For instance, we can defined different decision
 * variables (time; base and joint position, velocity and acceleration; contact forces), i.e.
 * the description of the state vector by implementing convertDecisionVariablesToStateModel method.
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
		 * @brief Describes the constraint function requires for computing numerical
		 * differentiation (Jacobian computation) with Eigen
		 */
		struct ConstraintFunction : Functor<double>
		{
			ConstraintFunction(model::OptimizationModel* model) : Functor<double>(0,0),
					model_(model) {}
			int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& g) const
			{
				g.resize(model_->getDimensionOfConstraints());
				model_->evaluateConstraints(g, x);

				return 0;
			}

			model::OptimizationModel* const model_;
		};

		/**
		 * @brief Describes the cost function requires for computing numerical
		 * differentiation (gradient computation) with Eigen
		 */
		struct CostFunction : Functor<double>
		{
			CostFunction(model::OptimizationModel* model) : Functor<double>(0,0), model_(model) {}
			int operator() (const Eigen::VectorXd& x, Eigen::VectorXd& f) const
			{
				f.resize(1);
				double cost;
				model_->evaluateCosts(cost, x);
				f(0) = cost;

				return 0;
			}

			model::OptimizationModel* const model_;
		};

		/** @brief Constructor function */
		OptimizationModel();

		/** @brief Destructor function */
		virtual ~OptimizationModel();

		/** @brief Initializes the optimization model, i.e. the dimensions of the optimization
		 * vectors */
		virtual void init(bool only_soft_constraints = false) = 0;

		/**
		 * @brief Gets the starting point of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full initial point
		 */
		virtual void getStartingPoint(Eigen::Ref<Eigen::VectorXd> full_initial_point) = 0;

		/**
		 * @brief Pure abstract method for evaluating the bounds of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full state lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full state upper bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint upper bound
		 */
		virtual void evaluateBounds(Eigen::Ref<Eigen::VectorXd> full_state_lower_bound,
									Eigen::Ref<Eigen::VectorXd> full_state_upper_bound,
									Eigen::Ref<Eigen::VectorXd> full_constraint_lower_bound,
									Eigen::Ref<Eigen::VectorXd> full_constraint_upper_bound) = 0;
		/**
		 * @brief Pure abstract method for evaluating the constraint function given a
		 * current decision state
		 * @param Eigen::Ref<Eigen::VectorXd> Constraint vector
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		virtual void evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
								 	 	 const Eigen::Ref<const Eigen::VectorXd>& decision_var) = 0;

		/**
		 * @brief Pure abstract method for evaluating the cost function given a current
		 * decision state
		 * @param double& Cost value
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		virtual void evaluateCosts(double& cost,
								   const Eigen::Ref<const Eigen::VectorXd>& decision_var) = 0;

		/**
		 * @brief Evaluates the jacobian of the constraint function given a current decision
		 * state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const Eigen::VectorXd& Decision vector
		 */
		virtual void evaluateConstraintJacobian(Eigen::MatrixXd& jacobian,
												const Eigen::VectorXd& decision_var);

		/**
		 * @brief Evaluates the gradient of the cost function given a current decision
		 * state
		 * @param Eigen::MatrixXd& Gradient of the cost function
		 * @param const Eigen::VectorXd& Decision vector
		 */
		virtual void evaluateCostGradient(Eigen::MatrixXd& gradient,
										  const Eigen::VectorXd& decision_var);

		/**
		 * @brief Evaluates the solution from an optimizer
		 * @param const Eigen::Ref<const Eigen::VectorXd>& Solution vector
		 * @return WholeBodyTrajectory& Returns the whole-body trajectory solution
		 */
		virtual WholeBodyTrajectory& evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution) = 0;

		/**
		 * @brief Configures the numerical differentiation algorithm
		 * @param enum Eigen::NumericalDiffMode Numerical differentiation mode, currently there
		 * are two method: forward and central differentiation
		 * @param double Machine epsilon constant
		 */
		void configureNumericalDifferentiation(enum Eigen::NumericalDiffMode,
											   double epsilon = 1E-06);

		/** @brief Gets the dimension of the state vector of the optimization problem */
		unsigned int getDimensionOfState();

		/** @brief Gets the dimension of the constraints of the optimization problem */
		unsigned int getDimensionOfConstraints();


	protected:
		/** @brief Constraint functor for numerical differentiation */
		ConstraintFunction constraint_function_;

		/** @brief Cost functor for numerical differentiation */
		CostFunction cost_function_;

		/** @brief Numerical differentiation mode */
		Eigen::NumericalDiffMode num_diff_mode_;

		/** @brief Whole-body solution */
		WholeBodyTrajectory motion_solution_;

		/** @brief Dimension of the state vector at instantaneous point */
		unsigned int state_dimension_;

		/** @brief Dimension of the constraint vector at instantaneous point */
		unsigned int constraint_dimension_;

		/** @brief Dimension of the terminal constraint vector */
		unsigned int terminal_constraint_dimension_;

		/** @brief Horizon of the optimal control problem */
		unsigned int horizon_;

		/** @brief Machine epsilon constant which gives an upper bound on the relative error
		    due to rounding in floating point arithmetic */
		double epsilon_;
};

} //@namespace model
} //@namespace dwl


#endif
