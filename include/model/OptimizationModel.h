#ifndef DWL__MODEL__OPTIMIZATION_MODEL__H
#define DWL__MODEL__OPTIMIZATION_MODEL__H

#include <model/DynamicalSystem.h>
#include <model/Constraint.h>
#include <model/Cost.h>
#include <unsupported/Eigen/NumericalDiff>
#include <utils/utils.h>


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
		~OptimizationModel();

		/**
		 * @brief Ges the starting point of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full initial point
		 */
		void getStartingPoint(Eigen::Ref<Eigen::VectorXd> full_initial_point);

		/**
		 * @brief Evaluates the bounds of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full state lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full state upper bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint upper bound
		 */
		void evaluateBounds(Eigen::Ref<Eigen::VectorXd> full_state_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_state_upper_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_upper_bound);
		/**
		 * @brief Evaluates the constraint function given a current decision state
		 * @param Eigen::Ref<Eigen::VectorXd> Constraint vector
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
								 const Eigen::Ref<const Eigen::VectorXd>& decision_var);

		/**
		 * @brief Evaluates the cost function given a current decision state
		 * @param double& Cost value
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateCosts(double& cost,
						   const Eigen::Ref<const Eigen::VectorXd>& decision_var);

		/**
		 * @brief Evaluates the jacobian of the constraint function given a current decision
		 * state
		 * @param Eigen::MatrixXd& Jacobian of the constraint function
		 * @param const Eigen::VectorXd& Decision vector
		 */
		void evaluateConstraintJacobian(Eigen::MatrixXd& jacobian,
										const Eigen::VectorXd& decision_var);

		/**
		 * @brief Evaluates the gradient of the cost function given a current decision
		 * state
		 * @param Eigen::MatrixXd& Gradient of the cost function
		 * @param const Eigen::VectorXd& Decision vector
		 */
		void evaluateCostGradient(Eigen::MatrixXd& gradient,
								  const Eigen::VectorXd& decision_var);

		/**
		 * @brief Evaluates the solution from an optimizer
		 * @param const Eigen::Ref<const Eigen::VectorXd>& Solution vector
		 * @return std::vector<LocomotionState>& Returns the locomotion trajectory solution
		 */
		std::vector<LocomotionState>& evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution);

		/**
		 * @brief Adds the dynamical system (active constraints) to the optimization problem
		 * @param DynamicalSystem* Dynamical system constraint to add it
		 */
		void addDynamicalSystem(DynamicalSystem* dynamical_system);

		/** @brief Removes the current dynamical system constraint */
		void removeDynamicalSystem();

		/**
		 * @brief Adds an active or inactive constraints to the optimization problem
		 * @param Constraint* Constraint to add it
		 */
		void addConstraint(Constraint* constraint);

		/**
		 * @brief Removes an active or inactive constraints to the optimization problem
		 * @param Constraint* Constraint to remove it
		 */
		void removeConstraint(std::string constraint_name);

		/**
		 * @brief Adds a cost function for the optimization problem
		 * @param Cost* Cost to add it
		 */
		void addCost(Cost* cost);

		/**
		 * @brief Removes a cost function for the optimization problem
		 * @param Cost* Cost to remove it
		 */
		void removeCost(std::string cost_name);

		/**
		 * @brief Sets the horizon steps
		 * @param unsigned int Horizon
		 */
		void setHorizon(unsigned int horizon);

		/**
		 * @brief Configures the numerical differentiation algorithm
		 * @param enum Eigen::NumericalDiffMode Numerical differentiation mode, currently there
		 * are two method: forward and central differentiation
		 * @param double Machine epsilon constant
		 */
		void configureNumericalDifferentiation(enum Eigen::NumericalDiffMode,
											   double epsilon = 1E-06);

		/** @brief Gets the dynamical system constraint */
		DynamicalSystem* getDynamicalSystem();

		/** @brief Gets the active and inactive constraints, which not include the dynamical one */
		std::vector<Constraint*> getConstraints();

		/** @brief Gets the cost functions */
		std::vector<Cost*> getCosts();

		/** @brief Gets the dimension of the state vector of the optimization problem */
		unsigned int getDimensionOfState();

		/** @brief Gets the dimension of the constraints of the optimization problem */
		unsigned int getDimensionOfConstraints();

		/** @brief Gets the horizon value of the optimization problem */
		const unsigned int& getHorizon();


	protected:
		/** @brief Dynamical system constraint pointer */
		DynamicalSystem* dynamical_system_;

		/** @brief Vector of active and inactive constraints pointers */
		std::vector<Constraint*> constraints_;

		/** @brief Vector of costs pointers */
		std::vector<Cost*> costs_;

		/** @brief Constraint functor for numerical differentiation */
		ConstraintFunction constraint_function_;

		/** @brief Cost functor for numerical differentiation */
		CostFunction cost_function_;

		/** @brief Numerical differentiation mode */
		Eigen::NumericalDiffMode num_diff_mode_;

		/** @brief Locomotion solution */
		std::vector<LocomotionState> locomotion_solution_;

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

		/** @brief Indicates if it was added an active constraint in the solver */
		bool is_added_dynamic_system_;

		/** @brief Indicates if it was added a constraint in the solver */
		bool is_added_constraint_;

		/** @brief Indicates if it was added a cost in the solver */
		bool is_added_cost_;
};

} //@namespace model
} //@namespace dwl


#endif
