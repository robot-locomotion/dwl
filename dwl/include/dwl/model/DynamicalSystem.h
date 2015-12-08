#ifndef DWL__MODEL__DYNAMICAL_SYSTEM__H
#define DWL__MODEL__DYNAMICAL_SYSTEM__H

#include <dwl/model/Constraint.h>


namespace dwl
{

namespace model
{

/** @brief Defines the whole-body variables of the optimization problem */
struct WholeBodyVariables
{
	WholeBodyVariables(bool full_opt = false) : time(full_opt), position(full_opt),
			velocity(full_opt),	acceleration(full_opt), effort(full_opt),
			contact_pos(full_opt), contact_vel(full_opt), contact_acc(full_opt),
			contact_for(full_opt) {}
	bool time;
	bool position;
	bool velocity;
	bool acceleration;
	bool effort;
	bool contact_pos;
	bool contact_vel;
	bool contact_acc;
	bool contact_for;
};

/** @brief Defines the different methods for step-time integration */
enum StepIntegrationMethod {Fixed, Variable};

/**
 * @class DynamicalSystem
 * @brief This abstract class defines common methods for implementing dynamical system constraint.
 * Additionally, this class has different facilities for defining the state bounds (joint limits),
 * initial state, starting state, state dimension, etc. Some of these information are extracted
 * from an URDF model
 */
class DynamicalSystem : public Constraint
{
	public:
		/** @brief Constructor function */
		DynamicalSystem();

		/** @brief Destructor function */
		virtual ~DynamicalSystem();

		/**
		 * @brief Initializes the dynamical system constraint given an URDF model (xml)
		 * @param Print model information
		 */
		void init(bool info);

		/** @brief Initializes the dynamical system properties */
		virtual void initDynamicalSystem();

		/** @brief Reads and sets the joint limit from an URDF model */
		void jointLimitsFromURDF();

		/**
		 * @brief Computes the dynamical and time integration constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void compute(Eigen::VectorXd& constraint,
					 const WholeBodyState& state);

		/**
		 * @brief Computes the dynamical constraint vector given a certain state. Note that the
		 * time integration of the dynamical system is computed in the available numerical
		 * integration methods implemented in compute() function.
		 * @param Eigen::VectorXd& Evaluated the dynamical constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		virtual void computeDynamicalConstraint(Eigen::VectorXd& constraint,
				 	 	 	 	 	 	 	 	const WholeBodyState& state);

		/**
		 * @brief Computes the terminal constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated the terminal constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void computeTerminalConstraint(Eigen::VectorXd& constraint,
									   const WholeBodyState& state);

		/**
		 * @brief Computes the constraint from the time integration. Additionally, it's updated
		 * the time value in case of fixed-step integration, i.e. optimization without time as a
		 * decision variable. Note that there are different numerical integration methods
		 * @param Eigen::VectorXd& Evaluated the dynamical constraint function
		 * @param const WholeBodyState& Whole-body state
		 */
		void numericalIntegration(Eigen::VectorXd& constraint,
								  const WholeBodyState& state);

		/**
		 * @brief Gets the bounds of the dynamical system constraint which included the time
		 * integration bounds
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		void getBounds(Eigen::VectorXd& lower_bound,
					   Eigen::VectorXd& upper_bound);

		/**
		 * @brief Gets the dynamical system bounds vector given a certain state. Note that the
		 * time integration bounds of the dynamical system are get according the available
		 * numerical integration methods implemented in compute() function.
		 * @param Eigen::VectorXd& Lower bounds of the dynamical system
		 * @param Eigen::VectorXd& Upper bounds of the dynamical system
		 */
		virtual void getDynamicalBounds(Eigen::VectorXd& lower_bound,
										Eigen::VectorXd& upper_bound);

		/**
		 * @brief Gets the terminal bounds vector given a certain state
		 * @param Eigen::VectorXd& Lower bounds of the dynamical system
		 * @param Eigen::VectorXd& Upper bounds of the dynamical system
		 */
		void getTerminalBounds(Eigen::VectorXd& lower_bound,
							   Eigen::VectorXd& upper_bound);

		/**
		 * @brief Sets the lower and upper state bounds
		 * @param const WholeBodyState& Lower whole-body state bounds
		 * @param const WholeBodyState& Upper whole-body state bounds
		 */
		void setStateBounds(const WholeBodyState& lower_bound,
							const WholeBodyState& upper_bound);

		/**
		 * @brief Sets the initial state of the dynamical constraint
		 * @param const WholeBodyState& Initial whole-body state
		 */
		void setInitialState(const WholeBodyState& initial_state);

		/**
		 * @brief Sets the terminal state
		 * @param const WholeBodyState& Terminal whole-body state
		 */
		void setTerminalState(const WholeBodyState& terminal_state);

		/**
		 * @brief Sets the step integration method (fixed or variable). The default value is fixed
		 * @param StepIntegrationMethod Step integration method
		 */
		void setStepIntegrationMethod(StepIntegrationMethod method);

		/**
		 * @brief Sets the fixed-step integration time
		 * @param const double& Fixed-step integration time
		 */
		void setStepIntegrationTime(const double& step_time);

		/** @brief Gets the kinematics of the system */
		WholeBodyKinematics& getKinematics();

		/** @brief Gets the dynamics of the system */
		WholeBodyDynamics& getDynamics();

		/**
		 * @brief Gets the lower and upper state bounds
		 * @param WholeBodyState& Lower whole-body state bounds
		 * @param WholeBodyState& Upper whole-body state bounds
		 */
		void getStateBounds(WholeBodyState& lower_bound,
							WholeBodyState& upper_bound);

		/** @brief Gets the initial whole-body state of the dynamical constraint */
		const WholeBodyState& getInitialState();

		/** @brief Gets the terminal whole-body state of the dynamical constraint */
		const WholeBodyState& getTerminalState();

		/** @brief Gets the dimension of the dynamical state */
		unsigned int getDimensionOfState();

		/** @brief Gets the dimension of the terminal constraint */
		unsigned int getTerminalConstraintDimension();

		/** @brief Gets the floating-base system information */
		FloatingBaseSystem& getFloatingBaseSystem();

		/** @brief Gets the fixed-step time of integration */
		const double& getFixedStepTime();

		/** @brief Sets the problem as full-trajectory optimization, which means to add
		 * a the terminal constraint to the optimization problem */
		void setFullTrajectoryOptimization();

		/**
		 * @brief Converts the generalized state vector to whole-body state
		 * @param WholeBodyState& Whole-body state
		 * @param const Eigen::VectorXd& Generalized state vector
		 */
		void toWholeBodyState(WholeBodyState& system_state,
							  const Eigen::VectorXd& generalized_state);

		/**
		 * @brief Converts the whole-body state to generalized state vector
		 * @param Eigen::VectorXd& Generalized state vector
		 * @param const WholeBodyState& Whole-body state
		 */
		void fromWholeBodyState(Eigen::VectorXd& generalized_state,
								const WholeBodyState& system_state);

		/** @brief Returns true if it's a fixed-step integration */
		bool isFixedStepIntegration();

		/** @brief Returns true if it's a full-trajectory optimization problem */
		bool isFullTrajectoryOptimization();


	protected:
		/** @brief Dimension of the dynamical state */
		unsigned int state_dimension_;

		/** @brief Dimension of the terminal constraint */
		unsigned int terminal_constraint_dimension_;

		/** @brief Initial whole-body state */
		WholeBodyState initial_state_;

		/** @brief Terminal whole-body state */
		WholeBodyState terminal_state_;

		/** @brief Lower whole-body state bounds */
		WholeBodyState lower_state_bound_;

		/** @brief Upper whole-body state bounds */
		WholeBodyState upper_state_bound_;

		/** @brief Whole-body variables defined given a dynamical system constraint */
		WholeBodyVariables system_variables_;

		/** @brief Step integration method */
		StepIntegrationMethod integration_method_;

		/** @brief Fixed-step time value [in seconds] */
		double step_time_;


	private:
		/** @brief Computes the state dimension of the dynamical constraint */
		void computeStateDimension();

		/** @brief Initializes conditions of the dynamical constraint */
		void initialConditions();

		/** @brief Indicates if it's a full-trajectory optimization */
		bool is_full_trajectory_optimization_;
};

} //@namespace model
} //@namespace dwl

#endif
