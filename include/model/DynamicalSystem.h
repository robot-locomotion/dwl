#ifndef DWL__MODEL__DYNAMICAL_SYSTEM__H
#define DWL__MODEL__DYNAMICAL_SYSTEM__H

#include <model/Constraint.h>


namespace dwl
{

namespace model
{

/** @brief Defines the locomotion variables of the optimization problem */
struct LocomotionVariables
{
	LocomotionVariables(bool full_opt = false) : time(full_opt), position(full_opt),
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
		 * @param std::string URDF model
		 * @param Print model information
		 */
		void init(std::string urdf_model,
				  bool info);

		/** @brief Initializes the dynamical system properties */
		virtual void initDynamicalSystem();

		/**
		 * @brief Reads and sets the joint limit from an URDF model
		 * @param std::string URDF model
		 */
		void jointLimitsFromURDF(std::string urdf_model);

		/**
		 * @brief Computes the dynamical and time integration constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const LocomotionState& state);

		/**
		 * @brief Computes the dynamical constraint vector given a certain state. Note that the
		 * time integration of the dynamical system is computed in the available numerical
		 * integration methods implemented in compute() function.
		 * @param Eigen::VectorXd& Evaluated the dynamical constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void computeDynamicalConstraint(Eigen::VectorXd& constraint,
				 	 	 	 	 	 	 	 	const LocomotionState& state);

		/**
		 * @brief Computes the terminal constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated the terminal constraint function
		 * @param const LocomotionState& State vector
		 */
		void computeTerminalConstraint(Eigen::VectorXd& constraint,
									   const LocomotionState& state);

		/**
		 * @brief Computes the constraint from the time integration. Additionally, it's updated
		 * the time value in case of fixed-step integration, i.e. optimization without time as a
		 * decision variable. Note that there are different numerical integration methods
		 * @param Eigen::VectorXd& Evaluated the dynamical constraint function
		 * @param const LocomotionState& State vector
		 */
		void numericalIntegration(Eigen::VectorXd& constraint,
								  const LocomotionState& state);

		/**
		 * @brief Gets the bounds of the dynamical system constraint which included the time
		 * integration bounds
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		virtual void getBounds(Eigen::VectorXd& lower_bound,
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
		 * @param const LocomotionState& Lower state bounds
		 * @param const LocomotionState& Upper state bounds
		 */
		void setStateBounds(const LocomotionState& lower_bound,
							const LocomotionState& upper_bound);

		/**
		 * @brief Sets the initial state of the dynamical constraint
		 * @param const LocomotionState& Initial state
		 */
		void setInitialState(const LocomotionState& initial_state);

		/**
		 * @brief Sets the terminal state
		 * @param const LocomotionState& Terminal state
		 */
		void setTerminalState(const LocomotionState& terminal_state);

		/**
		 * @brief Sets the starting state
		 * @param const LocomotionState& Starting state
		 */
		void setStartingState(const LocomotionState& starting_state);

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
		 * @param LocomotionState& Lower state bounds
		 * @param LocomotionState& Upper state bounds
		 */
		void getStateBounds(LocomotionState& lower_bound,
							LocomotionState& upper_bound);

		/** @brief Gets the initial state of the dynamical constraint */
		const LocomotionState& getInitialState();

		/** @brief Gets the terminal state of the dynamical constraint */
		const LocomotionState& getTerminalState();

		/** @brief Gets the starting state */
		const LocomotionState& getStartingState();

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
		 * @brief Converts the generalized state vector to locomotion state
		 * @param LocomotionState& Locomotion state
		 * @param const Eigen::VectorXd& Generalized state vector
		 */
		void toLocomotionState(LocomotionState& state_model,
							   const Eigen::VectorXd& generalized_state);

		/**
		 * @brief Converts the locomotion state to generalized state vector
		 * @param Eigen::VectorXd& Generalized state vector
		 * @param const LocomotionState& Locomotion state
		 */
		void fromLocomotionState(Eigen::VectorXd& generalized_state,
								 const LocomotionState& state_model);

		/** @brief Returns true if it's a fixed-step integration */
		bool isFixedStepIntegration();

		/** @brief Returns true if it's a full-trajectory optimization problem */
		bool isFullTrajectoryOptimization();


	protected:
		/** @brief Dimension of the dynamical state */
		unsigned int state_dimension_;

		/** @brief Dimension of the terminal constraint */
		unsigned int terminal_constraint_dimension_;

		/** @brief Initial state */
		LocomotionState initial_state_;

		/** @brief Terminal state */
		LocomotionState terminal_state_;

		/** @brief Starting state uses from optimizers */
		LocomotionState starting_state_;

		/** @brief Lower state bounds */
		LocomotionState lower_state_bound_;

		/** @brief Upper state bounds */
		LocomotionState upper_state_bound_;

		/** @brief Locomotion variables defined given a dynamical system constraint */
		LocomotionVariables locomotion_variables_;

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
