#ifndef DWL__MODEL__DYNAMICAL_SYSTEM__H
#define DWL__MODEL__DYNAMICAL_SYSTEM__H

#include <model/Constraint.h>
#include <model/WholeBodyDynamics.h>


namespace dwl
{

namespace model
{

struct LocomotionVariables
{
	LocomotionVariables(bool full_opt = false) : time(full_opt), position(full_opt),
			velocity(full_opt),	acceleration(full_opt), effort(full_opt),
			contact_pos(full_opt), contact_for(full_opt) {}
	bool time;
	bool position;
	bool velocity;
	bool acceleration;
	bool effort;
	bool contact_pos;
	bool contact_for;
};

class DynamicalSystem : public Constraint
{
	public:
		/** @brief Constructor function */
		DynamicalSystem();

		/** @brief Destructor function */
		virtual ~DynamicalSystem();

		/**
		 * @brief Build the model rigid-body system from an URDF file
		 * @param std::string URDF file
		 * @param struct rbd::FloatingBaseSystem* Defines the general properties of a floating-base
		 * system
		 * @param Print model information
		 */
		void modelFromURDFFile(std::string urdf_model,
							   struct rbd::FloatingBaseSystem* system = NULL,
							   bool info = false);

		/**
		 * @brief Build the model rigid-body system from an URDF model (xml)
		 * @param std::string URDF model
		 * @param struct rbd::FloatingBaseSystem* Defines the general properties of a floating-base
		 * system
		 * @param Print model information
		 */
		void modelFromURDFModel(std::string urdf_model,
								struct rbd::FloatingBaseSystem* system = NULL,
								bool info = false);

		/**
		 * @brief Computes the dynamic constraint vector given a certain state
		 * @param Eigen::VectorXd& Evaluated constraint function
		 * @param const LocomotionState& State vector
		 */
		virtual void compute(Eigen::VectorXd& constraint,
							 const LocomotionState& state) = 0;

		/**
		 * @brief Gets the bounds of the dynamical system constraint
		 * @param Eigen::VectorXd& Lower bounds
		 * @param Eigen::VectorXd& Upper bounds
		 */
		virtual void getBounds(Eigen::VectorXd& lower_bound,
							   Eigen::VectorXd& upper_bound) = 0;

		/**
		 * @brief Sets floating-base system properties
		 * @param rbd::FloatingBaseSystem* system
		 */
		void setFloatingBaseSystem(rbd::FloatingBaseSystem* system);

		/**
		 * @brief Sets the lower and upper state bounds
		 * @param const LocomotionState& Lower state bounds
		 * @param const LocomotionState& Upper state bounds
		 */
		void setStateBounds(const LocomotionState& lower_bound,
							const LocomotionState& upper_bound);

		/**
		 * @brief Sets the starting state
		 * @param const LocomotionState& Starting state
		 */
		void setStartingState(const LocomotionState& starting_state);

		/**
		 * @brief Gets the lower and upper state bounds
		 * @param LocomotionState& Lower state bounds
		 * @param LocomotionState& Upper state bounds
		 */
		void getStateBounds(LocomotionState& lower_bound,
							LocomotionState& upper_bound);

		/**
		 * @brief Gets the starting state
		 * @param LocomotionState& Starting state
		 */
		void getStartingState(LocomotionState& starting_state);

		/** @brief Gets the dimension of the dynamical state */
		unsigned int getDimensionOfState();

		/** @brief Gets the number of end-effectors */
		unsigned int getNumberOfEndEffectors();

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


	protected:
		/** @brief Whole-body dynamical model */
		WholeBodyDynamics dynamics_;

		/** @brief A floating-base system definition */
		rbd::FloatingBaseSystem* system_;

		/** @brief Dimension of the dynamical state */
		unsigned int state_dimension_;

		/** @brief Number of end-effectors */
		unsigned int num_endeffectors_;

		/** @brief Degree of freedom of the floating-base system */
		unsigned int system_dof_;

		/** @brief Degree of freedom of the joint */
		unsigned int joint_dof_;

		/** @brief Starting state uses from optimizers */
		LocomotionState starting_state_;

		/** @brief Lower state bounds */
		LocomotionState lower_state_bound_;

		/** @brief Upper state bounds */
		LocomotionState upper_state_bound_;

		LocomotionVariables locomotion_variables_;
};

} //@namespace model
} //@namespace dwl

#endif
