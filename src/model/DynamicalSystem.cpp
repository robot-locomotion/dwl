#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : state_dimension_(0), locomotion_variables_(false),
		step_time_(0.1)
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::init(std::string urdf_model,
						   bool info)
{
	// Setting initial conditions
	initialConditions();

	// Reading and setting the joint limits
	jointLimitsFromURDF(urdf_model);

	// Initializing the dynamical system
	initDynamicalSystem();

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", system_.getSystemDoF());
		printf("The joint DOF of floating-base system is %i\n", system_.getJointDoF());
	}
}


void DynamicalSystem::initDynamicalSystem()
{

}


void DynamicalSystem::jointLimitsFromURDF(std::string urdf_model)
{
	// Reading the joint limits
	Eigen::VectorXd lower_joint_pos, upper_joint_pos, joint_vel, joint_eff;
	urdf_model::getJointLimits(lower_joint_pos, upper_joint_pos,
							   joint_vel, joint_eff, urdf_model);

	// Setting the joint limits
	lower_state_bound_.joint_pos = lower_joint_pos;
	lower_state_bound_.joint_vel = -joint_vel;
	lower_state_bound_.joint_eff = -joint_eff;
	upper_state_bound_.joint_pos = upper_joint_pos;
	upper_state_bound_.joint_vel = joint_vel;
	upper_state_bound_.joint_eff = joint_eff;
}


void DynamicalSystem::compute(Eigen::VectorXd& constraint,
							  const LocomotionState& state)
{
	// Evaluating the numerical integration
	Eigen::VectorXd time_constraint;
	LocomotionState updated_state = state;
	numericalIntegration(time_constraint, updated_state);

	// Computing the dynamical constraint
	Eigen::VectorXd dynamical_constraint;
	computeDynamicalConstraint(dynamical_constraint, updated_state);

	// Adding both constraints
	unsigned int dynamical_dim = dynamical_constraint.size();
	constraint.resize(system_.getSystemDoF() + dynamical_dim);
	constraint.segment(0, system_.getSystemDoF()) = time_constraint;
	constraint.segment(system_.getSystemDoF(), dynamical_dim) = dynamical_constraint;
}


void DynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
												 const LocomotionState& state)
{
	printf(RED "FATAL: the dynamical constraint was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::numericalIntegration(Eigen::VectorXd& constraint,
										   LocomotionState& state)
{
	// Resizing the constraint vector
	constraint.resize(system_.getSystemDoF());

	// Transcription of the constrained inverse dynamic equation using Euler-backward integration.
	// This integration method adds numerical stability
	Eigen::VectorXd base_int = last_state_.base_pos - state.base_pos + step_time_ * state.base_vel;
	Eigen::VectorXd joint_int = last_state_.joint_pos - state.joint_pos +	step_time_ * state.joint_vel;
	constraint = system_.toGeneralizedJointState(base_int, joint_int);

	// Updating the time state
	state.time += step_time_;
}


void DynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
								Eigen::VectorXd& upper_bound)
{
	Eigen::VectorXd time_bound = Eigen::VectorXd::Zero(system_.getSystemDoF());

	// Getting the dynamical bounds
	Eigen::VectorXd dynamical_lower_bound, dynamical_upper_bound;
	getDynamicalBounds(dynamical_lower_bound, dynamical_upper_bound);

	// Adding both bounds
	unsigned int dynamical_dim = dynamical_lower_bound.size();
	lower_bound.resize(system_.getSystemDoF() + dynamical_dim);
	upper_bound.resize(system_.getSystemDoF() + dynamical_dim);
	lower_bound.segment(0, system_.getSystemDoF()) = time_bound;
	lower_bound.segment(system_.getSystemDoF(), dynamical_dim) = dynamical_lower_bound;
	upper_bound.segment(0, system_.getSystemDoF()) = time_bound;
	upper_bound.segment(system_.getSystemDoF(), dynamical_dim) = dynamical_upper_bound;
}


void DynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
										 Eigen::VectorXd& upper_bound)
{
	printf(RED "FATAL: the dynamical bounds was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::setStartingState(const LocomotionState& starting_state)
{
	starting_state_ = starting_state;
}


void DynamicalSystem::setStepIntegrationTime(const double& step_time)
{
	step_time_ = step_time;
}


WholeBodyKinematics& DynamicalSystem::getKinematics()
{
	return kinematics_;
}


WholeBodyDynamics& DynamicalSystem::getDynamics()
{
	return dynamics_;
}


void DynamicalSystem::setStateBounds(const LocomotionState& lower_bound,
									 const LocomotionState& upper_bound)
{
	lower_state_bound_ = lower_bound;
	upper_state_bound_ = upper_bound;
}


void DynamicalSystem::setInitialState(const LocomotionState& initial_state)
{
	initial_state_ = initial_state;
}


const LocomotionState& DynamicalSystem::getStartingState()
{
	return starting_state_;
}


void DynamicalSystem::getStateBounds(LocomotionState& lower_bound,
									 LocomotionState& upper_bound)
{
	lower_bound = lower_state_bound_;
	upper_bound = upper_state_bound_;
}


const LocomotionState& DynamicalSystem::getInitialState()
{
	return initial_state_;
}


unsigned int DynamicalSystem::getDimensionOfState()
{
	return state_dimension_;
}


FloatingBaseSystem& DynamicalSystem::getFloatingBaseSystem()
{
	return system_;
}


const double& DynamicalSystem::getFixedStepTime()
{
	return step_time_;
}

void DynamicalSystem::toLocomotionState(LocomotionState& locomotion_state,
										const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint dimensions
	locomotion_state.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	locomotion_state.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	locomotion_state.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	locomotion_state.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		locomotion_state.time = generalized_state(idx);
		++idx;
	}
	if (locomotion_variables_.position) {
		system_.fromGeneralizedJointState(locomotion_state.base_pos,
										  locomotion_state.joint_pos,
										  (Eigen::VectorXd) generalized_state.segment(idx, system_.getSystemDoF()));
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.velocity) {
		system_.fromGeneralizedJointState(locomotion_state.base_vel,
										  locomotion_state.joint_vel,
										  (Eigen::VectorXd) generalized_state.segment(idx, system_.getSystemDoF()));
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.acceleration) {
		system_.fromGeneralizedJointState(locomotion_state.base_acc,
										  locomotion_state.joint_acc,
										  (Eigen::VectorXd) generalized_state.segment(idx, system_.getSystemDoF()));
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		FloatingBaseSystem fake_system(false, system_.getJointDoF());
		fake_system.fromGeneralizedJointState(locomotion_state.base_eff,
											  locomotion_state.joint_eff,
											  (Eigen::VectorXd) generalized_state.segment(idx, system_.getJointDoF()));
		idx += system_.getJointDoF();
	}
	if (locomotion_variables_.contact_pos || locomotion_variables_.contact_vel ||
			locomotion_variables_.contact_acc || locomotion_variables_.contact_for) {
		locomotion_state.contacts.resize(system_.getNumberOfEndEffectors());
		for (unsigned int i = 0; i < system_.getNumberOfEndEffectors(); i++) {
			locomotion_state.contacts[i].end_effector = i;
			if (locomotion_variables_.contact_pos) {
				locomotion_state.contacts[i].position = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (locomotion_variables_.contact_vel) {
				locomotion_state.contacts[i].velocity = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (locomotion_variables_.contact_acc) {
				locomotion_state.contacts[i].acceleration = generalized_state.segment<3>(idx);
				idx += 3;
			}
			if (locomotion_variables_.contact_for) {
				locomotion_state.contacts[i].force = generalized_state.segment<3>(idx);
				idx += 3;
			}
		}
	}
}


void DynamicalSystem::fromLocomotionState(Eigen::VectorXd& generalized_state,
										  const LocomotionState& locomotion_state)
{
	// Resizing the generalized state vector
	generalized_state.resize(state_dimension_);

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		generalized_state(idx) = locomotion_state.time;
	}
	if (locomotion_variables_.position) {
		generalized_state.segment(idx, system_.getSystemDoF()) =
				system_.toGeneralizedJointState(locomotion_state.base_pos,
												locomotion_state.joint_pos);
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.velocity) {
		generalized_state.segment(idx, system_.getSystemDoF()) =
				system_.toGeneralizedJointState(locomotion_state.base_vel,
												locomotion_state.joint_vel);
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.acceleration) {
		generalized_state.segment(idx, system_.getSystemDoF()) =
				system_.toGeneralizedJointState(locomotion_state.base_acc,
												locomotion_state.joint_acc);
		idx += system_.getSystemDoF();
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		FloatingBaseSystem fake_system(false, system_.getJointDoF());
		generalized_state.segment(idx, system_.getJointDoF()) =
				fake_system.toGeneralizedJointState(locomotion_state.base_eff,
											 	 	locomotion_state.joint_eff);
		idx += system_.getJointDoF();
	}
	if (locomotion_variables_.contact_pos || locomotion_variables_.contact_vel ||
			locomotion_variables_.contact_acc || locomotion_variables_.contact_for) {
		if (locomotion_state.contacts.size() != system_.getNumberOfEndEffectors()) {
			printf(RED "FATAL: the number of contact and end-effectors are not consistent\n" COLOR_RESET);
			exit(EXIT_FAILURE);
		}

		for (unsigned int i = 0; i < system_.getNumberOfEndEffectors(); i++) {
			if (locomotion_variables_.contact_pos) {
				generalized_state.segment<3>(idx) = locomotion_state.contacts[i].position;
				idx += 3;
			}
			if (locomotion_variables_.contact_vel) {
				generalized_state.segment<3>(idx) = locomotion_state.contacts[i].velocity;
				idx += 3;
			}
			if (locomotion_variables_.contact_acc) {
				generalized_state.segment<3>(idx) = locomotion_state.contacts[i].acceleration;
				idx += 3;
			}
			if (locomotion_variables_.contact_for) {
				generalized_state.segment<3>(idx) = locomotion_state.contacts[i].force;
				idx += 3;
			}
		}
	}
}


bool DynamicalSystem::isFixedStepIntegration()
{
	return !locomotion_variables_.time;
}


void DynamicalSystem::initialConditions()
{
	// Computing the state dimension give the locomotion variables
	state_dimension_ = (locomotion_variables_.position + locomotion_variables_.velocity
			+ locomotion_variables_.acceleration) * system_.getSystemDoF()
			+ locomotion_variables_.effort * system_.getJointDoF() + 3 *
			(locomotion_variables_.contact_pos + locomotion_variables_.contact_vel +
					locomotion_variables_.contact_acc + locomotion_variables_.contact_for) *
					system_.getNumberOfEndEffectors();


	// No-bound limit by default
	lower_state_bound_.base_pos = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_pos = -NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	lower_state_bound_.base_vel = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_vel = -NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	lower_state_bound_.base_acc = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_acc = -NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	lower_state_bound_.base_eff = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_eff = -NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	upper_state_bound_.base_pos = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_pos = NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	upper_state_bound_.base_vel = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_vel = NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	upper_state_bound_.base_acc = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_acc = NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	upper_state_bound_.base_eff = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_eff = NO_BOUND * Eigen::VectorXd::Ones(system_.getJointDoF());
	lower_state_bound_.contacts.resize(system_.getNumberOfEndEffectors());
	upper_state_bound_.contacts.resize(system_.getNumberOfEndEffectors());
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++) {
		lower_state_bound_.contacts[k].position = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contacts[k].velocity = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contacts[k].acceleration = -NO_BOUND * Eigen::Vector3d::Ones();
		lower_state_bound_.contacts[k].force = -NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contacts[k].position = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contacts[k].velocity = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contacts[k].acceleration = NO_BOUND * Eigen::Vector3d::Ones();
		upper_state_bound_.contacts[k].force = NO_BOUND * Eigen::Vector3d::Ones();
	}


	// Starting state
	starting_state_.joint_pos = Eigen::VectorXd::Zero(system_.getJointDoF());
	starting_state_.joint_vel = Eigen::VectorXd::Zero(system_.getJointDoF());
	starting_state_.joint_acc = Eigen::VectorXd::Zero(system_.getJointDoF());
	starting_state_.joint_eff = Eigen::VectorXd::Zero(system_.getJointDoF());
	starting_state_.contacts.resize(system_.getNumberOfEndEffectors());
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++) {
		starting_state_.contacts[k].end_effector = 0;
		starting_state_.contacts[k].position = Eigen::Vector3d::Zero();
		starting_state_.contacts[k].velocity = Eigen::Vector3d::Zero();
		starting_state_.contacts[k].acceleration = Eigen::Vector3d::Zero();
		starting_state_.contacts[k].force = Eigen::Vector3d::Zero();
	}
}

} //@namespace model
} //@namespace dwl
