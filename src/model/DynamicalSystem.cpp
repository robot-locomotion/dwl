#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : system_(NULL), state_dimension_(0), num_endeffectors_(1),
		system_dof_(0), joint_dof_(0), locomotion_variables_(false)
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::modelFromURDFFile(std::string filename,
										struct rbd::FloatingBaseSystem* system,
										bool info)
{
	// Initializing the kinematical and dynamical model from the filename
	kinematics_.modelFromURDFFile(filename, system, info);
	dynamics_.modelFromURDFFile(filename, system, false);
	system_ = system;

	// Setting initial conditions
	initialConditions();

	// Reading the file
	std::ifstream model_file(filename.c_str());
	if (!model_file) {
		std::cerr << "Error opening file '" << filename << "'." << std::endl;
		abort();
	}

	// Reserving memory for the contents of the file
	std::string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
			std::istreambuf_iterator<char>());
	model_file.close();

	// Reading the joint limits
	jointLimitsFromURDF(model_xml_string);

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", system_dof_);
		printf("The joint DOF of floating-base system is %i\n", joint_dof_);
	}
}


void DynamicalSystem::modelFromURDFModel(std::string urdf_model,
										 struct rbd::FloatingBaseSystem* system,
										 bool info)
{
	// Initializing the kinematical and dynamical model from the URDF model
	kinematics_.modelFromURDFModel(urdf_model, system, info);
	dynamics_.modelFromURDFModel(urdf_model, system, false);
	system_ = system;

	// Setting initial conditions
	initialConditions();

	// Reading and setting the joint limits
	jointLimitsFromURDF(urdf_model);

	if (info) {
		printf("The state dimension is %i\n", state_dimension_);
		printf("The full DOF of floating-base system is %i\n", system_dof_);
		printf("The joint DOF of floating-base system is %i\n", joint_dof_);
	}
}


void DynamicalSystem::jointLimitsFromURDF(std::string urdf_model)
{
	// Reading the joint limits
	Eigen::VectorXd lower_joint_pos, upper_joint_pos, joint_vel, joint_eff;
	urdf_model::getJointLimits(lower_joint_pos, upper_joint_pos,
							   joint_vel, joint_eff, urdf_model, system_);

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
	// Resizing the constraint vector
	Eigen::VectorXd time_constraint(system_dof_);

	// Transcription of the constrained inverse dynamic equation using Euler-backward integration.
	// This integration method adds numerical stability
	double step_time = 0.1;
	Eigen::VectorXd base_int = last_state_.base_pos - state.base_pos + step_time * state.base_vel;
	Eigen::VectorXd joint_int = last_state_.joint_pos - state.joint_pos +	step_time * state.joint_vel;
	time_constraint = rbd::toGeneralizedJointState(base_int, joint_int, system_);

	// Updating the time state
	LocomotionState updated_state = state;
	updated_state.time += step_time;

	// Computing the dynamical constraint
	Eigen::VectorXd dynamical_constraint;
	computeDynamicalConstraint(dynamical_constraint, updated_state);

	// Adding both constraints
	unsigned int dynamical_dim = dynamical_constraint.size();
	constraint.resize(system_dof_ + dynamical_dim);
	constraint.segment(0, system_dof_) = time_constraint;
	constraint.segment(system_dof_, dynamical_dim) = dynamical_constraint;
}


void DynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
												 const LocomotionState& state)
{
	printf(RED "FATAL: the dynamical constraint was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
								Eigen::VectorXd& upper_bound)
{
	Eigen::VectorXd time_bound = Eigen::VectorXd::Zero(system_dof_);

	// Getting the dynamical bounds
	Eigen::VectorXd dynamical_lower_bound, dynamical_upper_bound;
	getDynamicalBounds(dynamical_lower_bound, dynamical_upper_bound);

	// Adding both bounds
	unsigned int dynamical_dim = dynamical_lower_bound.size();
	lower_bound.resize(system_dof_ + dynamical_dim);
	upper_bound.resize(system_dof_ + dynamical_dim);
	lower_bound.segment(0, system_dof_) = time_bound;
	lower_bound.segment(system_dof_, dynamical_dim) = dynamical_lower_bound;
	upper_bound.segment(0, system_dof_) = time_bound;
	upper_bound.segment(system_dof_, dynamical_dim) = dynamical_upper_bound;
}


void DynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
										 Eigen::VectorXd& upper_bound)
{
	printf(RED "FATAL: the dynamical bounds was not implemented\n" COLOR_RESET);
	exit(EXIT_FAILURE);
}


void DynamicalSystem::setFloatingBaseSystem(rbd::FloatingBaseSystem* system)
{
	system_ = system;

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();
}


void DynamicalSystem::setStartingState(const LocomotionState& starting_state)
{
	starting_state_ = starting_state;
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


void DynamicalSystem::getStartingState(LocomotionState& starting_state)
{
	starting_state = starting_state_;
}


void DynamicalSystem::getStateBounds(LocomotionState& lower_bound,
									 LocomotionState& upper_bound)
{
	lower_bound = lower_state_bound_;
	upper_bound = upper_state_bound_;
}


void DynamicalSystem::getInitialState(LocomotionState& initial_state)
{
	initial_state = initial_state_;
}


unsigned int DynamicalSystem::getDimensionOfState()
{
	return state_dimension_;
}


unsigned int DynamicalSystem::getNumberOfEndEffectors()
{
	return num_endeffectors_;
}


void DynamicalSystem::toLocomotionState(LocomotionState& locomotion_state,
										const Eigen::VectorXd& generalized_state)
{
	// Resizing the joint dimensions
	locomotion_state.joint_pos = Eigen::VectorXd::Zero(joint_dof_);
	locomotion_state.joint_vel = Eigen::VectorXd::Zero(joint_dof_);
	locomotion_state.joint_acc = Eigen::VectorXd::Zero(joint_dof_);
	locomotion_state.joint_eff = Eigen::VectorXd::Zero(joint_dof_);

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		locomotion_state.time = generalized_state(idx);
		++idx;
	}
	if (locomotion_variables_.position) {
		rbd::fromGeneralizedJointState(locomotion_state.base_pos,
									   locomotion_state.joint_pos,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.velocity) {
		rbd::fromGeneralizedJointState(locomotion_state.base_vel,
									   locomotion_state.joint_vel,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.acceleration) {
		rbd::fromGeneralizedJointState(locomotion_state.base_acc,
									   locomotion_state.joint_acc,
									   (Eigen::VectorXd) generalized_state.segment(idx, system_dof_),
									   system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		rbd::FloatingBaseSystem fake_system(false, joint_dof_);
		rbd::fromGeneralizedJointState(locomotion_state.base_eff,
									   locomotion_state.joint_eff,
									   (Eigen::VectorXd) generalized_state.segment(idx, joint_dof_),
									   &fake_system);
		idx += joint_dof_;
	}
	if (locomotion_variables_.contact_pos || locomotion_variables_.contact_vel ||
			locomotion_variables_.contact_acc || locomotion_variables_.contact_for) {
		locomotion_state.contacts.resize(num_endeffectors_);
		for (unsigned int i = 0; i < num_endeffectors_; i++) {
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
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_pos,
											 locomotion_state.joint_pos,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.velocity) {
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_vel,
											 locomotion_state.joint_vel,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.acceleration) {
		generalized_state.segment(idx, system_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_acc,
											 locomotion_state.joint_acc,
											 system_);
		idx += system_dof_;
	}
	if (locomotion_variables_.effort) {
		// Defining a fake floating-base system (fixed-base) for converting only joint effort
		rbd::FloatingBaseSystem fake_system(false, joint_dof_);
		generalized_state.segment(idx, joint_dof_) =
				rbd::toGeneralizedJointState(locomotion_state.base_eff,
											 locomotion_state.joint_eff,
											 &fake_system);
		idx += joint_dof_;
	}
	if (locomotion_variables_.contact_pos || locomotion_variables_.contact_vel ||
			locomotion_variables_.contact_acc || locomotion_variables_.contact_for) {
		if (locomotion_state.contacts.size() != num_endeffectors_) {
			printf(RED "FATAL: the number of contact and end-effectors are not consistent\n" COLOR_RESET);
			exit(EXIT_FAILURE);
		}

		for (unsigned int i = 0; i < num_endeffectors_; i++) {
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


void DynamicalSystem::initialConditions()
{
	// Computing the state dimension give the locomotion variables
	state_dimension_ = (locomotion_variables_.position + locomotion_variables_.velocity
			+ locomotion_variables_.acceleration) * system_->getSystemDOF()
			+ locomotion_variables_.effort * system_->getJointDOF();

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();

	// No-bound limit by default
	lower_state_bound_.base_pos = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_pos = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_vel = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_vel = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_acc = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_acc = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	lower_state_bound_.base_eff = -NO_BOUND * rbd::Vector6d::Ones();
	lower_state_bound_.joint_eff = -NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_pos = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_pos = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_vel = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_vel = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_acc = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_acc = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);
	upper_state_bound_.base_eff = NO_BOUND * rbd::Vector6d::Ones();
	upper_state_bound_.joint_eff = NO_BOUND * Eigen::VectorXd::Ones(joint_dof_);

	// Starting state
	starting_state_.joint_pos = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_vel = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_acc = Eigen::VectorXd::Zero(joint_dof_);
	starting_state_.joint_eff = Eigen::VectorXd::Zero(joint_dof_);
}

} //@namespace model
} //@namespace dwl
