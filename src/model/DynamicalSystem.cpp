#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : system_(NULL), state_dimension_(0), num_endeffectors_(1),
		system_dof_(0), joint_dof_(0), locomotion_variables_(false) //TODO clean it
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::modelFromURDFFile(std::string model_file,
										struct rbd::FloatingBaseSystem* system,
										bool info)
{
	dynamics_.modelFromURDFFile(model_file, system, info);

	system_ = system;

	// Computing the state dimension give the locomotion variables
	state_dimension_ = (locomotion_variables_.position + locomotion_variables_.velocity
			+ locomotion_variables_.acceleration) * system_->getSystemDOF()
			+ locomotion_variables_.effort * system_->getJointDOF();

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();
}


void DynamicalSystem::modelFromURDFModel(std::string urdf_model,
										 struct rbd::FloatingBaseSystem* system,
										 bool info)
{
	dynamics_.modelFromURDFModel(urdf_model, system, info);

	system_ = system;

	// Computing the state dimension give the locomotion variables
	state_dimension_ = (locomotion_variables_.position + locomotion_variables_.velocity
			+ locomotion_variables_.acceleration) * system_->getSystemDOF()
			+ locomotion_variables_.effort * system_->getJointDOF();

	// Getting the dof of the system
	system_dof_ = system_->getSystemDOF();
	joint_dof_ = system_->getJointDOF();
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
//TODO add other locomotion variables
{
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
}


void DynamicalSystem::fromLocomotionState(Eigen::VectorXd& generalized_state,
										  const LocomotionState& locomotion_state)
//TODO add other locomotion variables
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
}

} //@namespace model
} //@namespace dwl
