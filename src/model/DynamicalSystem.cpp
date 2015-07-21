#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : system_(NULL), state_dimension_(0), num_joints_(0),
		num_endeffectors_(1), locomotion_variables_(false) //TODO clean it
{

}


DynamicalSystem::~DynamicalSystem()
{

}


void DynamicalSystem::setFloatingBaseSystem(rbd::FloatingBaseSystem* system)
{
	system_ = system;
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


unsigned int DynamicalSystem::getNumberOfJoints()
{
	return num_joints_;
}


unsigned int DynamicalSystem::getNumberOfEndEffectors()
{
	return num_endeffectors_;
}


void DynamicalSystem::toLocomotionState(LocomotionState& locomotion_state,
										const Eigen::VectorXd& generalized_state)
//TODO add other locomotion variables
{
	// Getting the number of degree of freedom
	unsigned int num_dof = system_->getFloatingBaseDOF() + system_->getJointDOF();

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		locomotion_state.time = generalized_state(idx);
		++idx;
	}
	if (locomotion_variables_.position) {
		rbd::fromGeneralizedJointState(locomotion_state.base_pos,
									   locomotion_state.joint_pos,
									   (Eigen::VectorXd) generalized_state.segment(idx, num_dof),
									   system_);
		idx += num_dof;
	}
	if (locomotion_variables_.velocity) {
		rbd::fromGeneralizedJointState(locomotion_state.base_vel,
									   locomotion_state.joint_vel,
									   (Eigen::VectorXd) generalized_state.segment(idx, num_dof),
									   system_);
		idx += num_dof;
	}
	if (locomotion_variables_.acceleration) {
		rbd::fromGeneralizedJointState(locomotion_state.base_acc,
									   locomotion_state.joint_acc,
									   (Eigen::VectorXd) generalized_state.segment(idx, num_dof),
									   system_);
		idx += num_dof;
	}
}


void DynamicalSystem::fromLocomotionState(Eigen::VectorXd& generalized_state,
										  const LocomotionState& locomotion_state)
//TODO add other locomotion variables
{
	// Getting the number of degree of freedom
	unsigned int num_dof = system_->getFloatingBaseDOF() + system_->getJointDOF();

	// Converting the generalized state vector to locomotion state
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		generalized_state(idx) = locomotion_state.time;
	}
	if (locomotion_variables_.position) {
		generalized_state.segment(idx, num_dof) =
				rbd::toGeneralizedJointState(locomotion_state.base_pos,
											 locomotion_state.joint_pos,
											 system_);
		idx += num_dof;
	}
	if (locomotion_variables_.velocity) {
		generalized_state.segment(idx, num_dof) =
				rbd::toGeneralizedJointState(locomotion_state.base_vel,
											 locomotion_state.joint_vel,
											 system_);
		idx += num_dof;
	}
	if (locomotion_variables_.acceleration) {
		generalized_state.segment(idx, num_dof) =
				rbd::toGeneralizedJointState(locomotion_state.base_acc,
											 locomotion_state.joint_acc,
											 system_);
		idx += num_dof;
	}
}

} //@namespace model
} //@namespace dwl
