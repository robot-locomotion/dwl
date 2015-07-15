#include <model/DynamicalSystem.h>


namespace dwl
{

namespace model
{

DynamicalSystem::DynamicalSystem() : state_dimension_(0), num_joints_(4), num_endeffectors_(1),
		locomotion_variables_(false) //TODO clean it
{
	locomotion_variables_.joint_pos = true; //TODO remove it
	state_dimension_ = 4;
}


DynamicalSystem::~DynamicalSystem()
{

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
{
	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		locomotion_state.time = generalized_state(idx);
		++idx;
	} else if (locomotion_variables_.base_pos) {
		locomotion_state.base_pos = generalized_state.segment<6>(idx);
		idx += 6;
	} else if (locomotion_variables_.base_vel) {
		locomotion_state.base_vel = generalized_state.segment<6>(idx);
		idx += 6;
	} else if (locomotion_variables_.base_acc) {
		locomotion_state.base_acc = generalized_state.segment<6>(idx);
		idx += 6;
	} else if (locomotion_variables_.joint_pos) {
		locomotion_state.joint_pos.resize(num_joints_);
		locomotion_state.joint_pos = generalized_state.segment(idx, num_joints_);
		idx += num_joints_;
	} else if (locomotion_variables_.joint_vel) {
		locomotion_state.joint_vel = generalized_state.segment(idx, num_joints_);
		idx += num_joints_;
	} else if (locomotion_variables_.joint_acc) {
		locomotion_state.joint_acc = generalized_state.segment(idx, num_joints_);
		idx += num_joints_;
	}// else if (locomotion_variables_.contact_pos) { //TODO implement for contact events
//		unsigned int num_endeffectors = dynamic_system_->getNumberOfEndEffectors();
//
//
//	}
}


void DynamicalSystem::fromLocomotionState(Eigen::VectorXd& generalized_state,
										  const LocomotionState& locomotion_state)
{
	// Resizing the generalized state vector
	generalized_state.resize(state_dimension_);

	unsigned int idx = 0;
	if (locomotion_variables_.time) {
		generalized_state(idx) = locomotion_state.time;
		++idx;
	} else if (locomotion_variables_.base_pos) {
		generalized_state.segment<6>(idx) = locomotion_state.base_pos;
		idx += 6;
	} else if (locomotion_variables_.base_vel) {
		generalized_state.segment<6>(idx) = locomotion_state.base_vel;
		idx += 6;
	} else if (locomotion_variables_.base_acc) {
		generalized_state.segment<6>(idx) = locomotion_state.base_acc;
		idx += 6;
	} else if (locomotion_variables_.joint_pos) {
		generalized_state.segment(idx, num_joints_) = locomotion_state.joint_pos;
		idx += num_joints_;
	} else if (locomotion_variables_.joint_vel) {
		generalized_state.segment(idx, num_joints_) = locomotion_state.joint_vel;
		idx += num_joints_;
	} else if (locomotion_variables_.joint_acc) {
		generalized_state.segment(idx, num_joints_) = locomotion_state.joint_acc;
		idx += num_joints_;
	}//TODO implement for contact events
}

} //@namespace model
} //@namespace dwl
