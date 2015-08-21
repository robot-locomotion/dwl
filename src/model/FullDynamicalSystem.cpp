#include <model/FullDynamicalSystem.h>


namespace dwl
{

namespace model
{

FullDynamicalSystem::FullDynamicalSystem()
{
	// Setting the name of the constraint
	name_ = "full";

	// Setting the locomotion variables for this dynamical constraint
	locomotion_variables_.position = true;
	locomotion_variables_.velocity = true;
	locomotion_variables_.effort = true;
	locomotion_variables_.contact_for = true;
}


FullDynamicalSystem::~FullDynamicalSystem()
{

}


void FullDynamicalSystem::initDynamicalSystem()
{
	// Getting the end-effector names
	end_effector_names_.clear();
	urdf_model::LinkID end_effector = system_.getEndEffectors();
	for (urdf_model::LinkID::const_iterator endeffector_it = end_effector.begin();
			endeffector_it != end_effector.end(); endeffector_it++) {
		// Getting and setting the end-effector names
		std::string name = endeffector_it->first;
		end_effector_names_.push_back(name);
	}
}


void FullDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
													 const LocomotionState& state)
{
	// Resizing the constraint vector
	constraint.resize(system_.getSystemDoF());

	// Computing the step time
	double step_time = state.time - last_state_.time;

	// Computing the joint acceleration from velocities
	Eigen::VectorXd base_acc = (state.base_vel - last_state_.base_vel) / step_time;
	Eigen::VectorXd joint_acc = (state.joint_vel - last_state_.joint_vel) / step_time;

	// Setting the contact forces
	rbd::BodyWrench contact_forces;
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		contact_forces[end_effector_names_[k]] << 0, 0, 0, state.contacts[k].force;

	// Computing the full inverse dynamics. In real-cases, the floating-base effort (state.base_eff)
	// is always equals to zero, which implicates that we are imposing that the base_wrench equals
	// to null vector. TODO Another implementation could be posed as floating-base inverse dynamics
	rbd::Vector6d estimated_base_wrench;
	Eigen::VectorXd estimated_joint_forces;
	dynamics_.computeInverseDynamics(estimated_base_wrench, estimated_joint_forces,
									 state.base_pos, state.joint_pos,
									 state.base_vel, state.joint_vel,
									 base_acc, joint_acc, contact_forces);
	constraint = system_.toGeneralizedJointState(estimated_base_wrench - state.base_eff,
												 estimated_joint_forces - state.joint_eff);
}


void FullDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
											 Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(system_.getSystemDoF());
	upper_bound = Eigen::VectorXd::Zero(system_.getSystemDoF());
}

} //@namespace model
} //@namespace dwl
