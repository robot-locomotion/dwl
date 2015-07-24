#include <model/ConstrainedDynamicalSystem.h>


namespace dwl
{

namespace model
{

ConstrainedDynamicalSystem::ConstrainedDynamicalSystem()
{
	// Setting the name of the constraint
	name_ = "constrained";

	// Setting the locomotion variables for this dynamical constraint
	locomotion_variables_.position = true;
	locomotion_variables_.velocity = true;
	locomotion_variables_.acceleration = true;
	locomotion_variables_.effort = true;
}


ConstrainedDynamicalSystem::~ConstrainedDynamicalSystem()
{

}


void ConstrainedDynamicalSystem::setActiveEndEffectors(const rbd::BodySelector& active_set)
{
	active_endeffectors_ = active_set;
}


void ConstrainedDynamicalSystem::compute(Eigen::VectorXd& constraint,
										 const LocomotionState& state)
{
	constraint.resize(joint_dof_);

	// Transcription of the constrained inverse dynamic equation using Euler-backward integration.
	// This integration method adds numerical stability
	Eigen::VectorXd estimated_joint_forces;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															state.base_acc - last_state_.base_acc,
															state.joint_acc - last_state_.joint_acc,
															active_endeffectors_);

	constraint = estimated_joint_forces - state.joint_eff;
}


void ConstrainedDynamicalSystem::getBounds(Eigen::VectorXd& lower_bound,
										   Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(joint_dof_);
	upper_bound = Eigen::VectorXd::Zero(joint_dof_);
}

} //@namespace model
} //@namespace dwl
