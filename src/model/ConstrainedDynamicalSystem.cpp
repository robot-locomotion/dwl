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
	locomotion_variables_.effort = true;
}


ConstrainedDynamicalSystem::~ConstrainedDynamicalSystem()
{

}


void ConstrainedDynamicalSystem::setActiveEndEffectors(const rbd::BodySelector& active_set)
{
	active_endeffectors_ = active_set;
}


void ConstrainedDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
															const LocomotionState& state)
{
	// Resizing the constraint vector
	constraint.resize(joint_dof_ + 3); //TODO make for n end-effectors

	// Computing the step time
	double step_time = state.time - last_state_.time;

	Eigen::VectorXd estimated_joint_forces;
	Eigen::VectorXd base_acc = (state.base_vel - last_state_.base_vel) / step_time;
	Eigen::VectorXd joint_acc = (state.joint_vel - last_state_.joint_vel) / step_time;
	dynamics_.computeConstrainedFloatingBaseInverseDynamics(estimated_joint_forces,
															state.base_pos, state.joint_pos,
															state.base_vel, state.joint_vel,
															base_acc, joint_acc,
															active_endeffectors_);
	constraint.segment(0, joint_dof_) = estimated_joint_forces - state.joint_eff;


	// This constrained inverse dynamic algorithm could generated joint forces in cases where the
	// ground (or environment) is pulling or pushing the end-effector, which an unreal situation.
	// So, it's required to impose a velocity kinematic constraint in the active end-effectors
	rbd::BodyVector endeffectors_vel;
	kinematics_.computeVelocity(endeffectors_vel,
								state.base_pos, state.joint_pos,
								state.base_vel, state.joint_vel,
								active_endeffectors_, rbd::Linear);
	for (rbd::BodyVector::iterator endeffector_it = endeffectors_vel.begin();
			endeffector_it != endeffectors_vel.end(); endeffector_it++) { //TODO make for n end-effectors
		constraint.segment<3>(joint_dof_) = endeffector_it->second;
	}
}


void ConstrainedDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
													Eigen::VectorXd& upper_bound)
{
//	lower_bound = Eigen::VectorXd::Zero(system_dof_ + joint_dof_ + 3);
//	upper_bound = Eigen::VectorXd::Zero(system_dof_ + joint_dof_ + 3);
	lower_bound = Eigen::VectorXd::Zero(joint_dof_ + 3);
	upper_bound = Eigen::VectorXd::Zero(joint_dof_ + 3);
}

} //@namespace model
} //@namespace dwl
