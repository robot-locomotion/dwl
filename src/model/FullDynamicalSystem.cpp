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
//	locomotion_variables_.acceleration = true;
	locomotion_variables_.effort = true;
	locomotion_variables_.contact_for = true;
}


FullDynamicalSystem::~FullDynamicalSystem()
{

}


void FullDynamicalSystem::init(std::string urdf_model)
{
	// Getting the end-effector names
	end_effector_names_.clear();
	urdf_model::LinkID end_effector = system_.getEndEffectors();
	for (urdf_model::LinkID::iterator endeffector_it = end_effector.begin();
			endeffector_it != end_effector.end(); endeffector_it++) {
		// Getting and setting the end-effector names
		std::string name = endeffector_it->first;
		end_effector_names_.push_back(name);
	}

	// Setting the constraint dimension
	constraint_dimension_ = system_.getJointDoF();
}


void FullDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
													 const LocomotionState& state)
{
	// Resizing the constraint vector
	constraint.resize(system_.getJointDoF());

	// Computing the step time
	double step_time = state.time - last_state_.time;

	// Computing the joint acceleration from velocities
	Eigen::VectorXd joint_acc = (state.joint_vel - last_state_.joint_vel) / step_time;

	// Setting the contact forces
	rbd::BodyWrench contact_forces;
	for (unsigned int k = 0; k < system_.getFloatingBaseDoF(); k++)
		contact_forces[end_effector_names_[k]] << 0, 0, 0, state.contacts[k].force;

	// Computing the floating-base inverse dynamics
	rbd::Vector6d base_acc;
	Eigen::VectorXd estimated_joint_forces;
	dynamics_.computeFloatingBaseInverseDynamics(base_acc, estimated_joint_forces,
												 state.base_pos, state.joint_pos,
												 state.base_vel, state.joint_vel,
												 joint_acc, contact_forces);
	constraint = estimated_joint_forces - state.joint_eff;
}


void FullDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
											 Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(system_.getJointDoF());
	upper_bound = Eigen::VectorXd::Zero(system_.getJointDoF());
}

} //@namespace model
} //@namespace dwl
