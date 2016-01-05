#include <dwl/ocp/CentroidalDynamicalSystem.h>


namespace dwl
{

namespace ocp
{

CentroidalDynamicalSystem::CentroidalDynamicalSystem() : total_mass_(0.)
{
	// Setting the name of the constraint
	name_ = "centroidal";

	// Setting the locomotion variables for this dynamical constraint
	system_variables_.position = true;
	system_variables_.velocity = true;
	system_variables_.contact_pos = true;
	system_variables_.contact_for = true;
}


CentroidalDynamicalSystem::~CentroidalDynamicalSystem()
{

}


void CentroidalDynamicalSystem::initDynamicalSystem()
{
	// Getting the end-effector names
	end_effector_names_ = system_.getEndEffectorNames();

	// Getting the total mass
	total_mass_ = system_.getTotalMass();
}


void CentroidalDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
														   const WholeBodyState& state)
{
	// Resizing the constraint vector
	constraint.resize(4 * system_.getNumberOfEndEffectors());//(1+3)

	// Computing the step time
	double step_time = state.time - state_buffer_[0].time;

	// Computing the joint acceleration from velocities
	Eigen::VectorXd base_acc = (state.base_vel - state_buffer_[0].base_vel) / step_time;


	// Computing the centroidal dynamics
	// TODO right now, there is only the linear momentum (HyL don't have angular momentum). I need
	// to implement it, and move it to WholeBodyDynamics class
	Eigen::Vector3d estimated_com_acc = Eigen::Vector3d::Zero();
	for (rbd::BodyWrench::const_iterator contact_it = state.contact_eff.begin();
			contact_it != state.contact_eff.end(); contact_it++) {
		estimated_com_acc += contact_it->second.segment<3>(rbd::LX) / total_mass_;
		estimated_com_acc += system_.getRBDModel().gravity;
	}
	constraint(0) = estimated_com_acc(rbd::Z) - base_acc(rbd::LZ);




	// Computing the contact position
	rbd::BodyVector contact_pos;
	kinematics_.computeForwardKinematics(contact_pos,
										 state.base_pos, state.joint_pos,
										 end_effector_names_, rbd::Linear);
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint.segment<3>(3 * k + 1) = contact_pos.at(end_effector_names_[k]) -
			state.contact_pos.at(end_effector_names_[k]);
}


void CentroidalDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
												   Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(4 * system_.getNumberOfEndEffectors());
	upper_bound = Eigen::VectorXd::Zero(4 * system_.getNumberOfEndEffectors());
}

} //@namespace ocp
} //@namespace dwl
