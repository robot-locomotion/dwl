#include <model/CentroidalDynamicalSystem.h>


namespace dwl
{

namespace model
{

CentroidalDynamicalSystem::CentroidalDynamicalSystem() : total_mass_(0.)
{
	// Setting the name of the constraint
	name_ = "centroidal";

	// Setting the locomotion variables for this dynamical constraint
	locomotion_variables_.position = true;
	locomotion_variables_.velocity = true;
//	locomotion_variables_.effort = true;

	locomotion_variables_.contact_pos = true;
	locomotion_variables_.contact_for = true;
}


CentroidalDynamicalSystem::~CentroidalDynamicalSystem()
{

}


void CentroidalDynamicalSystem::initDynamicalSystem()
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

	// Getting the total mass
	unsigned int num_bodies = system_.getRBDModel().mBodies.size();
	for (unsigned int i = 0; i < num_bodies; i++)
		total_mass_ += system_.getRBDModel().mBodies[i].mMass;
}


void CentroidalDynamicalSystem::computeDynamicalConstraint(Eigen::VectorXd& constraint,
														   const LocomotionState& state)
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

	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		estimated_com_acc += state.contacts[k].force / total_mass_;
	estimated_com_acc += system_.getRBDModel().gravity;

	constraint(0) = estimated_com_acc(rbd::Z) - base_acc(rbd::LZ);


	// Computing the contact position
	rbd::BodyVector contact_pos;
	kinematics_.computeForwardKinematics(contact_pos,
										 state.base_pos, state.joint_pos,
										 end_effector_names_, rbd::Linear);
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint.segment<3>(3 * k + 1) = contact_pos.find(end_effector_names_[k])->second -
			state.contacts[k].position;
}


void CentroidalDynamicalSystem::getDynamicalBounds(Eigen::VectorXd& lower_bound,
												   Eigen::VectorXd& upper_bound)
{
	lower_bound = Eigen::VectorXd::Zero(4 * system_.getNumberOfEndEffectors());
	upper_bound = Eigen::VectorXd::Zero(4 * system_.getNumberOfEndEffectors());
}

} //@namespace model
} //@namespace dwl
