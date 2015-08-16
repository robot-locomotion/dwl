#include <model/InelasticContactModelConstraint.h>


namespace dwl
{

namespace model
{

InelasticContactModelConstraint::InelasticContactModelConstraint()
{

}


InelasticContactModelConstraint::~InelasticContactModelConstraint()
{

}


void InelasticContactModelConstraint::computeFirstComplement(Eigen::VectorXd& constraint,
															 const LocomotionState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = state.contacts[k].force(rbd::LZ);
}


void InelasticContactModelConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
															  const LocomotionState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Getting the end-effector names
	urdf_model::LinkSelector end_effector_names = system_.getEndEffectors();

	// Computing the contact position
	rbd::BodyVector contact_pos;
	kinematics_.computeForwardKinematics(contact_pos,
										 state.base_pos, state.joint_pos,
										 end_effector_names, rbd::Linear);

	// Adding the contact distance per every end-effector as a the second complementary
	// TODO there is missing the concept of surface
	double surface_height = -0.27;
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = contact_pos.find(end_effector_names[k])->second(rbd::LZ) - surface_height;
}

} //@namespace model
} //@namespace dwl
