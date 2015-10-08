#include <model/InelasticContactVelocityConstraint.h>


namespace dwl
{

namespace model
{

InelasticContactVelocityConstraint::InelasticContactVelocityConstraint()
{
	// Setting the name of the constraint
	name_ = "inelastic contact velocity model";
}


InelasticContactVelocityConstraint::~InelasticContactVelocityConstraint()
{

}


void InelasticContactVelocityConstraint::init(std::string urdf_model,
										   bool info)
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

	// Setting the complementary dimension
	complementary_dimension_ = system_.getNumberOfEndEffectors();
}


void InelasticContactVelocityConstraint::computeFirstComplement(Eigen::VectorXd& constraint,
																const LocomotionState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = state.contacts[k].force(rbd::Z);
}


void InelasticContactVelocityConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
																 const LocomotionState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Computing the changes of the contact position
	rbd::BodyVector current_contact_pos;
	kinematics_.computeForwardKinematics(current_contact_pos,
										 state.base_pos, state.joint_pos,
										 end_effector_names_, rbd::Linear);
	rbd::BodyVector last_contact_pos;
	kinematics_.computeForwardKinematics(last_contact_pos,
										 state_buffer_[0].base_pos, state_buffer_[0].joint_pos,
										 end_effector_names_, rbd::Linear);

	// Adding the contact distance per every end-effector as a the second complementary
	// TODO there is missing the concept of surface
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = current_contact_pos.find(end_effector_names_[k])->second(rbd::X) -
			last_contact_pos.find(end_effector_names_[k])->second(rbd::X);
}

} //@namespace model
} //@namespace dwl
