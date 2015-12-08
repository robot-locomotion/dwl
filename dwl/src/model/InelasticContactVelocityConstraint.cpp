#include <dwl/model/InelasticContactVelocityConstraint.h>


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


void InelasticContactVelocityConstraint::init(bool info)
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
																const WholeBodyState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	for (rbd::BodyWrench::const_iterator contact_it = state.contact_eff.begin();
			contact_it != state.contact_eff.end(); contact_it++) {
		std::string name = contact_it->first;
		unsigned int id = system_.getEndEffectors().find(name)->second;

		constraint(id) = contact_it->second(rbd::LZ);
	}
}


void InelasticContactVelocityConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
																 const WholeBodyState& state)
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
	for (rbd::BodyVector::const_iterator contact_it = state.contact_pos.begin();
			contact_it != state.contact_pos.end(); contact_it++) {
		std::string name = contact_it->first;
		Eigen::VectorXd position = contact_it->second;
		unsigned int id = system_.getEndEffectors().find(name)->second;

		constraint(id) = position(rbd::X) - last_contact_pos.find(name)->second(rbd::X);
	}
}

} //@namespace model
} //@namespace dwl
