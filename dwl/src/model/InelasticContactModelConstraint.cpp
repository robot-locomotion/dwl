#include <dwl/model/InelasticContactModelConstraint.h>


namespace dwl
{

namespace model
{

InelasticContactModelConstraint::InelasticContactModelConstraint()
{
	// Setting the name of the constraint
	name_ = "inelastic contact model";
}


InelasticContactModelConstraint::~InelasticContactModelConstraint()
{

}


void InelasticContactModelConstraint::init(std::string urdf_model,
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


void InelasticContactModelConstraint::computeFirstComplement(Eigen::VectorXd& constraint,
															 const WholeBodyState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	urdf_model::LinkID contact_links = system_.getEndEffectors();
	for (urdf_model::LinkID::const_iterator contact_it = contact_links.begin();
			contact_it != contact_links.end(); contact_it++) {
		std::string name = contact_it->first;
		unsigned int id = contact_it->second;
		constraint(id) = state.contact_eff.at(name)(rbd::LZ);
	}
}


void InelasticContactModelConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
															  const WholeBodyState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Computing the contact position
	rbd::BodyVector contact_pos;
	kinematics_.computeForwardKinematics(contact_pos,
										 state.base_pos, state.joint_pos,
										 end_effector_names_, rbd::Linear);

	// Adding the contact distance per every end-effector as a the second complementary
	// TODO there is missing the concept of surface
	double surface1_height = -0.582715;
	double surface2_height = -0.402715;
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++) {
		Eigen::VectorXd position = contact_pos.find(end_effector_names_[k])->second;

		if (position(rbd::X) < 0.125)
			constraint(k) = position(rbd::Z) - surface1_height;
		else
			constraint(k) = position(rbd::Z) - surface2_height;
	}
}

} //@namespace model
} //@namespace dwl
