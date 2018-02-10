#include <dwl/ocp/InelasticContactModelConstraint.h>


namespace dwl
{

namespace ocp
{

InelasticContactModelConstraint::InelasticContactModelConstraint()
{
	// Setting the name of the constraint
	name_ = "inelastic contact model";
}


InelasticContactModelConstraint::~InelasticContactModelConstraint()
{

}


void InelasticContactModelConstraint::init(bool info)
{
	// Getting the end-effector names
	end_effector_names_.clear();
	urdf_model::LinkID end_effector = fbs_->getEndEffectors();
	for (urdf_model::LinkID::iterator endeffector_it = end_effector.begin();
			endeffector_it != end_effector.end(); endeffector_it++) {
		// Getting and setting the end-effector names
		std::string name = endeffector_it->first;
		end_effector_names_.push_back(name);
	}

	// Setting the complementary dimension
	complementary_dimension_ = fbs_->getNumberOfEndEffectors();
}


void InelasticContactModelConstraint::computeFirstComplement(Eigen::VectorXd& constraint,
															 const WholeBodyState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(fbs_->getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	for (rbd::BodyVector6d::const_iterator contact_it = state.contact_eff.begin();
			contact_it != state.contact_eff.end(); contact_it++) {
		std::string name = contact_it->first;
		unsigned int id = fbs_->getEndEffectors().find(name)->second;

		constraint(id) = contact_it->second(rbd::LZ);
	}
}


void InelasticContactModelConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
															  const WholeBodyState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(fbs_->getNumberOfEndEffectors());

	// Computing the contact position
	rbd::BodyVectorXd contact_pos;
	wkin_->computeForwardKinematics(contact_pos,
									state.base_pos, state.joint_pos,
									end_effector_names_, rbd::Linear);

	// Adding the contact distance per every end-effector as a the second complementary
	// TODO there is missing the concept of surface
	double surface1_height = -0.582715;
	double surface2_height = -0.402715;
	for (rbd::BodyVectorXd::const_iterator contact_it = state.contact_pos.begin();
			contact_it != state.contact_pos.end(); contact_it++) {
		std::string name = contact_it->first;
		Eigen::VectorXd position = contact_it->second;
		unsigned int id = fbs_->getEndEffectors().find(name)->second;

		if (position(rbd::X) < 0.125)
			constraint(id) = position(rbd::Z) - surface1_height;
		else
			constraint(id) = position(rbd::Z) - surface2_height;
	}
}

} //@namespace ocp
} //@namespace dwl
