#include <model/InelasticContactModelConstraint.h>


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
															 const LocomotionState& state)
{
	// Resizing the complementary constraint dimension
	constraint.resize(system_.getNumberOfEndEffectors());

	// Adding the normal contact forces per every end-effector as a the first complementary
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = state.contacts[k].force(rbd::Z);
}


void InelasticContactModelConstraint::computeSecondComplement(Eigen::VectorXd& constraint,
															  const LocomotionState& state)
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
	double surface_height = -0.582715;
	for (unsigned int k = 0; k < system_.getNumberOfEndEffectors(); k++)
		constraint(k) = contact_pos.find(end_effector_names_[k])->second(rbd::Z) - surface_height;
}

} //@namespace model
} //@namespace dwl
