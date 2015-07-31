#include <model/Cost.h>


namespace dwl
{

namespace model
{

Cost::Cost()
{

}


Cost::~Cost()
{

}


void Cost::setWeights(LocomotionState& weights)
{
	// Checking the cost variables
	cost_variables_.base_pos = !weights.base_pos.isZero();
	cost_variables_.base_vel = !weights.base_vel.isZero();
	cost_variables_.base_acc = !weights.base_acc.isZero();
	cost_variables_.base_eff = !weights.base_eff.isZero();
	cost_variables_.joint_pos = !weights.joint_pos.isZero();
	cost_variables_.joint_vel = !weights.joint_vel.isZero();
	cost_variables_.joint_acc = !weights.joint_acc.isZero();
	cost_variables_.joint_eff = !weights.joint_eff.isZero();

	if (weights.contacts.size() > 0) {
		cost_variables_.contact_pos = weights.contacts[0].position.isZero();
		cost_variables_.contact_vel = weights.contacts[0].velocity.isZero();
		cost_variables_.contact_acc = weights.contacts[0].acceleration.isZero();
		cost_variables_.contact_for = weights.contacts[0].force.isZero();
	}

	locomotion_weights_ = weights;
}


void Cost::setDesiredState(LocomotionState& desired_state)
{
	desired_state_ = desired_state;
}


std::string Cost::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
