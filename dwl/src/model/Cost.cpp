#include <dwl/model/Cost.h>


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


void Cost::setWeights(const WholeBodyState& weights)
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

	if (weights.contact_pos.size() > 0) {
		std::string name = "foot"; // TODO read through system
		cost_variables_.contact_pos = !weights.contact_pos.at(name).isZero();
		cost_variables_.contact_vel = !weights.contact_vel.at(name).isZero();
		cost_variables_.contact_acc = !weights.contact_acc.at(name).isZero();
		cost_variables_.contact_for = !weights.contact_eff.at(name).isZero();
	}

	locomotion_weights_ = weights;
}


void Cost::setDesiredState(const WholeBodyState& desired_state)
{
	desired_state_ = desired_state;
}


std::string Cost::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
