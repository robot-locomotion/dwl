#include <dwl/ocp/Cost.h>


namespace dwl
{

namespace ocp
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
		for (rbd::BodyVectorXd::const_iterator pos_it = weights.contact_pos.begin();
				pos_it != weights.contact_pos.end(); pos_it++) {
			if (!pos_it->second.isZero())
				cost_variables_.contact_pos = true;
			else
				cost_variables_.contact_pos = false;
			break;
		}
		for (rbd::BodyVectorXd::const_iterator vel_it = weights.contact_vel.begin();
				vel_it != weights.contact_vel.end(); vel_it++) {
			if (!vel_it->second.isZero())
				cost_variables_.contact_vel = true;
			else
				cost_variables_.contact_vel = false;
			break;
		}
		for (rbd::BodyVectorXd::const_iterator acc_it = weights.contact_acc.begin();
				acc_it != weights.contact_acc.end(); acc_it++) {
			if (!acc_it->second.isZero())
				cost_variables_.contact_acc = true;
			else
				cost_variables_.contact_acc = false;
			break;
		}
		for (rbd::BodyVector6d::const_iterator eff_it = weights.contact_eff.begin();
				eff_it != weights.contact_eff.end(); eff_it++) {
			if (!eff_it->second.isZero())
				cost_variables_.contact_for = true;
			else
				cost_variables_.contact_for = false;
			break;
		}
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

} //@namespace ocp
} //@namespace dwl
