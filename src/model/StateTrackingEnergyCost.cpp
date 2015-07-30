#include <model/StateTrackingEnergyCost.h>


namespace dwl
{

namespace model
{

StateTrackingEnergyCost::StateTrackingEnergyCost()
{
	name_ = "state-tracking energy";
}


StateTrackingEnergyCost::~StateTrackingEnergyCost()
{

}


void StateTrackingEnergyCost::compute(double& cost,
									  const LocomotionState& state)
{
	LocomotionState des_state;
	des_state.base_pos << 0, 0, 0, 0, 0, -0.1;
	Eigen::VectorXd error_base_pos = des_state.base_pos - state.base_pos;
	Eigen::VectorXd error_base_vel = des_state.base_vel - state.base_vel;
//	Eigen::VectorXd error_base_acc = des_state.base_acc - state.base_acc;

	cost = 10000 * error_base_pos.transpose() * error_base_pos;
	cost += 10 * error_base_vel.transpose() * error_base_vel;
//	cost += 100 * error_base_acc.transpose() * error_base_acc;
}

} //@namespace model
} //@namespace dwl
