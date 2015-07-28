#include <model/StateTrackingEnergyCost.h>


namespace dwl
{

namespace model
{

StateTrackingEnergyCost::StateTrackingEnergyCost()
{

}


StateTrackingEnergyCost::~StateTrackingEnergyCost()
{

}


void StateTrackingEnergyCost::compute(double& cost,
									  const LocomotionState& state)
{
	LocomotionState des_state;
	des_state.base_pos << 0, 0, 0, 0, 0, 0.1;
	cost = (state.base_pos - des_state.base_pos).transpose() * (state.base_pos - des_state.base_pos);
}

} //@namespace model
} //@namespace dwl
