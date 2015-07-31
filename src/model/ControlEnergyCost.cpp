#include <model/ControlEnergyCost.h>


namespace dwl
{

namespace model
{

ControlEnergyCost::ControlEnergyCost()
{
	name_ = "control energy";
}


ControlEnergyCost::~ControlEnergyCost()
{

}


void ControlEnergyCost::compute(double& cost,
								const LocomotionState& state)
{
	cost = state.joint_eff.transpose() * locomotion_weights_.joint_eff.asDiagonal() *
			state.joint_eff;
}

} //@namespace model
} //@namespace dwl
