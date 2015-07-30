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
	cost = 0.001 * state.joint_eff.transpose() * state.joint_eff;
}

} //@namespace model
} //@namespace dwl
