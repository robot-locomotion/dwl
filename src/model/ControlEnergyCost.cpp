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
	// Checking sizes
	if (state.joint_eff.size() != locomotion_weights_.joint_eff.size()) {
		printf(RED "FATAL: the joint efforts dimensions are not consistent\n" COLOR_RESET);
		exit(EXIT_FAILURE);
	}

	// Computing the control cost
	cost = state.joint_eff.transpose() * locomotion_weights_.joint_eff.asDiagonal() *
			state.joint_eff;
}

} //@namespace model
} //@namespace dwl
