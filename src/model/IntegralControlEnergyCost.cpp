#include <model/IntegralControlEnergyCost.h>


namespace dwl
{

namespace model
{

IntegralControlEnergyCost::IntegralControlEnergyCost()
{
	name_ = "integral control energy";
}


IntegralControlEnergyCost::~IntegralControlEnergyCost()
{

}


void IntegralControlEnergyCost::compute(double& cost,
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
	cost *= state.duration;
}

} //@namespace model
} //@namespace dwl
