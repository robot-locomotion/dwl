#ifndef DWL__MODEL__TERMINAL_STATE_TRACKING_ENERGY_COST__H
#define DWL__MODEL__TERMINAL_STATE_TRACKING_ENERGY_COST__H

#include <model/Cost.h>


namespace dwl
{

namespace model
{

/**
 * @class Implements a quadratic cost function for computing a terminal state (position, velocity
 * and acceleration) tracking energy cost given a locomotion state
 */
class TerminalStateTrackingEnergyCost : public Cost
{
	public:
		/** @brief Constructor function */
		TerminalStateTrackingEnergyCost();

		/** @brief Destructor function */
		~TerminalStateTrackingEnergyCost();

		/**
		 * @brief Computes the state-tracking energy cost given a locomotion state. The
		 * state-tracking energy is defined as quadratic cost function
		 * @param double& Cost value
		 * @param const LocomotionState& Locomotion state
		 */
		void compute(double& cost,
					 const LocomotionState& state);
};

} //@namespace model
} //@namespace dwl

#endif
