#ifndef DWL__OCP__TERMINAL_STATE_TRACKING_ENERGY_COST__H
#define DWL__OCP__TERMINAL_STATE_TRACKING_ENERGY_COST__H

#include <dwl/ocp/Cost.h>


namespace dwl
{

namespace ocp
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
		 * @param const WholeBodyState& Whole-body state
		 */
		void compute(double& cost,
					 const WholeBodyState& state);
};

} //@namespace ocp
} //@namespace dwl

#endif
