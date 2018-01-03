#ifndef DWL__OCP__INTEGRAL_STATE_TRACKING_ENERGY_COST__H
#define DWL__OCP__INTEGRAL_STATE_TRACKING_ENERGY_COST__H

#include <dwl/ocp/Cost.h>


namespace dwl
{

namespace ocp
{

/**
 * @class Implements a quadratic cost function for computing a integral state (position, velocity
 * and acceleration) tracking energy cost given a locomotion state
 */
class IntegralStateTrackingEnergyCost : public Cost
{
	public:
		/** @brief Constructor function */
		IntegralStateTrackingEnergyCost();

		/** @brief Destructor function */
		~IntegralStateTrackingEnergyCost();

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
