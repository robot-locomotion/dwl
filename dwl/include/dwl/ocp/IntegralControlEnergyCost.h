#ifndef DWL__OCP__INTEGRAL_CONTROL_ENERGY_COST__H
#define DWL__OCP__INTEGRAL_CONTROL_ENERGY_COST__H

#include <dwl/ocp/Cost.h>


namespace dwl
{

namespace ocp
{

/**
 * @class Implements a quadratic cost function for computing a integral control energy cost given
 * a locomotion state
 */
class IntegralControlEnergyCost : public Cost
{
	public:
		/** @brief Constructor function */
		IntegralControlEnergyCost();

		/** @brief Destructor function */
		~IntegralControlEnergyCost();

		/**
		 * @brief Computes the control energy cost, i.e. joint efforts energy, given a locomotion
		 * state. The control energy is defined as quadratic cost function
		 * @param double& Cost value
		 * @param const WholeBodyState& Whole-body state
		 */
		void compute(double& cost,
					 const WholeBodyState& state);
};

} //@namespace ocp
} //@namespace dwl

#endif
