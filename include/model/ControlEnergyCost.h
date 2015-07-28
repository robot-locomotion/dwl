#ifndef DWL__MODEL__CONTROL_ENERGY_COST__H
#define DWL__MODEL__CONTROL_ENERGY_COST__H

#include <model/Cost.h>


namespace dwl
{

namespace model
{

class ControlEnergyCost : public Cost
{
	public:
		ControlEnergyCost();
		~ControlEnergyCost();

		void compute(double& cost,
					 const LocomotionState& state);
};

} //@namespace model
} //@namespace dwl

#endif
