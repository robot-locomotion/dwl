#ifndef DWL__MODEL__STATE_TRACKING_ENERGY_COST__H
#define DWL__MODEL__STATE_TRACKING_ENERGY_COST__H

#include <model/Cost.h>


namespace dwl
{

namespace model
{

class StateTrackingEnergyCost : public Cost
{
	public:
		StateTrackingEnergyCost();
		~StateTrackingEnergyCost();

		void compute(double& cost,
					 const LocomotionState& state);
};

} //@namespace model
} //@namespace dwl

#endif
