#ifndef DWL_SlopeFeature_H
#define DWL_SlopeFeature_H

#include <environment/Feature.h>
#include <utils/macros.h>


namespace dwl
{

namespace environment
{


class SlopeFeature : public Feature
{
	public:
		SlopeFeature();
		~SlopeFeature();

		void computeReward(double& reward_value, Terrain terrain_info);
};


} //@namespace environment

} //@namespace dwl

#endif
