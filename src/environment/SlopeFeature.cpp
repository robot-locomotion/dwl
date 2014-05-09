#include <environment/SlopeFeature.h>
#include <Eigen/Dense>


namespace dwl
{

namespace environment
{


SlopeFeature::SlopeFeature() : flat_threshold_(0.0 * (M_PI / 180.0)), bad_threshold_(70.0 * (M_PI / 180.0))
{
	name_ = "slope";
}

SlopeFeature::~SlopeFeature()
{

}

void SlopeFeature::computeReward(double& reward_value, Terrain terrain_info)
{
//	printf("Computing the slope feature\n");
	double slope = fabs(acos((double) terrain_info.surface_normal(2)));

	if (slope < flat_threshold_)
		reward_value = 0.0;
	else {
		double p = (bad_threshold_ - slope) / (bad_threshold_ - flat_threshold_);
		if (p < 0.01)
			p = 0.01;
		reward_value = log(p);
	}

	reward_value *= 5; /* heuristic value */
}


} //@namespace environment

} //@namespace dwl
