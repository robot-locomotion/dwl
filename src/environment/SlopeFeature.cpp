#include <environment/SlopeFeature.h>
#include <Eigen/Dense>


namespace dwl
{

namespace environment
{


SlopeFeature::SlopeFeature() : flat_threshold_(0.0 * (M_PI / 180.0)), steep_threshold_(70.0 * (M_PI / 180.0))
{
	name_ = "slope";
	gain_ = 5;
}

SlopeFeature::~SlopeFeature()
{

}


void SlopeFeature::computeReward(double& reward_value, Terrain terrain_info)
{
	double slope = fabs(acos((double) terrain_info.surface_normal(2)));

	if (slope < flat_threshold_)
		reward_value = 0.0;
	else {
		double p = (steep_threshold_ - slope) / (steep_threshold_ - flat_threshold_);
		if (p < 0.01)
			p = 0.01;
		reward_value = log(p);
	}

	reward_value *= gain_; /* heuristic value */
}


} //@namespace environment

} //@namespace dwl
