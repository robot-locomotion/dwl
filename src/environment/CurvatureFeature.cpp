#include <environment/CurvatureFeature.h>
#include <Eigen/Dense>


namespace dwl
{

namespace environment
{


CurvatureFeature::CurvatureFeature() : positive_threshold_(6.0), negative_threshold_(-6.0)
{
	name_ = "curvature";
	gain_ = 1;
}

CurvatureFeature::~CurvatureFeature()
{

}

void CurvatureFeature::computeReward(double& reward_value, Terrain terrain_info)
{
	double curvature = terrain_info.curvature;

	 // The worse condition
	if (curvature * 10000 > 9) {
		reward_value = -2;

		return;
	}

	double p;
	if (curvature > positive_threshold_)
		p = 1.0;
	else if (curvature < negative_threshold_)
		p = 0.13;
	else
		p = 0.13 + ((curvature - negative_threshold_) / (positive_threshold_ - negative_threshold_)) * (1 - 0.13);

	reward_value = gain_ * log(p);
}


} //@namespace environment

} //@namespace dwl
