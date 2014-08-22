#include <environment/CurvatureFeature.h>
#include <Eigen/Dense>


namespace dwl
{

namespace environment
{


CurvatureFeature::CurvatureFeature() : positive_threshold_(6.0), negative_threshold_(-6.0)
{
	name_ = "Curvature";
}

CurvatureFeature::~CurvatureFeature()
{

}

void CurvatureFeature::computeReward(double& reward_value, Terrain terrain_info)
{
	double curvature = terrain_info.curvature;

	 // The worse condition
	if (curvature * 10000 > 9) {
		reward_value = max_reward_;

		return;
	}

	double p;
	if (curvature > positive_threshold_)
		reward_value = 0;
	else if (curvature < negative_threshold_)
		reward_value = max_reward_;
	else
		reward_value = max_reward_ + log((curvature - negative_threshold_) / (positive_threshold_ - negative_threshold_));
}


} //@namespace environment

} //@namespace dwl
