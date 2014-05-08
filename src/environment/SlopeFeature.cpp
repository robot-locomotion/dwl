#include <environment/SlopeFeature.h>
#include <Eigen/Dense>


namespace dwl
{

namespace environment
{


SlopeFeature::SlopeFeature()
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

	reward_value = slope;


	addCellToRewardMap(reward_value, terrain_info);
	std::cout << "grid size = " << reward_gridmap_.size() << std::endl;
}


} //@namespace environment

} //@namespace dwl
