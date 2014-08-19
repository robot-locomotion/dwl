#include <environment/Feature.h>


namespace dwl
{

namespace environment
{

Feature::Feature() : weight_(1), gain_(1), space_discretization_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
{

}


Feature::~Feature()
{

}


std::string Feature::getName()
{
	return name_;
}


void Feature::computeReward(double& reward_value, Terrain terrain_info)
{
	printf(YELLOW "Could not computed the reward value of the terrain because was not defined\n" COLOR_RESET);
}


void Feature::computeReward(double& reward_value, RobotAndTerrain info)
{
	printf(YELLOW "Could not computed the reward value of the robot because was not defined\n" COLOR_RESET);
}


void Feature::setWeight(double weight)
{
	weight_ = weight;
}


void Feature::getWeight(double& weight)
{
	weight = weight_;
}


void Feature::setNeighboringArea(double min_x, double max_x, double min_y, double max_y, double resolution)
{
	neightboring_area_.max_x = max_x;
	neightboring_area_.min_x = min_x;
	neightboring_area_.max_y = max_y;
	neightboring_area_.min_y = min_y;
	neightboring_area_.grid_resolution = resolution;
}


void Feature::setGainFeature(double gain)
{
	gain_ = gain;
}

} //@namespace environment
} //@namespace dwl
