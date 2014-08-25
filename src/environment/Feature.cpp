#include <environment/Feature.h>


namespace dwl
{

namespace environment
{

Feature::Feature() : max_reward_(-2), robot_(NULL), weight_(1),
		space_discretization_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
{
	//space_discretization_.setStateResolution(0.04, M_PI / 24);
}


Feature::~Feature()
{

}


void Feature::reset(robot::Robot* robot)
{
	printf(BLUE "Setting the robot properties in the %s feature \n" COLOR_RESET, name_.c_str());
	robot_ = robot;
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

} //@namespace environment
} //@namespace dwl
