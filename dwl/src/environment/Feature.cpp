#include <dwl/environment/Feature.h>


namespace dwl
{

namespace environment
{

Feature::Feature() :
		space_discretization_(std::numeric_limits<double>::max(),
				std::numeric_limits<double>::max()),
				robot_(NULL), max_cost_(2.), weight_(1.)
{

}


Feature::~Feature()
{

}


void Feature::reset(robot::Robot* robot)
{
	printf(BLUE "Setting the robot properties in the %s feature \n"
			COLOR_RESET, name_.c_str());
	robot_ = robot;
}


std::string Feature::getName()
{
	return name_;
}


void Feature::computeCost(double& cost_value,
						  const Terrain& terrain_info)
{
	printf(YELLOW "Could not computed the cost value of the terrain because"
			" was not defined\n" COLOR_RESET);
}


void Feature::computeCost(double& cost_value,
						  const RobotAndTerrain& info)
{
	printf(YELLOW "Could not computed the cost value of the robot because was"
			" not defined\n" COLOR_RESET);
}


void Feature::setWeight(double weight)
{
	weight_ = weight;
}


void Feature::getWeight(double& weight)
{
	weight = weight_;
}


void Feature::setNeighboringArea(double min_x, double max_x, double min_y,
		double max_y, double resolution)
{
	neightboring_area_.max_x = max_x;
	neightboring_area_.min_x = min_x;
	neightboring_area_.max_y = max_y;
	neightboring_area_.min_y = min_y;
	neightboring_area_.resolution = resolution;
}

} //@namespace environment
} //@namespace dwl
