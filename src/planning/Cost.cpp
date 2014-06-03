#include <planning/Cost.h>


namespace dwl
{

namespace planning
{

Cost::Cost() : is_cost_map_(false), gridmap_(0.04, 0.02)
{

}


Cost::~Cost()
{

}


void Cost::setCostMap(std::vector<Cell> reward_map)
{
	printf(YELLOW "Could not setted the cost map because this is not cost map class\n" COLOR_RESET);
}


double Cost::get(Eigen::VectorXd state)
{
	printf(YELLOW "Could not getted the cost because this was not defined\n" COLOR_RESET);

	return 0;
}


void Cost::get(AdjacencyMap& adjacency_map, Eigen::Vector3d robot_state, bool terrain_cost)
{
	printf(YELLOW "Could not getted the cost because this was not defined\n" COLOR_RESET);
}


void Cost::setGridMapResolution(double resolution)
{
	// Setting the gridmap resolution
	gridmap_.setResolution(resolution, true);
	gridmap_.setResolution(resolution, false);
}


bool Cost::isCostMap()
{
	return is_cost_map_;
}

} //@namespace planning

} //@namespace dwl
