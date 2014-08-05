#include <planning/Cost.h>


namespace dwl
{

namespace planning
{

Cost::Cost() : is_cost_map_(false)
{

}


Cost::~Cost()
{

}


void Cost::setCostMap(std::vector<RewardCell> reward_map)
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


bool Cost::isCostMap()
{
	return is_cost_map_;
}


std::string dwl::planning::Cost::getName()
{
	return name_;
}

} //@namespace planning
} //@namespace dwl
