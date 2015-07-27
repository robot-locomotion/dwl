#include <model/Cost.h>


namespace dwl
{

namespace model
{

Cost::Cost() : is_cost_map_(false)
{

}


Cost::~Cost()
{

}


void Cost::setCostMap(std::vector<RewardCell> reward_map)
{
	printf(YELLOW "Could not set the cost map because this is not cost map class\n" COLOR_RESET);
}


void Cost::get(AdjacencyMap& adjacency_map,
			   Eigen::Vector3d robot_state,
			   bool terrain_cost)
{
	printf(YELLOW "Could not get the cost because this was not defined\n" COLOR_RESET);
}


bool Cost::isCostMap()
{
	return is_cost_map_;
}


std::string Cost::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
