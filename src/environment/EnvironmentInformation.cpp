#include <environment/EnvironmentInformation.h>


namespace dwl
{

namespace environment
{

EnvironmentInformation::EnvironmentInformation() : gridmap_(0.04, 0.02), average_cost_(0), terrain_information_(false)
{

}


EnvironmentInformation::~EnvironmentInformation()
{

}


void EnvironmentInformation::setEnvironmentInformation(std::vector<Cell> reward_map)
{
	// Cleaning the old information
	std::map<Vertex, Weight> empty_terrain_cost_per_vertex;
	terrain_cost_map_.swap(empty_terrain_cost_per_vertex);
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	unsigned int vertex_id;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < reward_map.size(); i++) {
		// Setting the resolution of the environment
		if (i == 0) {
			printf("Setting the grid resolution to %f\n m", reward_map[i].size);
			setResolution(reward_map[i].size, true);
			setResolution(reward_map[i].size, false);
		}

		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		terrain_cost_map_[vertex_id] = - reward_map[i].reward;
		average_cost_ += - reward_map[i].reward;
		if (reward_map[i].size < resolution) {
			resolution = reward_map[i].size;
		}
	}
	average_cost_ /= reward_map.size();

	terrain_information_ = true;
}


void EnvironmentInformation::setResolution(double resolution, bool gridmap)
{
	gridmap_.setResolution(resolution, gridmap);
}


void EnvironmentInformation::getTerrainCostMap(CostMap& costmap)
{
	CostMap terrain;
	costmap = terrain_cost_map_;
}


double EnvironmentInformation::getAverageCostOfTerrain()
{
	return average_cost_;
}


const PlaneGrid& EnvironmentInformation::getGridModel() const
{
	return gridmap_;
}


bool EnvironmentInformation::isTerrainInformation()
{
	return terrain_information_;
}


} //@namespace environment
} //@namespace dwl
