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
	CostMap empty_terrain_cost_map;
	HeightMap empty_terrain_height_map;
	terrain_cost_map_.swap(empty_terrain_cost_map);
	terrain_height_map_.swap(empty_terrain_height_map);
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	unsigned int vertex_id;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < reward_map.size(); i++) {
		// Setting the resolution of the environment
		if (i == 0) {
			printf("Setting the grid resolution to %f\n", reward_map[i].size);
			setResolution(reward_map[i].size, true);
			setResolution(0.02, false);//reward_map[i]//TODO
		}

		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		terrain_cost_map_[vertex_id] = - reward_map[i].reward;
		terrain_height_map_[vertex_id] = gridmap_.keyToCoord(reward_map[i].cell_key.height_id, false);
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
	costmap = terrain_cost_map_;
}


void EnvironmentInformation::getTerrainHeightMap(HeightMap& heightmap)
{
	heightmap = terrain_height_map_;
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
