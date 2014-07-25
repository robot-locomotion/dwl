#include <environment/EnvironmentInformation.h>


namespace dwl
{

namespace environment
{

EnvironmentInformation::EnvironmentInformation() : space_discretization_(0.04, 0.04, M_PI / 24), average_cost_(0),
		terrain_information_(false)
{
	//TODO Thinks about the setting the resolutions
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
	Vertex vertex_2d;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < reward_map.size(); i++) {
		// Setting the resolution of the environment
		if (i == 0) {
			printf("Setting the grid resolution to %f\n", reward_map[i].size);
			setResolution(reward_map[i].size);
		}

		// Building a cost map for a every 3d vertex
		space_discretization_.keyToVertex(vertex_2d, reward_map[i].key, true);
		terrain_cost_map_[vertex_2d] = -reward_map[i].reward;

		// Building a height map (3d vertex) according to certain 2d position (2d vertex)
		double height;
		space_discretization_.keyToCoord(height, reward_map[i].key.z);
		terrain_height_map_[vertex_2d] = height;

		average_cost_ += -reward_map[i].reward;

		if (reward_map[i].size < resolution) {
			resolution = reward_map[i].size;
		}
	}

	// Computing the average cost of the terrain
	average_cost_ /= reward_map.size();

	terrain_information_ = true;
}


void EnvironmentInformation::setResolution(double resolution)
{
	space_discretization_.setEnvironmentResolution(resolution);
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


const SpaceDiscretization& EnvironmentInformation::getSpaceModel() const
{
	return space_discretization_;
}


bool EnvironmentInformation::isTerrainInformation()
{
	return terrain_information_;
}

} //@namespace environment
} //@namespace dwl
