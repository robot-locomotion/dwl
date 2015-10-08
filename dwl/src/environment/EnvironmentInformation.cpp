#include <environment/EnvironmentInformation.h>


namespace dwl
{

namespace environment
{

EnvironmentInformation::EnvironmentInformation() :
		terrain_space_discretization_(0.04, 0.04, M_PI / 200),
		obstacle_space_discretization_(0.04, 0.04, M_PI / 200), average_terrain_cost_(0),
		terrain_information_(false), obstacle_information_(false),
		terrain_resolution_(std::numeric_limits<double>::max()),
		obstacle_resolution_(std::numeric_limits<double>::max())
{

}


EnvironmentInformation::~EnvironmentInformation()
{

}


void EnvironmentInformation::setEnvironmentInformation(std::vector<RewardCell> reward_map)
{
	// Cleaning the old information
	CostMap empty_terrain_cost_map;
	HeightMap empty_terrain_height_map;
	terrain_cost_map_.swap(empty_terrain_cost_map);
	terrain_height_map_.swap(empty_terrain_height_map);
	average_terrain_cost_ = 0;

	// Storing the cost-map data according the vertex id
	Vertex vertex_2d;
	if (reward_map.size() != 0) {
		// Setting the resolution
		terrain_resolution_ = reward_map[0].plane_size;
		setTerrainResolution(terrain_resolution_, true);
		setTerrainResolution(reward_map[0].height_size, false);

		for (int i = 0; i < (int) reward_map.size(); i++) {
			// Building a cost-map for a every 3d vertex
			terrain_space_discretization_.keyToVertex(vertex_2d, reward_map[i].key, true);
			terrain_cost_map_[vertex_2d] = -reward_map[i].reward;

			// Building a height map (3d vertex) according to certain 2d position (2d vertex)
			double height;
			terrain_space_discretization_.keyToCoord(height, reward_map[i].key.z, false);
			terrain_height_map_[vertex_2d] = height;

			average_terrain_cost_ += -reward_map[i].reward;
		}

		// Computing the average cost of the terrain
		average_terrain_cost_ /= reward_map.size();

		terrain_information_ = true;
	}
}


void EnvironmentInformation::setEnvironmentInformation(std::vector<Cell> obstacle_map)
{
	// Cleaning the old information
	ObstacleMap empty_terrain_obstacle_map;
	obstacle_map_.swap(empty_terrain_obstacle_map);

	//Storing the obstacle-map data according the vertex id
	Vertex vertex_2d;
	if (obstacle_map.size() != 0) {
		// Setting the obstacle resolution
		obstacle_resolution_ = obstacle_map[0].plane_size;
		setObstacleResolution(obstacle_resolution_, true);
		setObstacleResolution(obstacle_resolution_, false);

		for (int i = 0; i < (int) obstacle_map.size(); i++) {
			// Building a cost map for a every 3d vertex
			obstacle_space_discretization_.keyToVertex(vertex_2d, obstacle_map[i].key, true);
			obstacle_map_[vertex_2d] = true;
		}

		obstacle_information_ = true;
	}
}


void EnvironmentInformation::setTerrainResolution(double resolution, bool plane)
{
	terrain_space_discretization_.setEnvironmentResolution(resolution, plane);
}


void EnvironmentInformation::setObstacleResolution(double resolution, bool plane)
{
	obstacle_space_discretization_.setEnvironmentResolution(resolution, plane);
}


void EnvironmentInformation::setStateResolution(double position_resolution,
		double angular_resolution)
{
	terrain_space_discretization_.setStateResolution(position_resolution, angular_resolution);
	obstacle_space_discretization_.setStateResolution(position_resolution, angular_resolution);
}


void EnvironmentInformation::getTerrainCostMap(CostMap& costmap)
{
	costmap = terrain_cost_map_;
}


void EnvironmentInformation::getTerrainHeightMap(HeightMap& heightmap)
{
	heightmap = terrain_height_map_;
}


void EnvironmentInformation::getObstacleMap(ObstacleMap& obstaclemap)
{
	obstaclemap = obstacle_map_;
}


double EnvironmentInformation::getTerrainResolution()
{
	return terrain_resolution_;
}


double EnvironmentInformation::getObstacleResolution()
{
	return obstacle_resolution_;
}


double EnvironmentInformation::getAverageCostOfTerrain()
{
	return average_terrain_cost_;
}


SpaceDiscretization& EnvironmentInformation::getTerrainSpaceModel()
{
	return terrain_space_discretization_;
}


SpaceDiscretization& EnvironmentInformation::getObstacleSpaceModel()
{
	return obstacle_space_discretization_;
}


bool EnvironmentInformation::isTerrainInformation()
{
	return terrain_information_;
}


bool EnvironmentInformation::isObstacleInformation()
{
	return obstacle_information_;
}

} //@namespace environment
} //@namespace dwl
