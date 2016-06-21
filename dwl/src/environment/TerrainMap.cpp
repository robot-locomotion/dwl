#include <dwl/environment/TerrainMap.h>


namespace dwl
{

namespace environment
{

TerrainMap::TerrainMap() :
		terrain_discretization_(0.04, 0.04, M_PI / 200),
		obstacle_discretization_(0.04, 0.04, M_PI / 200),
		average_cost_(0), terrain_information_(false),
		obstacle_information_(false),
		terrain_resolution_(std::numeric_limits<double>::max()),
		obstacle_resolution_(std::numeric_limits<double>::max())
{

}


TerrainMap::~TerrainMap()
{

}


void TerrainMap::setRewardMap(std::vector<RewardCell> reward_map)
{
	// Cleaning the old information
	CostMap empty_terrain_cost_map;
	HeightMap empty_terrain_height_map;
	costmap_.swap(empty_terrain_cost_map);
	heightmap_.swap(empty_terrain_height_map);
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	Vertex vertex_2d;
	if (reward_map.size() != 0) {
		// Setting the resolution
		terrain_resolution_ = reward_map[0].plane_size;
		setTerrainResolution(terrain_resolution_, true);
		setTerrainResolution(reward_map[0].height_size, false);

		for (unsigned int i = 0; i < reward_map.size(); i++) {
			// Building a cost-map for a every 3d vertex
			terrain_discretization_.keyToVertex(vertex_2d, reward_map[i].key, true);
			costmap_[vertex_2d] = -reward_map[i].reward;

			// Building a height map (3d vertex) according to certain 2d position (2d vertex)
			double height;
			terrain_discretization_.keyToCoord(height, reward_map[i].key.z, false);
			heightmap_[vertex_2d] = height;

			average_cost_ += -reward_map[i].reward;
		}

		// Computing the average cost of the terrain
		average_cost_ /= reward_map.size();

		terrain_information_ = true;
	}
}


void TerrainMap::setObstacleMap(std::vector<Cell> obstacle_map)
{
	// Cleaning the old information
	ObstacleMap empty_terrain_obstacle_map;
	obstaclemap_.swap(empty_terrain_obstacle_map);

	//Storing the obstacle-map data according the vertex id
	Vertex vertex_2d;
	if (obstacle_map.size() != 0) {
		// Setting the obstacle resolution
		obstacle_resolution_ = obstacle_map[0].plane_size;
		setObstacleResolution(obstacle_resolution_, true);
		setObstacleResolution(obstacle_resolution_, false);

		for (unsigned int i = 0; i < obstacle_map.size(); i++) {
			// Building a cost map for a every 3d vertex
			obstacle_discretization_.keyToVertex(vertex_2d, obstacle_map[i].key, true);
			obstaclemap_[vertex_2d] = true;
		}

		obstacle_information_ = true;
	}
}


void TerrainMap::setTerrainResolution(double resolution,
									  bool plane)
{
	terrain_discretization_.setEnvironmentResolution(resolution, plane);
}


void TerrainMap::setObstacleResolution(double resolution,
									   bool plane)
{
	obstacle_discretization_.setEnvironmentResolution(resolution, plane);
}


void TerrainMap::setStateResolution(double position_resolution,
									double angular_resolution)
{
	terrain_discretization_.setStateResolution(position_resolution,
											   angular_resolution);
	obstacle_discretization_.setStateResolution(position_resolution,
												angular_resolution);
}
}


void TerrainMap::getTerrainCostMap(CostMap& costmap)
{
	costmap = costmap_;
}


void TerrainMap::getTerrainHeightMap(HeightMap& heightmap)
{
	heightmap = heightmap_;
}


void TerrainMap::getObstacleMap(ObstacleMap& obstaclemap)
{
	obstaclemap = obstaclemap_;
}


double TerrainMap::getTerrainResolution()
{
	return terrain_resolution_;
}


double TerrainMap::getObstacleResolution()
{
	return obstacle_resolution_;
}


double TerrainMap::getAverageCostOfTerrain()
{
	return average_cost_;
}


SpaceDiscretization& TerrainMap::getTerrainSpaceModel()
{
	return terrain_discretization_;
}


SpaceDiscretization& TerrainMap::getObstacleSpaceModel()
{
	return obstacle_discretization_;
}


bool TerrainMap::isTerrainInformation()
{
	return terrain_information_;
}


bool TerrainMap::isObstacleInformation()
{
	return obstacle_information_;
}

} //@namespace environment
} //@namespace dwl
