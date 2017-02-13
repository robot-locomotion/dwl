#include <dwl/environment/TerrainMap.h>


namespace dwl
{

namespace environment
{

TerrainMap::TerrainMap() :
		terrain_discretization_(0.04, 0.04, M_PI / 200),
		obstacle_discretization_(0.04, 0.04, M_PI / 200),
		average_cost_(0.), max_cost_(0.), terrain_information_(false),
		obstacle_information_(false),
		terrain_resolution_(std::numeric_limits<double>::max()),
		height_resolution_(std::numeric_limits<double>::max()),
		obstacle_resolution_(std::numeric_limits<double>::max())
{

}


TerrainMap::~TerrainMap()
{

}


void TerrainMap::setTerrainMap(TerrainData terrain_map)
{
	// Cleaning the old information
	CostMap empty_terrain_cost_map;
	HeightMap empty_terrain_height_map;
	costmap_.swap(empty_terrain_cost_map);
	heightmap_.swap(empty_terrain_height_map);
	average_cost_ = 0;

	// Storing the terrain data according the vertex id
	Vertex vertex_2d;
	if (terrain_map.size() != 0) {
		// Setting the resolution
		terrain_resolution_ = terrain_map[0].plane_size;
		height_resolution_ = terrain_map[0].height_size;
		setTerrainResolution(terrain_resolution_, true);
		setTerrainResolution(height_resolution_, false);

		for (unsigned int i = 0; i < terrain_map.size(); i++) {
			// Building a cost-map for a every 3d vertex
			terrain_discretization_.keyToVertex(vertex_2d, terrain_map[i].key, true);
			double cost_value = -terrain_map[i].reward;
			costmap_[vertex_2d] = cost_value;

			// Setting up the maximum cost value
			if (cost_value > max_cost_)
				max_cost_ = cost_value;

			// Building a height map (3d vertex) according to certain 2d position (2d vertex)
			double height;
			terrain_discretization_.keyToCoord(height, terrain_map[i].key.z, false);
			heightmap_[vertex_2d] = height;

			average_cost_ += -terrain_map[i].reward;
		}

		// Computing the average cost of the terrain
		average_cost_ /= terrain_map.size();

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


Weight TerrainMap::getTerrainCost(const Vertex& vertex)
{
	CostMap::iterator cost_it = costmap_.find(vertex);
	if (cost_it != costmap_.end())
		return cost_it->second;
	else
		return max_cost_; // pass the maximum cost for unknown cases
}


Weight TerrainMap::getTerrainCost(const Eigen::Vector2d& position)
{
	// Getting the vertex
	Vertex vertex;
	terrain_discretization_.coordToVertex(vertex, position);

	// Getting the cost value
	return getTerrainCost(vertex);
}


double TerrainMap::getTerrainHeight(const Vertex& vertex)
{
	HeightMap::iterator height_it = heightmap_.find(vertex);
	if (height_it != heightmap_.end())
		return height_it->second + height_resolution_ / 2;
	else
		return 0.;
}


double TerrainMap::getTerrainHeight(const Eigen::Vector2d position)
{
	// Getting the vertex
	Vertex vertex;
	terrain_discretization_.coordToVertex(vertex, position);

	// Getting the height value
	return getTerrainHeight(vertex);
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


double TerrainMap::getHeightResolution()
{
	return height_resolution_;
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
