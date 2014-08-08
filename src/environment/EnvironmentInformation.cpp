#include <environment/EnvironmentInformation.h>


namespace dwl
{

namespace environment
{

EnvironmentInformation::EnvironmentInformation() : space_discretization_(0.04, 0.04, M_PI / 24), average_cost_(0),
		terrain_cost_information_(false), terrain_obstacle_information_(false)
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
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	Vertex vertex_2d;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < reward_map.size(); i++) {
		// Setting the resolution of the environment
		if ((reward_map[i].plane_size != space_discretization_.getEnvironmentResolution(true)) ||
				(reward_map[i].height_size != space_discretization_.getEnvironmentResolution(false))) {
			printf("Setting the plane resolution to %f\n", reward_map[i].plane_size);
			printf("Setting the height resolution to %f\n", reward_map[i].height_size);
			setEnvironmentResolution(reward_map[i].plane_size, true);
			setEnvironmentResolution(reward_map[i].height_size, false);
		}

		// Building a cost map for a every 3d vertex
		space_discretization_.keyToVertex(vertex_2d, reward_map[i].key, true);
		terrain_cost_map_[vertex_2d] = -reward_map[i].reward;

		// Building a height map (3d vertex) according to certain 2d position (2d vertex)
		double height;
		space_discretization_.keyToCoord(height, reward_map[i].key.z, false);
		terrain_height_map_[vertex_2d] = height;

		average_cost_ += -reward_map[i].reward;

		if (reward_map[i].plane_size < resolution) {
			resolution = reward_map[i].plane_size;
		}
	}

	// Computing the average cost of the terrain
	average_cost_ /= reward_map.size();

	terrain_cost_information_ = true;
}


void EnvironmentInformation::setEnvironmentInformation(std::vector<Cell> obstacle_map)
{
	// Cleaning the old information
	ObstacleMap empty_terrain_obstacle_map;
	terrain_obstacle_map_.swap(empty_terrain_obstacle_map);

	//Storing the obstacle-map data according the vertex id
	Vertex vertex_2d;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < obstacle_map.size(); i++) {
		// Setting the resolution of the environment
		if ((obstacle_map[i].plane_size != space_discretization_.getEnvironmentResolution(true)) ||
				(obstacle_map[i].height_size != space_discretization_.getEnvironmentResolution(false))) {
			printf("Setting the plane resolution to %f\n", obstacle_map[i].plane_size);
			printf("Setting the height resolution to %f\n", obstacle_map[i].height_size);
			setEnvironmentResolution(obstacle_map[i].plane_size, true);
			setEnvironmentResolution(obstacle_map[i].height_size, false);
		}

		// Building a cost map for a every 3d vertex
		space_discretization_.keyToVertex(vertex_2d, obstacle_map[i].key, true);
		terrain_obstacle_map_[vertex_2d] = true;

		// Building a height map (3d vertex) according to certain 2d position (2d vertex)
		/*double height;
		space_discretization_.keyToCoord(height, reward_map[i].key.z, false);
		terrain_height_map_[vertex_2d] = height;*/


		if (obstacle_map[i].plane_size < resolution) {
			resolution = obstacle_map[i].plane_size;
		}
	}


	terrain_obstacle_information_ = true;
}


void EnvironmentInformation::setEnvironmentResolution(double resolution, bool plane)
{
	space_discretization_.setEnvironmentResolution(resolution, plane);
}


void EnvironmentInformation::setStateResolution(double position_resolution, double angular_resolution)
{
	space_discretization_.setStateResolution(position_resolution, angular_resolution);
}


void EnvironmentInformation::getTerrainCostMap(CostMap& costmap)
{
	costmap = terrain_cost_map_;
}


void EnvironmentInformation::getTerrainHeightMap(HeightMap& heightmap)
{
	heightmap = terrain_height_map_;
}


void EnvironmentInformation::getTerrainObstacleMap(ObstacleMap& obstaclemap)
{
	obstaclemap = terrain_obstacle_map_;
}


double EnvironmentInformation::getAverageCostOfTerrain()
{
	return average_cost_;
}


const SpaceDiscretization& EnvironmentInformation::getSpaceModel() const
{
	return space_discretization_;
}


bool EnvironmentInformation::isTerrainCostInformation()
{
	return terrain_cost_information_;
}


bool EnvironmentInformation::isTerrainObstacleInformation()
{
	return terrain_obstacle_information_;
}

} //@namespace environment
} //@namespace dwl
