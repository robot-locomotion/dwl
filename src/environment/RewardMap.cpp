#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : gridmap_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()), is_added_feature_(false),
		is_added_search_area_(false), interest_radius_x_(std::numeric_limits<double>::max()), interest_radius_y_(std::numeric_limits<double>::max()),
		cell_size_(0.04)
{

}


RewardMap::~RewardMap()
{
	if (is_added_feature_) {
	for (std::vector<Feature*>::iterator i = features_.begin(); i != features_.end(); i++)
		delete *i;
	}
}


void RewardMap::addFeature(Feature* feature)
{
	printf(GREEN "Adding the %s feature\n" COLOR_RESET, feature->getName().c_str());
	features_.push_back(feature);
	is_added_feature_ = true;
}


void RewardMap::removeFeature(std::string feature_name)
{
	for (int i = 0; i < features_.size(); i++) {
		if (feature_name == features_[i]->getName().c_str()) {
			printf(GREEN "Removing the %s feature\n" COLOR_RESET, features_[i]->getName().c_str());
			features_.erase(features_.begin() + i);

			return;
		}
		else if (i == features_.size() - 1) {
			printf(YELLOW "Could not remove the %s feature\n" COLOR_RESET, feature_name.c_str());
		}
	}
}


void RewardMap::removeRewardOutsideInterestRegion(Eigen::Vector3d robot_state)
{
	for (std::map<Vertex, Cell>::iterator vertex_iter = reward_gridmap_.begin();
			vertex_iter != reward_gridmap_.end();
			vertex_iter++)
	{
		Vertex v = vertex_iter->first;
		Eigen::Vector2d point = gridmap_.vertexToCoord(v);

		double xc = point(0) - robot_state(0);
		double yc = point(1) - robot_state(1);
		double yaw = robot_state(2);
		if (xc * cos(yaw) + yc * sin(yaw) >= 0.0) {
			if (pow(xc * cos(yaw) + yc * sin(yaw), 2) / pow(interest_radius_y_, 2) + pow(xc * sin(yaw) - yc * cos(yaw), 2) / pow(interest_radius_x_, 2) > 1) {
				reward_gridmap_.erase(v);
				terrain_heightmap_.erase(v);
			}
		} else {
			if (pow(xc, 2) + pow(yc, 2) > pow(1.5, 2)) {
				reward_gridmap_.erase(v);
				terrain_heightmap_.erase(v);
			}
		}
	}
}


void RewardMap::setInterestRegion(double radius_x, double radius_y)
{
	interest_radius_x_ = radius_x;
	interest_radius_y_ = radius_y;
}


void RewardMap::getCell(Cell& cell, double reward, Terrain terrain_info)
{
	Key grip_key;
	Eigen::Vector2d cell_position;
	cell_position(0) = terrain_info.position(0);
	cell_position(1) = terrain_info.position(1);
	gridmap_.coordToKeyChecked(grip_key, cell_position);

	cell.cell_key.grid_id = grip_key;
	cell.cell_key.height_id = gridmap_.coordToKey((double) terrain_info.position(2), false);
	cell.reward = reward;
	cell.size = cell_size_;
}


void RewardMap::getCell(CellKey& cell_key, Eigen::Vector3d position)
{
	Key grid_key;
	Eigen::Vector2d cell_position;
	cell_position(0) = position(0);
	cell_position(1) = position(1);
	gridmap_.coordToKeyChecked(grid_key, cell_position);

	cell_key.grid_id = grid_key;
	cell_key.height_id = gridmap_.coordToKey((double) position(2), false);
}


void RewardMap::addCellToRewardMap(Cell cell)
{
	Vertex vertex_id = gridmap_.gridMapKeyToVertex(cell.cell_key.grid_id);
	reward_gridmap_[vertex_id] = cell;
}


void RewardMap::removeCellToRewardMap(CellKey cell)
{
	Vertex v = gridmap_.gridMapKeyToVertex(cell.grid_id);
	reward_gridmap_.erase(v);
}


void RewardMap::addCellToTerrainHeightMap(CellKey cell)
{
	Vertex v = gridmap_.gridMapKeyToVertex(cell.grid_id);
	double height = gridmap_.keyToCoord(cell.height_id, false);
	terrain_heightmap_[v] = height;
}


void RewardMap::removeCellToTerrainHeightMap(CellKey cell)
{
	Vertex v = gridmap_.gridMapKeyToVertex(cell.grid_id);
	terrain_heightmap_.erase(v);
}


void RewardMap::addSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_resolution)
{
	SearchArea search_area;
	search_area.min_x = min_x;
	search_area.max_x = max_x;
	search_area.min_y = min_y;
	search_area.max_y = max_y;
	search_area.min_z = min_z;
	search_area.max_z = max_z;
	search_area.grid_resolution = grid_resolution;

	search_areas_.push_back(search_area);

	if (grid_resolution < gridmap_.getResolution(true))
		gridmap_.setResolution(grid_resolution, true);

	is_added_search_area_ = true;
}


void RewardMap::setNeighboringArea(int back_neighbors, int front_neighbors, int left_neighbors, int right_neighbors, int bottom_neighbors, int top_neighbors)
{
	neighboring_area_.min_x = back_neighbors;
	neighboring_area_.max_x = front_neighbors;
	neighboring_area_.min_y = left_neighbors;
	neighboring_area_.max_y = right_neighbors;
	neighboring_area_.min_z = bottom_neighbors;
	neighboring_area_.max_z = top_neighbors;
}


double RewardMap::getResolution(bool gridmap)
{
	return gridmap_.getResolution(gridmap);
}


void RewardMap::setModelerResolution(double resolution)
{
	gridmap_.setResolution(resolution, false);
}


std::map<Vertex, Cell> RewardMap::getRewardMap()
{
	return reward_gridmap_;
}

} //@namepace dwl
} //@namespace environment
