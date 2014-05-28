#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : gridmap_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()), is_added_feature_(false),
		is_added_search_area_(false), cell_size_(0.04)
{
	interest_area_.max_x = std::numeric_limits<double>::max();
	interest_area_.min_x = -std::numeric_limits<double>::max();
	interest_area_.max_y = std::numeric_limits<double>::max();
	interest_area_.min_y = -std::numeric_limits<double>::max();
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
	//pthread_mutex_lock(&environment_lock_);
	features_.push_back(feature);
	//pthread_mutex_unlock(&environment_lock_); //TODO: I had problems when re-run the code. For this reason I had to remove these lines. Figure out why!!!
	is_added_feature_ = true;
}


void RewardMap::removeFeature(std::string feature_name)
{
	for (int i = 0; i < features_.size(); i++) {
		if (feature_name == features_[i]->getName().c_str()) {
			printf(GREEN "Removing the %s feature\n" COLOR_RESET, features_[i]->getName().c_str());
			//pthread_mutex_lock(&environment_lock_);
			features_.erase(features_.begin() + i);
			//pthread_mutex_unlock(&environment_lock_);

			return;
		}
		else if (i == features_.size() - 1) {
			printf(YELLOW "Could not remove the %s feature\n" COLOR_RESET, feature_name.c_str());
		}
	}
}


void RewardMap::removeRewardOutsideInterestRegion(Eigen::Vector3d robot_state)
{
	Eigen::Vector2d point;
	Key key;
	for (std::vector<Cell>::iterator it = reward_gridmap_.begin(); it != reward_gridmap_.end();) {
		key = it->cell_key.grid_id;
		point(0) = gridmap_.keyToCoord(key.key[0], true);
		point(1) = gridmap_.keyToCoord(key.key[1], true);

		double xc = point(0) - robot_state(0);
		double yc = point(1) - robot_state(1);
		double yaw = robot_state(2);
		if (xc * cos(yaw) + yc * sin(yaw) >= 0.0) {
			if (pow(xc * cos(yaw) + yc * sin(yaw), 2) / pow(5.5, 2) + pow(xc * sin(yaw) - yc * cos(yaw), 2) / pow(1.5, 2) > 1)
				reward_gridmap_.erase(it);
			else
				++it;
		} else {
			if (pow(xc, 2) + pow(yc, 2) > pow(1.5, 2))
				reward_gridmap_.erase(it);
			else
				++it;
		}
	}
}


void RewardMap::setInterestRegion(double min_x, double max_x, double min_y, double max_y)
{
	interest_area_.min_x = min_x;
	interest_area_.max_x = max_x;
	interest_area_.min_y = min_y;
	interest_area_.max_y = max_y;
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
	reward_gridmap_.push_back(cell);
}


void RewardMap::removeCellToRewardMap(CellKey cell)
{
	int index;
	for (int i = 0; reward_gridmap_.size(); i++) {
		if (reward_gridmap_[i].cell_key.grid_id.key[0] == cell.grid_id.key[0] && reward_gridmap_[i].cell_key.grid_id.key[1] == cell.grid_id.key[1]
		&& reward_gridmap_[i].cell_key.height_id == cell.height_id) {
			index = i;

			break;
		}
	}
	reward_gridmap_.erase(reward_gridmap_.begin() + index);
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


std::vector<Cell> RewardMap::getRewardMap()
{
	return reward_gridmap_;
}


} //@namepace dwl

} //@namespace environment
