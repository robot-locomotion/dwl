#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : space_discretization_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
		is_added_feature_(false), is_added_search_area_(false), interest_radius_x_(std::numeric_limits<double>::max()),
		interest_radius_y_(std::numeric_limits<double>::max()), min_height_(std::numeric_limits<double>::max())
{

}


RewardMap::~RewardMap()
{
	if (is_added_feature_) {
	for (std::vector<Feature*>::iterator i = features_.begin(); i != features_.end(); i++)
		delete *i;
	}
}


void RewardMap::reset()
{
	terrain_rewardmap_.clear();
	terrain_heightmap_.clear();
}


void RewardMap::addFeature(Feature* feature)
{
	double weight;
	feature->getWeight(weight);
	printf(GREEN "Adding the %s feature with a weight of %f\n" COLOR_RESET, feature->getName().c_str(), weight);
	features_.push_back(feature);
	is_added_feature_ = true;
}


void RewardMap::removeFeature(std::string feature_name)
{
	int num_feature = features_.size();
	for (int i = 0; i < num_feature; i++) {
		if (feature_name == features_[i]->getName().c_str()) {
			printf(GREEN "Removing the %s feature\n" COLOR_RESET, features_[i]->getName().c_str());
			features_.erase(features_.begin() + i);

			return;
		}
		else if (i == num_feature - 1) {
			printf(YELLOW "Could not remove the %s feature\n" COLOR_RESET, feature_name.c_str());
		}
	}
}


void RewardMap::removeRewardOutsideInterestRegion(Eigen::Vector3d robot_state)
{
	// Getting the orientation of the body
	double yaw = robot_state(2);

	for (std::map<Vertex,RewardCell>::iterator vertex_iter = terrain_rewardmap_.begin();
			vertex_iter != terrain_rewardmap_.end();
			vertex_iter++)
	{
		Vertex v = vertex_iter->first;
		Eigen::Vector2d point;
		space_discretization_.vertexToCoord(point, v);

		double xc = point(0) - robot_state(0);
		double yc = point(1) - robot_state(1);
		if (xc * cos(yaw) + yc * sin(yaw) >= 0.0) {
			if (pow(xc * cos(yaw) + yc * sin(yaw), 2) / pow(interest_radius_y_, 2) + pow(xc * sin(yaw) - yc * cos(yaw), 2) / pow(interest_radius_x_, 2) > 1) {
				terrain_rewardmap_.erase(v);
				terrain_heightmap_.erase(v);
			}
		} else {
			if (pow(xc, 2) + pow(yc, 2) > pow(interest_radius_x_, 2)) {
				terrain_rewardmap_.erase(v);
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


void RewardMap::getCell(RewardCell& cell, double reward, Terrain terrain_info)
{
	space_discretization_.coordToKeyChecked(cell.key, terrain_info.position);
	cell.reward = reward;
	cell.plane_size = space_discretization_.getEnvironmentResolution(true);
	cell.height_size = space_discretization_.getEnvironmentResolution(false);
}


void RewardMap::getCell(Key& key, Eigen::Vector3d position)
{
	space_discretization_.coordToKeyChecked(key, position);
}


void RewardMap::addCellToRewardMap(RewardCell cell)
{
	Vertex vertex_id;
	space_discretization_.keyToVertex(vertex_id, cell.key, true);
	terrain_rewardmap_[vertex_id] = cell;
}


void RewardMap::removeCellToRewardMap(Vertex cell_vertex)
{
	terrain_rewardmap_.erase(cell_vertex);
}


void RewardMap::addCellToTerrainHeightMap(Vertex cell_vertex, double height)
{
	terrain_heightmap_[cell_vertex] = height;

	if (height < min_height_)
		min_height_ = height;
}


void RewardMap::removeCellToTerrainHeightMap(Vertex cell_vertex)
{
	terrain_heightmap_.erase(cell_vertex);
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
	search_area.resolution = grid_resolution;

	search_areas_.push_back(search_area);

	if (grid_resolution < space_discretization_.getEnvironmentResolution(true)) {
		space_discretization_.setEnvironmentResolution(grid_resolution, true);
		space_discretization_.setStateResolution(grid_resolution);
	}

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


double RewardMap::getResolution(bool plane)
{
	return space_discretization_.getEnvironmentResolution(plane);
}


void RewardMap::setResolution(double resolution, bool plane)
{
	space_discretization_.setEnvironmentResolution(resolution, plane);
}


const std::map<Vertex,RewardCell>& RewardMap::getRewardMap() const
{
	return terrain_rewardmap_;
}

} //@namepace dwl
} //@namespace environment
