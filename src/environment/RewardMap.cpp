#include <environment/RewardMap.h>


namespace dwl
{

namespace environment
{


RewardMap::RewardMap() : space_discretization_(0,0), is_added_feature_(false),
		is_added_search_area_(false), interest_radius_x_(std::numeric_limits<double>::max()),
		interest_radius_y_(std::numeric_limits<double>::max()),
		min_height_(std::numeric_limits<double>::max()), is_first_computation_(true),
		using_cloud_mean_(false), depth_(16)
{
	// Default neighboring area
	setNeighboringArea(-1, 1, -1, 1, -1, 1);
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
	printf(GREEN "Adding the %s feature with a weight of %f\n" COLOR_RESET,
			feature->getName().c_str(), weight);
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


void RewardMap::compute(octomap::OcTree* octomap,
						const Eigen::Vector4d& robot_state)
{
	if (!is_added_search_area_) {
		printf(YELLOW "Warning: adding a default search area \n" COLOR_RESET);
		// Adding a default search area
		addSearchArea(1.5, 4.0, -1.25, 1.25, -0.8, -0.2, 0.04);

		is_added_search_area_ = true;
	}

	// Computing reward map for several search areas
	double yaw = robot_state(3);
	unsigned int area_size = search_areas_.size();
	for (unsigned int n = 0; n < area_size; n++) {
		// Computing the boundary of the gridmap
		Eigen::Vector2d boundary_min, boundary_max;

		boundary_min(0) = search_areas_[n].min_x + robot_state(0);
		boundary_min(1) = search_areas_[n].min_y + robot_state(1);
		boundary_max(0) = search_areas_[n].max_x + robot_state(0);
		boundary_max(1) = search_areas_[n].max_y + robot_state(1);

		double resolution = search_areas_[n].resolution;
		for (double y = boundary_min(1); y <= boundary_max(1); y += resolution) {
			for (double x = boundary_min(0); x <= boundary_max(0); x += resolution) {
				// Computing the rotated coordinate of the point inside the search area
				double xr = (x - robot_state(0)) * cos(yaw) - (y - robot_state(1)) * sin(yaw) + robot_state(0);
				double yr = (x - robot_state(0)) * sin(yaw) + (y - robot_state(1)) * cos(yaw) + robot_state(1);

				// Checking if the cell belongs to dimensions of the map, and also getting the key of this cell
				double z = search_areas_[n].max_z + robot_state(2);
				octomap::OcTreeKey init_key;
				if (!octomap->coordToKeyChecked(xr, yr, z, depth_, init_key)) {
					printf(RED "Cell out of bounds\n" COLOR_RESET);

					return;
				}

				// Finding the cell of the surface
				int r = 0;
				while (z >= search_areas_[n].min_z + robot_state(2)) {
					octomap::OcTreeKey heightmap_key;
					octomap::OcTreeNode* heightmap_node = octomap->search(init_key, depth_);
					heightmap_key[0] = init_key[0];
					heightmap_key[1] = init_key[1];
					heightmap_key[2] = init_key[2] - r;

					heightmap_node = octomap->search(heightmap_key, depth_);
					octomap::point3d height_point = octomap->keyToCoord(heightmap_key, depth_);
					z = height_point(2);
					if (heightmap_node) {
						// Computation of the heightmap
						if (octomap->isNodeOccupied(heightmap_node)) {
							// Getting position of the occupied cell
							Key cell_key;
							Eigen::Vector3d cell_position;
							cell_position(0) = height_point(0);
							cell_position(1) = height_point(1);
							cell_position(2) = height_point(2);
							getCell(cell_key, cell_position);

							Vertex vertex_id;
							space_discretization_.keyToVertex(vertex_id, cell_key, true);

							if (is_first_computation_)
								addCellToTerrainHeightMap(vertex_id, (double) cell_position(2));
							else {
								bool new_status = true;
								if ((terrain_rewardmap_.count(vertex_id) > 0)) {
									// Evaluating if it changed status (height)
									RewardCell reward_cell = terrain_rewardmap_.find(vertex_id)->second;
									if (reward_cell.key.z != cell_key.z) {
										removeCellToRewardMap(vertex_id);
										removeCellToTerrainHeightMap(vertex_id);
									} else
										new_status = false;
								}

								if (new_status)
									addCellToTerrainHeightMap(vertex_id, (double) cell_position(2));
							}
							break;
						}
					}
					r++;
				}
			}
		}
	}

	// Computing the total reward
	for (std::map<Vertex, double>::iterator terrain_iter = terrain_heightmap_.begin();
			terrain_iter != terrain_heightmap_.end();
			terrain_iter++)
	{
		octomap::OcTreeKey heightmap_key;
		Vertex vertex_id = terrain_iter->first;
		Eigen::Vector2d xy_coord;
		space_discretization_.vertexToCoord(xy_coord, vertex_id);

		double height = terrain_iter->second;
		octomap::point3d terrain_point;
		terrain_point(0) = xy_coord(0);
		terrain_point(1) = xy_coord(1);
		terrain_point(2) = height;
		heightmap_key = octomap->coordToKey(terrain_point, depth_);

		if (is_first_computation_)
			computeRewards(octomap, heightmap_key);
		else {
			bool new_status = true;
			if (terrain_rewardmap_.count(vertex_id) > 0) {
				// Evaluating if it's changed status (height)
				RewardCell reward_cell = terrain_rewardmap_.find(vertex_id)->second;

				if (reward_cell.key.z != heightmap_key[2]) {
					removeCellToRewardMap(vertex_id);
					removeCellToTerrainHeightMap(vertex_id);
				} else
					new_status = true;//false;
			}

			if (new_status)
				computeRewards(octomap, heightmap_key);
		}
	}

	// Removing the points that doesn't belong to interest area
	Eigen::Vector3d robot_2dpose;
	robot_2dpose(0) = robot_state(0);
	robot_2dpose(1) = robot_state(1);
	robot_2dpose(2) = robot_state(3);
	removeRewardOutsideInterestRegion(robot_2dpose);

	is_first_computation_ = false;
}


void RewardMap::computeRewards(octomap::OcTree* octomap,
							   const octomap::OcTreeKey& heightmap_key)
{
	Terrain terrain_info;

	std::vector<Eigen::Vector3f> neighbors_position;
	octomap::OcTreeNode* heightmap_node = octomap->search(heightmap_key, depth_);

	// Adding to the cloud the point of interest
	Eigen::Vector3f heightmap_position;
	octomap::point3d heightmap_point = octomap->keyToCoord(heightmap_key, depth_);
	heightmap_position(0) = heightmap_point(0);
	heightmap_position(1) = heightmap_point(1);
	heightmap_position(2) = heightmap_point(2);
	neighbors_position.push_back(heightmap_position);

	// Iterates over the 8 neighboring sets
	octomap::OcTreeKey neighbor_key;
	octomap::OcTreeNode* neighbor_node = heightmap_node;
	bool is_there_neighboring = false;
	for (int i = neighboring_area_.min_z; i < neighboring_area_.max_z + 1; i++) {
		for (int j = neighboring_area_.min_y; j < neighboring_area_.max_y + 1; j++) {
			for (int k = neighboring_area_.min_x; k < neighboring_area_.max_x + 1; k++) {
				neighbor_key[0] = heightmap_key[0] + k;
				neighbor_key[1] = heightmap_key[1] + j;
				neighbor_key[2] = heightmap_key[2] + i;
				neighbor_node = octomap->search(neighbor_key, depth_);

				if (neighbor_node) {
					Eigen::Vector3f neighbor_position;
					octomap::point3d neighbor_point;
					if (octomap->isNodeOccupied(neighbor_node)) {
						neighbor_point = octomap->keyToCoord(neighbor_key, depth_);
						neighbor_position(0) = neighbor_point(0);
						neighbor_position(1) = neighbor_point(1);
						neighbor_position(2) = neighbor_point(2);
						neighbors_position.push_back(neighbor_position);

						is_there_neighboring = true;
					}
				}
			}
		}
	}

	if (is_there_neighboring) {
		// Computing terrain info
		EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
		if (neighbors_position.size() < 3 ||
				math::computeMeanAndCovarianceMatrix(neighbors_position,
													 covariance_matrix,
													 terrain_info.position) == 0)
			return;

		if (!using_cloud_mean_) {
			terrain_info.position(0) = neighbors_position[0](0);
			terrain_info.position(1) = neighbors_position[0](1);
			terrain_info.position(2) = neighbors_position[0](2);
		}

		math::solvePlaneParameters(terrain_info.surface_normal,
								   terrain_info.curvature,
								   covariance_matrix);
	}

	terrain_info.height_map = terrain_heightmap_;
	terrain_info.resolution = space_discretization_.getEnvironmentResolution(true);
	terrain_info.min_height = min_height_;

	// Computing the reward
	if (is_added_feature_) {
		double reward_value, weight, total_reward = 0;
		unsigned int num_feature = features_.size();
		for (unsigned int i = 0; i < num_feature; i++) {
			features_[i]->computeReward(reward_value, terrain_info);
			features_[i]->getWeight(weight);
			total_reward += weight * reward_value;
		}

		RewardCell cell;
		getCell(cell, total_reward, terrain_info);
		addCellToRewardMap(cell);
	}
	else {
		printf(YELLOW "Could not computed the reward of the features because it is necessary to "
				"add at least one\n" COLOR_RESET);
	}
}


void RewardMap::removeRewardOutsideInterestRegion(const Eigen::Vector3d& robot_state)
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
			if (pow(xc * cos(yaw) + yc * sin(yaw), 2) / pow(interest_radius_y_, 2) +
					pow(xc * sin(yaw) - yc * cos(yaw), 2) / pow(interest_radius_x_, 2) > 1) {
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


void RewardMap::setInterestRegion(double radius_x,
								  double radius_y)
{
	interest_radius_x_ = radius_x;
	interest_radius_y_ = radius_y;
}


void RewardMap::getCell(RewardCell& cell,
						double reward,
						const Terrain& terrain_info)
{
	space_discretization_.coordToKeyChecked(cell.key, terrain_info.position);
	cell.reward = reward;
	cell.plane_size = space_discretization_.getEnvironmentResolution(true);
	cell.height_size = space_discretization_.getEnvironmentResolution(false);
}


void RewardMap::getCell(Key& key,
						const Eigen::Vector3d& position)
{
	space_discretization_.coordToKeyChecked(key, position);
}


void RewardMap::addCellToRewardMap(const RewardCell& cell)
{
	Vertex vertex_id;
	space_discretization_.keyToVertex(vertex_id, cell.key, true);
	terrain_rewardmap_[vertex_id] = cell;
}


void RewardMap::removeCellToRewardMap(const Vertex& cell_vertex)
{
	terrain_rewardmap_.erase(cell_vertex);
}


void RewardMap::addCellToTerrainHeightMap(const Vertex& cell_vertex,
										  double height)
{
	terrain_heightmap_[cell_vertex] = height;

	if (height < min_height_)
		min_height_ = height;
}


void RewardMap::removeCellToTerrainHeightMap(const Vertex& cell_vertex)
{
	terrain_heightmap_.erase(cell_vertex);
}


void RewardMap::addSearchArea(double min_x, double max_x,
							  double min_y, double max_y,
							  double min_z, double max_z,
							  double grid_resolution)
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

	if (!is_added_search_area_ ||
			grid_resolution < space_discretization_.getEnvironmentResolution(true)) {
		space_discretization_.setEnvironmentResolution(grid_resolution, true);
		space_discretization_.setStateResolution(grid_resolution);
	}

	is_added_search_area_ = true;
}


void RewardMap::setNeighboringArea(int back_neighbors, int front_neighbors,
								   int left_neighbors, int right_neighbors,
								   int bottom_neighbors, int top_neighbors)
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


void RewardMap::setResolution(double resolution,
							  bool plane)
{
	space_discretization_.setEnvironmentResolution(resolution, plane);
}


const std::map<Vertex,RewardCell>& RewardMap::getRewardMap() const
{
	return terrain_rewardmap_;
}

} //@namepace dwl
} //@namespace environment
