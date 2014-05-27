#include <environment/RewardOctoMap.h>


namespace dwl
{

namespace environment
{


RewardOctoMap::RewardOctoMap() : is_first_computation_(true), using_cloud_mean_(false)
{
	// Default neighboring area
	setNeighboringArea(-1, 1, -1, 1, -1, 1);
}


RewardOctoMap::~RewardOctoMap()
{

}


void RewardOctoMap::compute(Modeler model, Eigen::Vector4d robot_state)
{
	octomap::OcTree* octomap = model.octomap;

	// To ensure memory reallocation
//	std::vector<Pose> empty;
//	normals_.swap(empty);

	if (!is_added_search_area_) {
		printf(YELLOW "Warning: adding a default search area" COLOR_RESET);
		// Adding a default search area
		addSearchArea(0.5, 3.0, -0.75, 0.75, -0.77, -0.4, 0.04);

		is_added_search_area_ = true;
	}

	double yaw = robot_state(3);//45 * 3.1416 /180; //TODO Elimites

	// Computing reward map for several search areas
	for (int n = 0; n < search_areas_.size(); n++) {
		// Computing the boundary of the gridmap
		Eigen::Vector2d boundary_min, boundary_max;
		Key min_grid, max_grid;
		boundary_min(0) = search_areas_[n].min_x + robot_state(0);
		boundary_min(1) = search_areas_[n].min_y + robot_state(1);
		boundary_max(0) = search_areas_[n].max_x + robot_state(0);
		boundary_max(1) = search_areas_[n].max_y + robot_state(1);

		for (double y = boundary_min(1); y < boundary_max(1); y += search_areas_[n].grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += search_areas_[n].grid_resolution) {
				// Computing the rotated coordinate of the point inside the search area
				double xr = (x - robot_state(0)) * cos(yaw) - (y - robot_state(1)) * sin(yaw) + robot_state(0);
				double yr = (x - robot_state(0)) * sin(yaw) + (y - robot_state(1)) * cos(yaw) + robot_state(1);

				// Checking if the cell belongs to dimensions of the map, and also getting the key of this cell
				double z = search_areas_[n].max_z + robot_state(2);
				octomap::OcTreeKey init_key;
				if (!octomap->coordToKeyChecked(xr, yr, z, 16, init_key)) {
					printf(RED "Cell out of bounds\n" COLOR_RESET);

					return;
				}

				// Finding the cell of the surface
				int r = 0;
				while (z >= search_areas_[n].min_z + robot_state(2)) {
					octomap::OcTreeKey heightmap_key;
					octomap::OcTreeNode* heightmap_node = octomap->search(init_key);
					heightmap_key[0] = init_key[0];
					heightmap_key[1] = init_key[1];
					heightmap_key[2] = init_key[2] - r;

					heightmap_node = octomap->search(heightmap_key);
					octomap::point3d height_point = octomap->keyToCoord(heightmap_key);
					z = height_point(2);
					if (heightmap_node) {
						// Computation of the features if it is found the surface cell
						if (octomap->isNodeOccupied(heightmap_node)) {
							CellKey cell_key;
							// Getting position of the occupied cell
							Eigen::Vector3d cell_position;
							cell_position(0) = height_point(0);
							cell_position(1) = height_point(1);
							cell_position(2) = height_point(2);
							getCell(cell_key, cell_position);

							if (is_first_computation_)
								computeFeaturesAndRewards(octomap, heightmap_key);
							else {
								bool new_status = true;
								for (int k = 0; k < reward_gridmap_.size(); k++) {//TODO improve the search around the percepcion region. possible?
									if (reward_gridmap_[k].cell_key.grid_id.key[0] == cell_key.grid_id.key[0] &&
											reward_gridmap_[k].cell_key.grid_id.key[1] == cell_key.grid_id.key[1]) {

										CellKey occupied_cell_key = reward_gridmap_[k].cell_key;
										if (reward_gridmap_[k].cell_key.height_id != cell_key.height_id)
											removeCellToRewardMap(occupied_cell_key);

										new_status = false;
										break;
									}
								}
								if (new_status)
									computeFeaturesAndRewards(octomap, heightmap_key);
							}
							break;
						}
					}
					r++;
				}
			}
		}
	}
	is_first_computation_ = false;

	// Removing the points that doesn't belong to interest area
	Eigen::Vector3d robot_2dpose;
	robot_2dpose(0) = robot_state(0);
	robot_2dpose(1) = robot_state(1);
	robot_2dpose(2) = robot_state(3);
	removeRewardOutsideInterestRegion(robot_2dpose);
}


bool RewardOctoMap::computeFeaturesAndRewards(octomap::OcTree* octomap, octomap::OcTreeKey heightmap_key)
{
	std::vector<Eigen::Vector3f> neighbors_position;
	octomap::OcTreeNode* heightmap_node = octomap->search(heightmap_key);

	// Adding to the cloud the point of interest
	Eigen::Vector3f heightmap_position;
	octomap::point3d heightmap_point = octomap->keyToCoord(heightmap_key);
	heightmap_position(0) = heightmap_point(0);
	heightmap_position(1) = heightmap_point(1);
	heightmap_position(2) = heightmap_point(2);
	neighbors_position.push_back(heightmap_position);

	// Iterate over the 8 neighboring sets
	octomap::OcTreeKey neighbor_key;
	octomap::OcTreeNode* neighbor_node = heightmap_node;
	bool is_there_neighboring = false;
	for (int i = neighboring_area_.min_z; i < neighboring_area_.max_z + 1; i++) {
		for (int j = neighboring_area_.min_y; j < neighboring_area_.max_y + 1; j++) {
			for (int k = neighboring_area_.min_x; k < neighboring_area_.max_x + 1; k++) {
				neighbor_key[0] = heightmap_key[0] + k;
				neighbor_key[1] = heightmap_key[1] + j;
				neighbor_key[2] = heightmap_key[2] + i;
				neighbor_node = octomap->search(neighbor_key);

				if (neighbor_node) {
					Eigen::Vector3f neighbor_position;
					octomap::point3d neighbor_point;
					if (octomap->isNodeOccupied(neighbor_node)) {
						neighbor_point = octomap->keyToCoord(neighbor_key);
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
	if (is_there_neighboring)
		return computeRewards(neighbors_position);

	return false;
}


bool RewardOctoMap::computeRewards(std::vector<Eigen::Vector3f> cloud)
{
	// Computing terrain info
	Terrain terrain_info;
	EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
	if (cloud.size() < 3 || math_.computeMeanAndCovarianceMatrix(cloud, covariance_matrix, terrain_info.position) == 0)
		return false;

	if (!using_cloud_mean_) {
		terrain_info.position(0) = cloud[0](0);
		terrain_info.position(1) = cloud[0](1);
		terrain_info.position(2) = cloud[0](2);
	}

	math_.solvePlaneParameters(covariance_matrix, terrain_info.surface_normal, terrain_info.curvature);

	// Computing the reward
	if (is_added_feature_) {
		double reward_value;
		for (int i = 0; i < features_.size(); i++) {
			features_[i]->computeReward(reward_value, terrain_info);
			//std::cout << features_[i]->getName() << " value = " << reward_value << std::endl;
		}

		Cell cell;
		getCell(cell, reward_value, terrain_info);
		addCellToRewardMap(cell);
	}
	else {
		printf(YELLOW "Could not compute the reward of the features because it is necessary to add at least one\n" COLOR_RESET);
		return false;
	}



	//------------------------ record data
/*	Eigen::Vector3d origin_vector = Eigen::Vector3d::Zero();
	origin_vector(0) = 1;
	Eigen::Quaternion<double> orientation;
	orientation.setFromTwoVectors(origin_vector, terrain_info.surface_normal);

	Pose pose;
	pose.position = terrain_info.position;
	pose.orientation(0) = orientation.x();
	pose.orientation(1) = orientation.y();
	pose.orientation(2) = orientation.z();
	pose.orientation(3) = orientation.w();
	normals_.push_back(pose);*/


	return true;
}


std::vector<Pose> RewardOctoMap::getNormals()
{
	return normals_;
}


} //@namespace environment

} //@namespace dwl
