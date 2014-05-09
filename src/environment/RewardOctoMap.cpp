#include <environment/RewardOctoMap.h>


namespace dwl
{

namespace environment
{


RewardOctoMap::RewardOctoMap() : is_first_computation_(true)
{
	// Default neighboring area
	setNeighboringArea(-1, 1, -1, 1, -1, 1);
}


RewardOctoMap::~RewardOctoMap()
{

}


void RewardOctoMap::compute(Modeler model, Eigen::Vector2d robot_position)
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

	// Computing reward map for several search areas
	for (int n = 0; n < search_areas_.size(); n++) {
		// Computing the boundaring of the gripmap
		Eigen::Vector2d coord;
		Key min_grid, max_grid;
		coord(0) = search_areas_[n].min_x + robot_position(0);
		coord(1) = search_areas_[n].min_y + robot_position(1);
		gridmap_.coordToKeyChecked(coord, min_grid);
		coord(0) = search_areas_[n].max_x + robot_position(0);
		coord(1) = search_areas_[n].max_y + robot_position(1);
		gridmap_.coordToKeyChecked(coord, max_grid);

		int search_step = round(search_areas_[n].grid_resolution / gridmap_.getResolution());
		if (search_step == 0)
			search_step = 1;

		for (unsigned short int j = min_grid.key[1]; j < max_grid.key[1]; j += search_step) {
			double y = gridmap_.keyToCoord(j);
			for (unsigned short int i = min_grid.key[0]; i < max_grid.key[0]; i += search_step) {
				double x = gridmap_.keyToCoord(i);

				std::cout << "key = " << i << " " << j << std::endl;

				// Checking if the voxel belongs to dimensions of the map, and also getting the key of this voxel
				double z = search_areas_[n].max_z;
				octomap::OcTreeKey init_key;
				if (!octomap->coordToKeyChecked(x, y, z, 16, init_key)) {
					printf(RED "Voxel out of bounds\n" COLOR_RESET);

					return;
				}

				// Finding the voxel of the surface
				int r = 0;
				while (z >= search_areas_[n].min_z) {
					octomap::OcTreeKey heightmap_key;
					octomap::OcTreeNode* heightmap_node = octomap->search(init_key);
					heightmap_key[0] = init_key[0];
					heightmap_key[1] = init_key[1];
					heightmap_key[2] = init_key[2] - r;

					heightmap_node = octomap->search(heightmap_key);
					octomap::point3d height_point = octomap->keyToCoord(heightmap_key);
					z = height_point(2);

					if (heightmap_node) {
						// Computation of the features if it is found the surface voxel
						if (octomap->isNodeOccupied(heightmap_node)) {
							if (is_first_computation_) {
								if (computeFeaturesAndRewards(octomap, heightmap_key))
									occupied_voxels_.push_back(heightmap_key);
							} else {
								bool new_status = true;
								for (int k = 0; k < occupied_voxels_.size(); k++) {
									if (occupied_voxels_[k] == heightmap_key) {
										new_status = false;

										break;
									}
								}
								if (new_status) {
									if (computeFeaturesAndRewards(octomap, heightmap_key))
										occupied_voxels_.push_back(heightmap_key);
								}
							}
							break;
						}
					}
					r++;
				}
			}
		}

		is_first_computation_ = false;
	}
}


bool RewardOctoMap::computeFeaturesAndRewards(octomap::OcTree* octomap, octomap::OcTreeKey heightmap_key)
{
	std::vector<Eigen::Vector3f> neighbors_position;
	octomap::OcTreeNode* heightmap_node = octomap->search(heightmap_key);

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

	math_.solvePlaneParameters(covariance_matrix, terrain_info.surface_normal, terrain_info.curvature);

	// Computing the reward
	if (is_added_feature_) {
		double reward_value;
		for (int i = 0; i < features_.size(); i++) {
			features_[i]->computeReward(reward_value, terrain_info);
			//std::cout << features_[i]->getName() << " value = " << reward_value << std::endl;
		}
		addCellToRewardMap(reward_value, terrain_info);
		std::cout << "grid size = " << reward_gridmap_.size() << std::endl;
	}
	else {
		printf(YELLOW "Could not compute the reward of the features because it is necessary to add at least one\n" COLOR_RESET);
		return false;
	}



	//------------------------ record data
	Eigen::Vector3d origin_vector = Eigen::Vector3d::Zero();
	origin_vector(0) = 1;
	Eigen::Quaternion<double> orientation;
	orientation.setFromTwoVectors(origin_vector, terrain_info.surface_normal);

	Pose pose;
	pose.position = terrain_info.position;
	pose.orientation(0) = orientation.x();
	pose.orientation(1) = orientation.y();
	pose.orientation(2) = orientation.z();
	pose.orientation(3) = orientation.w();
	normals_.push_back(pose);


	return true;
}


std::vector<Pose> RewardOctoMap::getNormals()
{
	return normals_;
}


} //@namespace environment

} //@namespace dwl
