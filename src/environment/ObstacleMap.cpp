#include <environment/ObstacleMap.h>


namespace dwl
{

namespace environment
{

ObstacleMap::ObstacleMap() : depth_(16), is_added_search_area_(false)
{

}


ObstacleMap::~ObstacleMap()
{

}

void ObstacleMap::compute(octomap::OcTree* octomap, Eigen::Vector4d robot_state)
{
	if (!is_added_search_area_) {
		printf(YELLOW "Warning: adding a default search area" COLOR_RESET);
		// Adding a default search area
		addSearchArea(0.5, 3.0, -0.75, 0.75, -0.77, -0.4, 0.04);//TODO

		is_added_search_area_ = true;
	}

	double yaw = robot_state(3);

	// Computing obstacle map for several search areas
	for (int n = 0; n < search_areas_.size(); n++) {
		// Computing the boundary of the gridmap
		Eigen::Vector2d boundary_min, boundary_max;

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
				if (!octomap->coordToKeyChecked(xr, yr, z, depth_, init_key)) {
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
							space_discretization_.coordToKeyChecked(cell_key, cell_position);

							Vertex vertex_id;
							space_discretization_.keyToVertex(vertex_id, cell_key, true);




							if (is_first_computation_)
								addCellToTerrainHeightMap(vertex_id, (double) cell_position(2));
							else {
								bool new_status = true;
								if ((reward_gridmap_.find(vertex_id)->first == vertex_id)) {
									// Evaluating if it changed status (height)
									Cell reward_cell = reward_gridmap_.find(vertex_id)->second;
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
	std::map<Vertex, double> terrain_heightmap = terrain_heightmap_;
	for (std::map<Vertex, double>::iterator terrain_iter = terrain_heightmap.begin();
			terrain_iter != terrain_heightmap.end();
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
		heightmap_key = octomap->coordToKey(terrain_point);

		if (is_first_computation_)
			computeRewards(octomap, heightmap_key);
		else {
			bool new_status = true;
			if (reward_gridmap_.find(vertex_id)->first == vertex_id) {
				// Evaluating if it's changed status (height)
				Cell reward_cell = reward_gridmap_.find(vertex_id)->second;

				if (reward_cell.key.z != heightmap_key[2]) {
					removeCellToRewardMap(vertex_id);
					removeCellToTerrainHeightMap(vertex_id);
				} else
					new_status = false;
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


} //@namespace environment
} //@namespace dwl
