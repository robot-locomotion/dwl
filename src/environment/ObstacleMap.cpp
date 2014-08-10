#include <environment/ObstacleMap.h>


namespace dwl
{

namespace environment
{

ObstacleMap::ObstacleMap() : space_discretization_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
		depth_(16), is_added_search_area_(false), interest_radius_x_(std::numeric_limits<double>::max()),
		interest_radius_y_(std::numeric_limits<double>::max())
{
	depth_ = 14; //TODO
}


ObstacleMap::~ObstacleMap()
{

}

void ObstacleMap::compute(octomap::OcTree* octomap, Eigen::Vector4d robot_state)
{
	if (!is_added_search_area_) {
		printf(YELLOW "Warning: adding a default search area" COLOR_RESET);
		// Adding a default search area
		addSearchArea(0.5, 3.0, -0.75, 0.75, -0.202, 0.2, 0.04);//TODO

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
					printf(RED "Cell out of bounds \n" COLOR_RESET);

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
							// Setting the obstacle cell sizes
							Cell obstacle_cell;
							obstacle_cell.plane_size = space_discretization_.getEnvironmentResolution(true);
							obstacle_cell.height_size = space_discretization_.getEnvironmentResolution(false);

							// Getting position of the occupied cell
							Eigen::Vector3d cell_position;
							cell_position(0) = height_point(0);
							cell_position(1) = height_point(1);
							cell_position(2) = height_point(2);
							space_discretization_.coordToKeyChecked(obstacle_cell.key, cell_position);

							// Adding the obstacle to map
							addCellToObstacleMap(obstacle_cell);

							break;
						}
					}
					r++;
				}
			}
		}
	}


	// Removing the points that doesn't belong to interest area
	Eigen::Vector3d robot_2dpose;
	robot_2dpose(0) = robot_state(0);
	robot_2dpose(1) = robot_state(1);
	robot_2dpose(2) = robot_state(3);
	removeObstacleOutsideInterestRegion(robot_2dpose);
}


void ObstacleMap::addSearchArea(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, double grid_resolution)
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

	if (grid_resolution < space_discretization_.getEnvironmentResolution(true))
		space_discretization_.setEnvironmentResolution(grid_resolution, true);

	is_added_search_area_ = true;
}


void ObstacleMap::removeObstacleOutsideInterestRegion(Eigen::Vector3d robot_state)
{
	// Getting the orientation of the body
	double yaw = robot_state(2);

	for (std::map<Vertex,Cell>::iterator vertex_iter = obstacle_gridmap_.begin();
			vertex_iter != obstacle_gridmap_.end();
			vertex_iter++)
	{
		Vertex v = vertex_iter->first;
		Eigen::Vector2d point;
		space_discretization_.vertexToCoord(point, v);

		double xc = point(0) - robot_state(0);
		double yc = point(1) - robot_state(1);
		if (xc * cos(yaw) + yc * sin(yaw) >= 0.0) {
			if (pow(xc * cos(yaw) + yc * sin(yaw), 2) / pow(interest_radius_y_, 2) + pow(xc * sin(yaw) - yc * cos(yaw), 2) / pow(interest_radius_x_, 2) > 1)
				obstacle_gridmap_.erase(v);
		} else {
			if (pow(xc, 2) + pow(yc, 2) > pow(interest_radius_x_, 2))
				obstacle_gridmap_.erase(v);
		}
	}
}


void ObstacleMap::addCellToObstacleMap(Cell cell)
{
	Vertex vertex_id;
	space_discretization_.keyToVertex(vertex_id, cell.key, true);
	obstacle_gridmap_[vertex_id] = cell;
}


void ObstacleMap::setInterestRegion(double radius_x, double radius_y)
{
	interest_radius_x_ = radius_x;
	interest_radius_y_ = radius_y;
}


double ObstacleMap::getResolution(bool plane)
{
	return space_discretization_.getEnvironmentResolution(plane);
}


void ObstacleMap::setResolution(double resolution, bool plane)
{
	space_discretization_.setEnvironmentResolution(resolution, plane);
}


const std::map<Vertex,Cell>& ObstacleMap::getObstacleMap() const
{
	return obstacle_gridmap_;
}

} //@namespace environment
} //@namespace dwl
