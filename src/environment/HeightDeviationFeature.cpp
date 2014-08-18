#include <environment/HeightDeviationFeature.h>


namespace dwl
{

namespace environment
{

HeightDeviationFeature::HeightDeviationFeature() : space_discretization_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
{
	name_ = "Height Deviation";
	gain_ = 50;
}


HeightDeviationFeature::~HeightDeviationFeature()
{

}


void HeightDeviationFeature::computeReward(double& reward_value, Terrain terrain_info)
{
	// Setting the grid resolution of the gridmap
	space_discretization_.setEnvironmentResolution(terrain_info.resolution, true);

	// Getting the cell position
	Eigen::Vector3d cell_position = terrain_info.position;

	// Computing the average height of the neighboring area
	double height_average = 0, height_deviation = 0;
	int counter = 0, num_unknown = 0;

	Eigen::Vector2d boundary_min, boundary_max;
	boundary_min(0) = neightboring_area_.min_x + cell_position(0);
	boundary_min(1) = neightboring_area_.min_y + cell_position(1);
	boundary_max(0) = neightboring_area_.max_x + cell_position(0);
	boundary_max(1) = neightboring_area_.max_y + cell_position(1);
	for (double y = boundary_min(1); y < boundary_max(1); y += neightboring_area_.grid_resolution) {
		for (double x = boundary_min(0); x < boundary_max(0); x += neightboring_area_.grid_resolution) {
			Eigen::Vector2d coord;
			coord(0) = x;
			coord(1) = y;
			Vertex vertex_2d;
			space_discretization_.coordToVertex(vertex_2d, coord);

			if (terrain_info.height_map.find(vertex_2d)->first == vertex_2d) {
				height_average += terrain_info.height_map.find(vertex_2d)->second;
				counter++;
			} else
				num_unknown++;
		}
	}
	if (counter != 0) {
		height_average /= counter;

		// Computing the standard deviation of the height
		for (double y = boundary_min(1); y < boundary_max(1); y += neightboring_area_.grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += neightboring_area_.grid_resolution) {
				Eigen::Vector2d coord;
				coord(0) = x;
				coord(1) = y;
				Vertex vertex_2d;
				space_discretization_.coordToVertex(vertex_2d, coord);

				if (terrain_info.height_map.find(vertex_2d)->first == vertex_2d) {
					height_deviation += fabs(terrain_info.height_map.find(vertex_2d)->second - height_average);
				}
			}
		}

		reward_value = -height_deviation / counter;

		double estimated_ground = 0; //TODO Implement a method that computes a estimated ground
		if (num_unknown != 0) {
			double unknown_deviation = 0;
			for (int i = 0; i < num_unknown; i++)
				unknown_deviation += fabs(estimated_ground - (double) cell_position(2));

			reward_value -= unknown_deviation / num_unknown;
		}
	} else
		reward_value = 0;

	reward_value *= gain_; /* heuristic value */
}

} //@namespace environment
} //@namespace dwl
