#include <environment/PotentialLegCollisionFeature.h>


namespace dwl
{

namespace environment
{

PotentialLegCollisionFeature::PotentialLegCollisionFeature() : potential_clearance_(0.04), potential_collision_(0.2)
{
	name_ = "Potential Leg Collision";
}


PotentialLegCollisionFeature::~PotentialLegCollisionFeature()
{

}


void PotentialLegCollisionFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	reward_value = 0;

	// Setting the resolution of the terrain
	space_discretization_.setEnvironmentResolution(info.resolution, true);

	// Getting the current pose
	Eigen::Vector2d position = info.pose.position;
	double yaw = info.pose.orientation;

	int leg_information = 0;
	for (int leg = 0; leg < robot_->getNumberOfLegs(); leg++) {
		// Getting the leg area and nominal stance of the leg
		SearchArea leg_area = robot_->getLegWorkAreas()[leg];
		Eigen::Vector3d nominal_stance = robot_->getNominalStance()[leg];

		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = position(0) + nominal_stance(0) + leg_area.min_x;
		boundary_min(1) = position(1) + nominal_stance(1) + leg_area.min_y;
		boundary_max(0) = position(0) + nominal_stance(0) + leg_area.max_x;
		boundary_max(1) = position(1) + nominal_stance(1) + leg_area.max_y;

		// Computing the maximum and minimun height around the leg area
		double max_height = -std::numeric_limits<double>::max();
		double mean_height = 0;
		int counter = 0;
		bool is_there_height_values = false;
		for (double y = boundary_min(1); y < boundary_max(1); y += leg_area.grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += leg_area.grid_resolution) {
				Eigen::Vector2d coord;
				coord(0) = (x - position(0)) * cos(yaw) - (y - position(1)) * sin(yaw) + position(0);
				coord(1) = (x - position(0)) * sin(yaw) + (y - position(1)) * cos(yaw) + position(1);
				Vertex vertex_2d;
				space_discretization_.coordToVertex(vertex_2d, coord);

				double height;
				if (info.height_map.find(vertex_2d)->first == vertex_2d) {
					height = info.height_map.find(vertex_2d)->second;

					// Updating the maximum height
					if (height > max_height)
						max_height = height;

					mean_height += height;
					counter++;

					is_there_height_values = true;
				}
			}
		}

		if (is_there_height_values) {
			mean_height /= counter;
			double max_diff_height = max_height - mean_height;

			if (max_diff_height < potential_clearance_)
				reward_value += 0.0;
			else if (max_diff_height < potential_collision_) {
				double potential_reward_value = log(0.75 * (1 - (max_diff_height - potential_clearance_) / (potential_collision_ - potential_clearance_)));
				if (min_reward_ > potential_reward_value)
					reward_value += min_reward_;
				else
					reward_value += potential_reward_value;
			} else
				reward_value += min_reward_;
			leg_information++;
		}
	}

	if (leg_information != 0)
		reward_value /= leg_information;
}

} //@namespace environment
} //@namespace dwl
