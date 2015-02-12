#include <environment/LegCollisionFeature.h>


namespace dwl
{

namespace environment
{

LegCollisionFeature::LegCollisionFeature(double clearance, double collision) :
		potential_clearance_(clearance), potential_collision_(collision)
{
	name_ = "Leg Collision";
}


LegCollisionFeature::~LegCollisionFeature()
{

}


void LegCollisionFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Setting the resolution of the terrain
	space_discretization_.setEnvironmentResolution(info.resolution, true);

	// Getting the current foothold position
	Eigen::Vector2d position = info.pose.position;
	double yaw = info.pose.orientation;
	Eigen::Vector3d foothold = info.potential_contact.position;
	int leg = info.potential_contact.end_effector;

	// Getting the leg workspace
	SearchArea leg_workspace = robot_->getPredefinedLegWorkspaces()[leg];

	Eigen::Vector2d boundary_min, boundary_max;
	boundary_min(0) = foothold(0) + leg_workspace.min_x;
	boundary_min(1) = foothold(1) + leg_workspace.min_y;
	boundary_max(0) = foothold(0) + leg_workspace.max_x;
	boundary_max(1) = foothold(1) + leg_workspace.max_y;

	// Computing the maximum and minimum height around the leg area
	double max_height = -std::numeric_limits<double>::max();
	bool is_there_height_values = false;
	for (double y = boundary_min(1); y <= boundary_max(1); y += leg_workspace.resolution) {
		for (double x = boundary_min(0); x <= boundary_max(0); x += leg_workspace.resolution) {
			Eigen::Vector2d coord;
			coord(0) = (x - position(0)) * cos(yaw) - (y - position(1)) * sin(yaw) + position(0);
			coord(1) = (x - position(0)) * sin(yaw) + (y - position(1)) * cos(yaw) + position(1);
			Vertex vertex_2d;
			space_discretization_.coordToVertex(vertex_2d, coord);

			double height;
			if (info.height_map.count(vertex_2d) > 0) {
				height = info.height_map.find(vertex_2d)->second;

				// Updating the maximum height
				if (height > max_height)
					max_height = height;

				is_there_height_values = true;
			}
		}
	}

	if (is_there_height_values) {
		double max_diff_height = max_height - foothold(2);
		if (max_diff_height > 0.0) {
			if (max_diff_height < potential_clearance_)
				reward_value = 0.0;
			else if (max_diff_height < potential_collision_) {
				reward_value = log(0.75 * (1 - (max_diff_height - potential_clearance_) /
						(potential_collision_ - potential_clearance_)));
				if (min_reward_ > reward_value)
					reward_value = min_reward_;
			} else
				reward_value = min_reward_;
		} else
			reward_value = 0;
	} else
		reward_value = min_reward_;
}

} //@namespace environment
} //@namespace dwl
