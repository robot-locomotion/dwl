#include <environment/PotentialBodyOrientationFeature.h>


namespace dwl
{

namespace environment
{

PotentialBodyOrientationFeature::PotentialBodyOrientationFeature(double max_roll, double max_pitch) :
		flat_orientation_(1.0 * (M_PI / 180.0)), max_roll_(max_roll * (M_PI / 180.0)),
		max_pitch_(max_pitch * (M_PI / 180.0))
{
	name_ = "Potential Body Orientation";
}


PotentialBodyOrientationFeature::~PotentialBodyOrientationFeature()
{

}


void PotentialBodyOrientationFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Setting the resolution of the terrain
	space_discretization_.setEnvironmentResolution(info.resolution, true);

	// Computing the height position of every leg
	double height_foothold;
	std::vector<Eigen::Vector3f> stance;
	Eigen::Vector3f leg_position;

	// Getting the current pose
	Eigen::Vector2d position = info.pose.position;
	double yaw = info.pose.orientation;

	// Getting the leg workspaces
	SearchAreaMap leg_search_areas = robot_->getFootstepSearchAreas();

	EndEffectorMap leg_map = robot_->getLegMap();
	for (EndEffectorMap::iterator it = leg_map.begin(); it != leg_map.end(); ++it) {
		unsigned int leg_id = it->first;

		// Getting the leg workspace
		SearchArea leg_search_area = leg_search_areas[leg_id];

		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = leg_search_area.min_x + position(0);
		boundary_min(1) = leg_search_area.min_y + position(1);
		boundary_max(0) = leg_search_area.max_x + position(0);
		boundary_max(1) = leg_search_area.max_y + position(1);

		// Computing the maximum and minimum height around the leg workspace
		for (double y = boundary_min(1); y <= boundary_max(1); y += leg_search_area.resolution) {
			for (double x = boundary_min(0); x <= boundary_max(0); x += leg_search_area.resolution) {
				Eigen::Vector2d foothold;
				foothold(0) = (x - position(0)) * cos(yaw) - (y - position(1)) * sin(yaw) + position(0);
				foothold(1) = (x - position(0)) * sin(yaw) + (y - position(1)) * cos(yaw) + position(1);

				// Computing the vertex of the foothold
				Vertex foothold_vertex;
				space_discretization_.coordToVertex(foothold_vertex, foothold);

				// Computing the height foothold
				if (info.height_map.count(foothold_vertex) > 0)
					height_foothold = info.height_map.find(foothold_vertex)->second;
				else
					height_foothold = robot_->getExpectedGround(leg_id);

				float foothold_x = foothold(0);
				float foothold_y = foothold(1);
				float foothold_z = height_foothold;
				leg_position << foothold_x, foothold_y, foothold_z;

				stance.push_back(leg_position);
			}
		}
	}

	// Computing the plane parameters
	Eigen::Vector3d normal;
	math::computePlaneParameters(normal, stance);

	// Computing the roll and pitch angles
	Eigen::Quaterniond normal_quaternion;
	Eigen::Vector3d origin;
	origin << 0, 0, 1;
	normal_quaternion.setFromTwoVectors(origin, normal);

	double r, p, y;
	Orientation orientation(normal_quaternion);
	orientation.getRPY(r, p, y);

	// Computing the reward value
	double roll_reward, pitch_reward;
	r = fabs(r);
	p = fabs(p);
	if (r < flat_orientation_)
		roll_reward = 0.0;
	else if (r < max_roll_) {
		roll_reward = log(0.75 * (1 - (r - flat_orientation_) / (max_roll_ - flat_orientation_)));
		if (min_reward_ > roll_reward)
			roll_reward = min_reward_;
	} else
		roll_reward = min_reward_;

	if (p < flat_orientation_)
		pitch_reward = 0.0;
	else if (p < max_pitch_) {
		pitch_reward = log(0.75 * (1 - (p - flat_orientation_) / (max_pitch_ - flat_orientation_)));
		if (min_reward_ > roll_reward)
			pitch_reward = min_reward_;
	} else
		pitch_reward = min_reward_;

	reward_value = roll_reward + pitch_reward;
}

} //@namespace environment
} //@namespace dwl
