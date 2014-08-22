#include <environment/PotentialBodyOrientationFeature.h>


namespace dwl
{

namespace environment
{

PotentialBodyOrientationFeature::PotentialBodyOrientationFeature() : flat_threshold_(0.0 * (M_PI / 180.0)), roll_threshold_(30.0 * (M_PI / 180.0)),
		pitch_threshold_(30.0 * (M_PI / 180.0))
{
	name_ = "Potential Body Orientation";
}


PotentialBodyOrientationFeature::~PotentialBodyOrientationFeature()
{

}


void PotentialBodyOrientationFeature::computeReward(double& reward_value, RobotAndTerrain info) //TODO Finish this feature
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

	for (int leg = 0; leg < robot_->getNumberOfLegs(); leg++) {
		// Getting the leg area
		SearchArea leg_area = robot_->getStanceAreas()[leg];

		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = leg_area.min_x + position(0);
		boundary_min(1) = leg_area.min_y + position(1);
		boundary_max(0) = leg_area.max_x + position(0);
		boundary_max(1) = leg_area.max_y + position(1);

		// Computing the maximum and minimun height around the leg area
		int counter = 0;
		bool is_there_height_values = false;
		for (double y = boundary_min(1); y < boundary_max(1); y += leg_area.grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += leg_area.grid_resolution) {
				Eigen::Vector2d foothold;
				foothold(0) = (x - position(0)) * cos(yaw) - (y - position(1)) * sin(yaw) + position(0);
				foothold(1) = (x - position(0)) * sin(yaw) + (y - position(1)) * cos(yaw) + position(1);

				// Computing the vertex of the foothold
				Vertex leg_vertex;
				space_discretization_.coordToVertex(leg_vertex, foothold);

				// Computing the height foothold
				if (info.height_map.find(leg_vertex)->first == leg_vertex)
					height_foothold = info.height_map.find(leg_vertex)->second;
				else
					height_foothold = robot_->getExpectedGround(leg);

				float foothold_x = foothold(0);
				float foothold_y = foothold(1);
				float foothold_z = height_foothold;
				leg_position << foothold_x, foothold_y, foothold_z;
				stance.push_back(leg_position);
			}
		}
	}

	// Computing the plane parameters
	utils::Math math;
	Eigen::Vector3d normal;
	math.computePlaneParameters(normal, stance);

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
	if (r < flat_threshold_)
		roll_reward = 0.0;
	else {
		if (r < roll_threshold_) {
			roll_reward = log((roll_threshold_ - r) / (roll_threshold_ - flat_threshold_));
			if (max_reward_ > roll_reward)
				roll_reward = max_reward_;
		} else
			roll_reward = max_reward_;
	}

	if (p < flat_threshold_)
		pitch_reward = 0.0;
	else {
		if (p < pitch_threshold_) {
			pitch_reward = log(fabs((pitch_threshold_ - p) / (pitch_threshold_ - flat_threshold_)));
			if (max_reward_ > roll_reward)
				pitch_reward = max_reward_;
		} else
			pitch_reward = max_reward_;
	}

	reward_value = roll_reward + pitch_reward;
}

} //@namespace environment
} //@namespace dwl
