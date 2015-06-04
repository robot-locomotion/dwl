#include <environment/BodyOrientationFeature.h>


namespace dwl
{

namespace environment
{

BodyOrientationFeature::BodyOrientationFeature(double max_roll, double max_pitch) :
		flat_orientation_(1.0 * (M_PI / 180.0)), max_roll_(max_roll * (M_PI / 180.0)),
		max_pitch_(max_pitch * (M_PI / 180.0))
{
	name_ = "Body Orientation";
}


BodyOrientationFeature::~BodyOrientationFeature()
{

}


void BodyOrientationFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Setting the resolution of the terrain
	space_discretization_.setEnvironmentResolution(info.resolution, true);

	// Getting the potential foothold
	std::vector<Contact> potential_footholds = info.current_contacts;
	potential_footholds.push_back(info.potential_contact);

	// Computing the potential stance
	std::vector<Eigen::Vector3f> stance;
	Eigen::Vector3f leg_position;
	int num_footholds = potential_footholds.size();
	for (int i = 0; i < num_footholds; i++) {
		float foothold_x = potential_footholds[i].position(0);
		float foothold_y = potential_footholds[i].position(1);
		float foothold_z = potential_footholds[i].position(2);

		leg_position << foothold_x, foothold_y, foothold_z;
		stance.push_back(leg_position);
	}

	// Computing the plane parameters
	Eigen::Vector3d normal;
	math::computePlaneParameters(normal, stance);

	// Computing the roll and pitch angles
	Eigen::Quaterniond normal_quaternion;
	Eigen::Vector3d origin;
	origin << 0, 0, 1;
	normal_quaternion.setFromTwoVectors(origin, normal);

	// Computing the reward value
	double roll_reward, pitch_reward;
	Eigen::Vector3d rpy = math::getRPY(normal_quaternion);
	double roll = fabs(math::getRoll(rpy));
	double pitch = fabs(math::getPitch(rpy));
	if (roll <= flat_orientation_)
		roll_reward = 0;
	else if (roll < max_roll_)
	{
		roll_reward = log(0.75 * (1 - roll / (max_roll_ - flat_orientation_)));
		if (min_reward_ > roll_reward)
			roll_reward = min_reward_;
	} else
		roll_reward = min_reward_;

	if (pitch <= flat_orientation_)
		pitch_reward = 0;
	else if (pitch < max_pitch_)
	{
		pitch_reward = log(0.75 * (1 - pitch / (max_pitch_ - flat_orientation_)));
		if (min_reward_ > roll_reward)
			pitch_reward = min_reward_;
	} else
		pitch_reward = min_reward_;

	reward_value = roll_reward + pitch_reward;
}

} //@namespace environment
} //@namespace dwl
