#include <environment/SupportTriangleFeature.h>


namespace dwl
{

namespace environment
{

SupportTriangleFeature::SupportTriangleFeature() : stable_inradii_(0.13), unstable_inradii_(0.08)
{
	name_ = "Support Triangle";
}


SupportTriangleFeature::~SupportTriangleFeature()
{

}


void SupportTriangleFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Setting the resolution of the terrain
	space_discretization_.setEnvironmentResolution(info.resolution, true);

	// Getting the potential foothold
	std::vector<Contact> potential_footholds = info.current_contacts;
	potential_footholds.push_back(info.potential_contact);

	// Getting the next leg
	int next_leg = robot_->getPatternOfLocomotion()[info.potential_contact.end_effector];

	// Computing the future stance
	std::vector<Eigen::Vector3d> future_stance;
	Eigen::Vector3d leg_position;

	for (int i = 0; i < potential_footholds.size(); i++) {
		if (potential_footholds[i].end_effector != next_leg) {
			leg_position = potential_footholds[i].position;
			future_stance.push_back(leg_position);
		}
	}

	// Computing the radius of a circle inscribed in the support triangle
	double inradii;
	Eigen::Vector2d foothold_0 = future_stance[0].head(2);
	Eigen::Vector2d foothold_1 = future_stance[1].head(2);
	Eigen::Vector2d foothold_2 = future_stance[2].head(2);
	double size_a = (foothold_1 - foothold_0).transpose() * (foothold_1 - foothold_0);
	double size_b = (foothold_2 - foothold_0).transpose() * (foothold_2 - foothold_0);
	double size_c = (foothold_2 - foothold_1).transpose() * (foothold_2 - foothold_1);

	utils::Math math;
	math.inRadiiTriangle(inradii, size_a, size_b, size_c);

	if (inradii >= stable_inradii_)
		reward_value = 0;
	else if (inradii > unstable_inradii_) {
		reward_value = log(0.75 * (inradii - unstable_inradii_) / (stable_inradii_ - unstable_inradii_));
		if (min_reward_ > reward_value)
			reward_value = min_reward_;
	} else
		reward_value = min_reward_;
}

} //@namespace environment
} //@namespace dwl
