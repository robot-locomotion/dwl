#include <environment/SupportTriangleFeature.h>


namespace dwl
{

namespace environment
{

SupportTriangleFeature::SupportTriangleFeature() : stable_inradii_(0.34), unstable_inradii_(0.1)
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
		if (i != next_leg) {
			leg_position = potential_footholds[i].position;
			future_stance.push_back(leg_position);
		}
	}

	// Computing the radius of a circle inscribed in the support triangle
	double inradii;
	double size_a = future_stance[0].transpose() * future_stance[1];
	double size_b = future_stance[0].transpose() * future_stance[2];
	double size_c = future_stance[1].transpose() * future_stance[2];

	utils::Math math;
	math.inRadiiTriangle(inradii, size_a, size_b, size_c);

	if (inradii >= stable_inradii_)
		reward_value = 0;
	else if (inradii > unstable_inradii_) {
		reward_value = log(0.75 * (1 - inradii / (stable_inradii_ - unstable_inradii_)));
		if (min_reward_ > reward_value)
			reward_value = min_reward_;
	} else
		reward_value = min_reward_;
}

} //@namespace environment
} //@namespace dwl
