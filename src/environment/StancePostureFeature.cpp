#include <environment/StancePostureFeature.h>


namespace dwl
{

namespace environment
{

StancePostureFeature::StancePostureFeature() : max_distance_(0.0), first_time_(true)
{
	name_ = "Stance Posture";
}


StancePostureFeature::~StancePostureFeature()
{

}


void StancePostureFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Computing the maximum distance
	if (first_time_) {
		SearchArea foothold_region =
				robot_->getFootstepSearchAreas(info.body_action)[info.potential_contact.end_effector];
		double quadrilateral_area = (foothold_region.max_x - foothold_region.min_x) *
				(foothold_region.max_y - foothold_region.min_y);
		double quadrilateral_semiperimeter = (foothold_region.max_x - foothold_region.min_x) +
				(foothold_region.max_x - foothold_region.min_x);
		max_distance_ = quadrilateral_area / quadrilateral_semiperimeter;

		first_time_ = false;
	}

	// Computing the distance from the stance posture
	Eigen::Vector3d stance_foothold = robot_->getStance(info.body_action)[info.potential_contact.end_effector];
	double distance = (info.potential_contact.position - stance_foothold).norm();

	// Computing the reward value
	if (distance > info.resolution) {
		if (distance >= max_distance_)
			reward_value = min_reward_;
		else
			reward_value = log(0.75 * (1 - distance / max_distance_));
	} else
		reward_value = 0;
}

} //@namespace environment
} //@namespace dwl
