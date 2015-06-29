#include <environment/KinematicFeasibilityFeature.h>


namespace dwl
{

namespace environment
{

KinematicFeasibilityFeature::KinematicFeasibilityFeature(double kin_lim_x, double kin_lim_y,
		double stable_displacement) : kin_lim_x_(kin_lim_x), kin_lim_y_(kin_lim_y),
				stable_displacement_(stable_displacement)
{
	name_ = "Kinematic Feasibility";
}


KinematicFeasibilityFeature::~KinematicFeasibilityFeature()
{

}


void KinematicFeasibilityFeature::computeReward(double& reward_value, RobotAndTerrain info)
{
	// Computing the foot displacement from nominal stance
	Contact foothold = info.potential_contact;
	Eigen::Vector2d robot_position = info.pose.position;
	Eigen::Vector2d nominal_stance = robot_->getNominalStance()[foothold.end_effector].head(2);
	Eigen::Vector2d foothold_from_stance = foothold.position.head(2) - (nominal_stance + robot_position);
	double displacement = foothold_from_stance.norm();

	// Computing the maximum displacement in the direction of the foothold
	Eigen::Vector2d kinematic_limits;
	kinematic_limits << kin_lim_x_, kin_lim_y_;
	double theta = atan2((double) foothold_from_stance(1), (double) foothold_from_stance(0));
	Eigen::Vector2d ellipse;
	ellipse(0) = kinematic_limits(0) * cos(theta);
	ellipse(1) = kinematic_limits(1) * sin(theta);
	double max_allowed_displacement = ellipse.norm();

	// Computing the reward value
	if (displacement >= stable_displacement_)
		reward_value = 0;
	else if (displacement > max_allowed_displacement) {
		reward_value = log(0.75 * (displacement - max_allowed_displacement) /
				(stable_displacement_ - max_allowed_displacement));
		if (min_reward_ > reward_value)
			reward_value = min_reward_;
	} else
		reward_value = min_reward_;
}

} //@namespace environment
} //@namespace dwl
