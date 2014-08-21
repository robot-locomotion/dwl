#include <robot/Robot.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : number_legs_(4), stance_size_(0.1)
{
	// Defining the stance position per leg
	nominal_stance_.resize(number_legs_);
	nominal_stance_[LF] << 0.4269, 0.2683, -0.6116;
	nominal_stance_[RF] << 0.4269, -0.2683, -0.6116;
	nominal_stance_[LH] << -0.4269, 0.2683, -0.6116;
	nominal_stance_[RH] << -0.4269, -0.2683, -0.6116;

	// Defining the pattern of locomotion
	pattern_locomotion_.resize(number_legs_);
	pattern_locomotion_[0] = LH;
	pattern_locomotion_[1] = LF;
	pattern_locomotion_[2] = RH;
	pattern_locomotion_[3] = RF;

	// Defining the search areas for the stance position of HyQ
	SearchArea stance_area;
	stance_area.grid_resolution = 0.04;
	for (int i = 0; i < number_legs_; i++) {
		stance_area.max_x = nominal_stance_[i](0) + stance_size_;
		stance_area.min_x = nominal_stance_[i](0) - stance_size_;
		stance_area.max_y = nominal_stance_[i](1) + stance_size_;
		stance_area.min_y = nominal_stance_[i](1) - stance_size_;
		stance_areas_.push_back(stance_area);
	}

	// Defining the body area of HyQ
	body_area_.max_x = nominal_stance_[LF](0);
	body_area_.min_x = nominal_stance_[LH](0);
	body_area_.max_y = nominal_stance_[LF](1);
	body_area_.min_y = nominal_stance_[RF](1);

	// Defining the leg areas
	double leg_workspace = 0.25;
	leg_area_.resize(number_legs_);
	for (int leg_id = 0; leg_id < number_legs_; leg_id++) {
		if ((leg_id == LF) || (leg_id == RF)) {
			leg_area_[leg_id].min_x = nominal_stance_[leg_id](0) - leg_workspace;
			leg_area_[leg_id].max_x = nominal_stance_[leg_id](0) + stance_size_;
			leg_area_[leg_id].min_y = nominal_stance_[leg_id](1) - stance_size_;
			leg_area_[leg_id].max_y = nominal_stance_[leg_id](1) + stance_size_;
		} else {
			leg_area_[leg_id].min_x = nominal_stance_[leg_id](0) - stance_size_;
			leg_area_[leg_id].max_x = nominal_stance_[leg_id](0) + leg_workspace;
			leg_area_[leg_id].min_y = nominal_stance_[leg_id](1) - stance_size_;
			leg_area_[leg_id].max_y = nominal_stance_[leg_id](1) + stance_size_;
		}
		leg_area_[leg_id].grid_resolution = 0.04;
	}
}


Robot::~Robot()
{

}


void Robot::setCurrentPose(Pose pose)
{
	current_pose_ = pose;
}


void Robot::setStanceAreas(std::vector<SearchArea> stance_areas)
{
	stance_areas_ = stance_areas;
}


void Robot::setPatternOfLocomotion(std::vector<int> pattern)
{
	pattern_locomotion_ = pattern;
}


Pose Robot::getCurrentPose()
{
	return current_pose_;
}


Area Robot::getBodyArea()
{
	return body_area_;
}


std::vector<Eigen::Vector3d> Robot::getNominalStance()
{
	return nominal_stance_;
}


std::vector<int> Robot::getPatternOfLocomotion()
{
	return pattern_locomotion_;
}


std::vector<SearchArea> Robot::getStanceAreas()
{
	return stance_areas_;
}


double Robot::getExpectedGround(int leg_id)
{
	return current_pose_.position(2) + nominal_stance_[leg_id](2);
}


std::vector<SearchArea> Robot::getLegWorkAreas()
{
	return leg_area_;
}


double Robot::getNumberOfLegs()
{
	return number_legs_;
}

} //@namespace robot
} //@namespace dwl
