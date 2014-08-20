#include <robot/Robot.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : number_legs_(4), stance_size_(0.1)
{
	// Defining the stance position per leg
	stance_position_.resize(number_legs_);
	stance_position_[LF] << 0.4269, 0.2683;
	stance_position_[RF] << 0.4269, -0.2683;
	stance_position_[LH] << -0.4269, 0.2683;
	stance_position_[RH] << -0.4269, -0.2683;

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
		stance_area.max_x = stance_position_[i](0) + stance_size_;
		stance_area.min_x = stance_position_[i](0) - stance_size_;
		stance_area.max_y = stance_position_[i](1) + stance_size_;
		stance_area.min_y = stance_position_[i](1) - stance_size_;
		stance_areas_.push_back(stance_area);
	}

	// Defining the body area of HyQ
	body_area_.max_x = stance_position_[LF](0);
	body_area_.min_x = stance_position_[LH](0);
	body_area_.max_y = stance_position_[LF](1);
	body_area_.min_y = stance_position_[RF](1);

	// Defining the leg areas
	double leg_workspace = 0.25;
	leg_area_.resize(number_legs_);
	for (int leg_id = 0; leg_id < number_legs_; leg_id++) {
		if ((leg_id == LF) || (leg_id == RF)) {
			leg_area_[leg_id].min_x = stance_position_[leg_id](0) - leg_workspace;
			leg_area_[leg_id].max_x = stance_position_[leg_id](0) + 0;
			leg_area_[leg_id].min_y = stance_position_[leg_id](1) - stance_size_;
			leg_area_[leg_id].max_y = stance_position_[leg_id](1) + stance_size_;
		} else {
			leg_area_[leg_id].min_x = stance_position_[leg_id](0) - 0;
			leg_area_[leg_id].max_x = stance_position_[leg_id](0) + leg_workspace;
			leg_area_[leg_id].min_y = stance_position_[leg_id](1) - stance_size_;
			leg_area_[leg_id].max_y = stance_position_[leg_id](1) + stance_size_;
		}
		leg_area_[leg_id].grid_resolution = 0.04;
	}
}


Robot::~Robot()
{

}


void Robot::setPose(Pose pose)
{
	current_pose_ = pose;
}

void Robot::setStanceAreas(std::vector<SearchArea> stance_areas)
{
	stance_areas_ = stance_areas;
}


const Pose& Robot::getPose() const
{
	return current_pose_;
}


const Area& Robot::getBodyArea() const
{
	return body_area_;
}


double Robot::getEstimatedGround()
{
	double stance_height = 0.60;
	double estimated_ground = current_pose_.position(2) - stance_height;

	return estimated_ground;
}

const std::vector<SearchArea>& Robot::getStanceAreas() const
{
	return stance_areas_;
}


const SearchArea& Robot::getLegArea(int leg_id) const
{
	return leg_area_[leg_id];
}


const std::vector<Eigen::Vector2d>& Robot::getStancePosition() const
{
	return stance_position_;
}


int Robot::getNextLeg(int sequence) const
{
	return pattern_locomotion_[sequence];
}


double Robot::getNumberOfLegs() const
{
	return number_legs_;
}

} //@namespace robot
} //@namespace dwl
