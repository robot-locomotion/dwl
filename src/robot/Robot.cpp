#include <robot/Robot.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : number_legs_(4)
{
	// Defining the stance position per leg
	stance_position_.resize(number_legs_);
	stance_position_[LF] << 0.4269, 0.3886;
	stance_position_[RF] << 0.4269, -0.3886;
	stance_position_[LH] << -0.4269, 0.3886;
	stance_position_[RH] << -0.4269, -0.3886;

	// Defining the pattern of locomotion
	pattern_locomotion_.resize(number_legs_);
	pattern_locomotion_[0] = LH;
	pattern_locomotion_[1] = LF;
	pattern_locomotion_[2] = RH;
	pattern_locomotion_[3] = RF;

	// Defining the search areas for the stance position of HyQ
	SearchArea stance_area;
	stance_area.grid_resolution = 0.12;
	for (int i = 0; i < number_legs_; i++) {
		stance_area.max_x = stance_position_[i](0) + 0.1;
		stance_area.min_x = stance_position_[i](0) - 0.1;
		stance_area.max_y = stance_position_[i](1) + 0.1;
		stance_area.min_y = stance_position_[i](1) - 0.1;
		stance_areas_.push_back(stance_area);
	}

	// Defining the body area of HyQ
	body_area_.grid_resolution = 0.04;
	body_area_.max_x = stance_position_[LF](0);
	body_area_.min_x = stance_position_[LH](0);
	body_area_.max_y = stance_position_[LF](1);
	body_area_.min_y = stance_position_[RF](1);
}


Robot::~Robot()
{

}


void Robot::setStanceAreas(std::vector<SearchArea> stance_areas)
{
	stance_areas_ = stance_areas;
}


const SearchArea& Robot::getBodyArea() const
{
	return body_area_;
}

const std::vector<SearchArea>& Robot::getStanceAreas() const
{
	return stance_areas_;
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
