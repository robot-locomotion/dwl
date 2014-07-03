#include <robot/Robot.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : number_legs_(4)
{
	stance_position_.resize(number_legs_);
	stance_position_[LF] << 0.4269, 0.3886;
	stance_position_[RF] << 0.4269, -0.3886;
	stance_position_[LH] << -0.4269, 0.3886;
	stance_position_[RH] << -0.4269, -0.3886;

	pattern_locomotion_.resize(number_legs_);
	pattern_locomotion_[0] = LH;
	pattern_locomotion_[1] = LF;
	pattern_locomotion_[2] = RH;
	pattern_locomotion_[3] = RF;


	SearchArea area;
	area.grid_resolution = 0.04;

	// Defining the search areas for the stance position of HyQ
	for (int i = 0; i < number_legs_; i++) {
		area.max_x = stance_position_[i][0] + 0.1;
		area.min_x = stance_position_[i][0] - 0.1;
		area.max_y = stance_position_[i][1] + 0.1;
		area.min_y = stance_position_[i][1] - 0.1;
		stance_areas_.push_back(area);
	}
}


Robot::~Robot()
{

}


void Robot::setStanceAreas(std::vector<SearchArea> stance_areas)
{

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
