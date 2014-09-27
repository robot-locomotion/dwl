#include <robot/Robot.h>

#include <utils/Math.h>

namespace dwl
{

namespace robot
{

Robot::Robot() : number_legs_(4), stance_size_(0.1)
{
	// Defining the stance position per leg
	double displacement_x = 0.04;
	nominal_stance_.resize(number_legs_);
	nominal_stance_[LF] << 0.36, 0.32, -0.5757;
	nominal_stance_[RF] << 0.36, -0.32, -0.5757;
	nominal_stance_[LH] << -0.36, 0.32, -0.5757;
	nominal_stance_[RH] << -0.36, -0.32, -0.5757;
	forward_nominal_stance_.resize(number_legs_);
	forward_nominal_stance_[LF] << 0.36 - displacement_x, 0.32, -0.5757;
	forward_nominal_stance_[RF] << 0.36 - displacement_x, -0.32, -0.5757;
	forward_nominal_stance_[LH] << -0.36 - displacement_x, 0.32, -0.5757;
	forward_nominal_stance_[RH] << -0.36 - displacement_x, -0.32, -0.5757;
	backward_nominal_stance_.resize(number_legs_);
	backward_nominal_stance_[LF] << 0.36 + displacement_x, 0.32, -0.5757;
	backward_nominal_stance_[RF] << 0.36 + displacement_x, -0.32, -0.5757;
	backward_nominal_stance_[LH] << -0.36 + displacement_x, 0.32, -0.5757;
	backward_nominal_stance_[RH] << -0.36 + displacement_x, -0.32, -0.5757;

	// Defining the pattern of locomotion
	pattern_locomotion_.resize(number_legs_);
	pattern_locomotion_[LF] = RH;
	pattern_locomotion_[RF] = LH;
	pattern_locomotion_[LH] = LF;
	pattern_locomotion_[RH] = RF;

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
	for (int i = 0; i < number_legs_; i++) {
		stance_area.max_x = forward_nominal_stance_[i](0) + stance_size_;
		stance_area.min_x = forward_nominal_stance_[i](0) - stance_size_;
		stance_area.max_y = forward_nominal_stance_[i](1) + stance_size_;
		stance_area.min_y = forward_nominal_stance_[i](1) - stance_size_;
		forward_stance_areas_.push_back(stance_area);
	}
	for (int i = 0; i < number_legs_; i++) {
		stance_area.max_x = backward_nominal_stance_[i](0) + stance_size_;
		stance_area.min_x = backward_nominal_stance_[i](0) - stance_size_;
		stance_area.max_y = backward_nominal_stance_[i](1) + stance_size_;
		stance_area.min_y = backward_nominal_stance_[i](1) - stance_size_;
		backward_stance_areas_.push_back(stance_area);
	}

	// Defining the body area of HyQ
	body_area_.max_x = nominal_stance_[LF](0);
	body_area_.min_x = nominal_stance_[LH](0);
	body_area_.max_y = nominal_stance_[LF](1);
	body_area_.min_y = nominal_stance_[RF](1);

	// Defining the leg areas
	double leg_workspace = 0.35;
	SearchArea leg_area;
	leg_area.grid_resolution = 0.04;
	for (int leg_id = 0; leg_id < number_legs_; leg_id++) {
		if ((leg_id == LF) || (leg_id == RF)) {
			leg_area.min_x = -leg_workspace;
			leg_area.max_x = 2.5 * stance_size_;
			leg_area.min_y = -stance_size_;
			leg_area.max_y = stance_size_;
		} else {
			leg_area.min_x = -stance_size_;
			leg_area.max_x = leg_workspace;
			leg_area.min_y = -stance_size_;
			leg_area.max_y = stance_size_;
		}

		leg_areas_.push_back(leg_area);
	}
}


Robot::~Robot()
{

}


void Robot::setCurrentPose(Pose pose)
{
	current_pose_ = pose;
}


void Robot::setCurrentContacts(std::vector<Contact> contacts)
{
	current_contacts_ = contacts;
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


std::vector<Contact> Robot::getCurrentContacts()
{
	return current_contacts_;
}


Area Robot::getBodyArea()
{
	return body_area_;
}


std::vector<Eigen::Vector3d> Robot::getNominalStance(Eigen::Vector3d action)
{
	std::vector<Eigen::Vector3d> nominal_stance;
	double frontal_action = action(0);
	if (frontal_action == 0)
		nominal_stance = nominal_stance_;
	else if (frontal_action > 0)
		nominal_stance = forward_nominal_stance_;
	else
		nominal_stance = backward_nominal_stance_;

	return nominal_stance;
}


std::vector<int> Robot::getPatternOfLocomotion()
{
	return pattern_locomotion_;
}


std::vector<SearchArea> Robot::getStanceAreas(Eigen::Vector3d action)
{
	std::vector<SearchArea> stance_areas;
	double frontal_action = action(0);
	if (frontal_action == 0)
		stance_areas = stance_areas_;
	else if (frontal_action > 0)
		stance_areas = forward_stance_areas_;
	else
		stance_areas = backward_stance_areas_;

	return stance_areas;
}


double Robot::getExpectedGround(int leg_id)
{
	return current_pose_.position(2) + nominal_stance_[leg_id](2);
}


std::vector<SearchArea> Robot::getLegWorkAreas()
{
	return leg_areas_;
}


double Robot::getNumberOfLegs()
{
	return number_legs_;
}

} //@namespace robot
} //@namespace dwl
