#include <robot/Robot.h>
#include <behavior/BodyMotorPrimitives.h>
#include <utils/Math.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : body_behavior_(NULL), number_legs_(0), number_end_effectors_(0), estimated_ground_from_body_(-0.55), last_past_leg_(1)
{
	body_behavior_ = new behavior::BodyMotorPrimitives();



	// Defining the pattern of locomotion
	pattern_locomotion_.resize(number_legs_);
	pattern_locomotion_[LF] = RH;
	pattern_locomotion_[RF] = LH;
	pattern_locomotion_[LH] = LF;
	pattern_locomotion_[RH] = RF;

	// Defining the stance position per leg
//	nominal_stance_.resize(number_legs_);
//	nominal_stance_[LF] << 0.36, 0.32, estimated_ground_from_body_;
//	nominal_stance_[RF] << 0.36, -0.32, estimated_ground_from_body_;
//	nominal_stance_[LH] << -0.36, 0.32, estimated_ground_from_body_;
//	nominal_stance_[RH] << -0.36, -0.32, estimated_ground_from_body_;

	// Defining the search areas for the stance position of HyQ
//	SearchArea stance_area;
//	stance_area.grid_resolution = 0.04;
//	for (int i = 0; i < number_legs_; i++) {
//		stance_area.max_x = nominal_stance_[i](0) + stance_size_;
//		stance_area.min_x = nominal_stance_[i](0) - stance_size_;
//		stance_area.max_y = nominal_stance_[i](1) + stance_size_;
//		stance_area.min_y = nominal_stance_[i](1) - stance_size_;
//		stance_areas_.push_back(stance_area);
//	}
//
//	// Defining the body area of HyQ
//	body_area_.max_x = nominal_stance_[LF](0);
//	body_area_.min_x = nominal_stance_[LH](0);
//	body_area_.max_y = nominal_stance_[LF](1);
//	body_area_.min_y = nominal_stance_[RF](1);

	// Defining the leg areas
//	double leg_workspace = 0.35;
//	SearchArea leg_area;
//	leg_area.grid_resolution = 0.04;
//	for (int leg_id = 0; leg_id < number_legs_; leg_id++) {
//		if ((leg_id == LF) || (leg_id == RF)) {
//			leg_area.min_x = -leg_workspace;
//			leg_area.max_x = 1.5 * stance_size_;
//			leg_area.min_y = -stance_size_;
//			leg_area.max_y = stance_size_;
//		} else {
//			leg_area.min_x = -1.5 * stance_size_;
//			leg_area.max_x = leg_workspace;
//			leg_area.min_y = -stance_size_;
//			leg_area.max_y = stance_size_;
//		}
//
//		leg_areas_.push_back(leg_area);
//	}
}


Robot::~Robot()
{

}


behavior::MotorPrimitives& Robot::getBodyMotorPrimitive()
{
	return *body_behavior_;
}


void Robot::read(std::string filepath)
{
	std::ifstream fin(filepath.c_str());

	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
//		for (unsigned it = 0; it < doc.size(); it++) {
//		    std::string key, value;
//		    it.first() >> key;
//		    it.second() >> value;
//		    std::cout << "Key: " << key << ", value: " << value << std::endl;

		// Reading the name of the robot
		std::string robot_name;
		it.first() >> robot_name;
		std::cout << robot_name << std::endl;

		// Reading the robot properties
		if (const YAML::Node* probot = doc.FindValue(robot_name)) {
			const YAML::Node& robot = *probot;

			// Reading the end-effectors of the robot
			if (const YAML::Node* pend_effectors = robot.FindValue("end_effectors")) {
				const YAML::Node& end_effectors = *pend_effectors;
				number_end_effectors_ = end_effectors.size();
				for (unsigned e = 0; e < number_end_effectors_; e++) {
					std::string name_leg;
					end_effectors[e] >> name_leg;
					end_effectors_[name_leg] = e;

					std::cout << "end_effector[" << e << "] = " << name_leg << std::endl; //TODO
				}
			}

			// Reading the nominal stance of the robot
			int leg_counter = 0;
			if (const YAML::Node* pnom_stance = robot.FindValue("nominal_stance")) {
				const YAML::Node& nom_stance = *pnom_stance;
				for (ContactID::iterator it = end_effectors_.begin(); it != end_effectors_.end(); ++it) {
					std::string leg_name = it->first;

					std::cout << leg_name << std::endl; //TODO

					if (const YAML::Node* plegs = nom_stance.FindValue(leg_name)) {
						leg_counter++;
						const YAML::Node& legs = *plegs;

						Eigen::Vector3d stance;
						if (const YAML::Node* px = legs.FindValue("x")) {
							const YAML::Node& x = *px;
							x >> stance(0);
						}
						if (const YAML::Node* py = legs.FindValue("y")) {
							const YAML::Node& y = *py;
							y >> stance(1);
						}
						stance(2) = estimated_ground_from_body_;
						nominal_stance_.push_back(stance);

						std::cout << stance << std::endl; //TODO
					}
				}
			}

			number_legs_ = leg_counter;

			// Reading the footstep search window
			if (const YAML::Node* pfootstep_area = robot.FindValue("footstep_search_window")) {
				const YAML::Node& footstep_area = *pfootstep_area;

				footstep_window_.grid_resolution = 0.04;
				if (const YAML::Node* pmin_x = footstep_area.FindValue("min_x")) {
					const YAML::Node& min_x = *pmin_x;
					min_x >> footstep_window_.min_x;
				}
				if (const YAML::Node* pmin_y = footstep_area.FindValue("min_y")) {
					const YAML::Node& min_y = *pmin_y;
					min_y >> footstep_window_.min_y;
				}
				if (const YAML::Node* pmax_x = footstep_area.FindValue("max_x")) {
					const YAML::Node& max_x = *pmax_x;
					max_x >> footstep_window_.max_x;
				}
				if (const YAML::Node* pmax_y = footstep_area.FindValue("max_y")) {
					const YAML::Node& max_y = *pmax_y;
					max_y >> footstep_window_.max_y;
				}


				std::cout << footstep_window_.min_x << " " << footstep_window_.min_y << " " << footstep_window_.max_x << " " << footstep_window_.max_y << std::endl;
			}
		}
	}

//	is_defined_motor_primitives_ = true;
}


void Robot::setCurrentPose(Pose pose)
{
	current_pose_ = pose;
}


void Robot::setCurrentContacts(std::vector<Contact> contacts)
{
	current_contacts_ = contacts;

	estimated_ground_from_body_ = 0;
	for (int i = 0; i < number_legs_; i++) //TODO
		estimated_ground_from_body_ += contacts[i].position(2);

	estimated_ground_from_body_ /= number_legs_;
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
	double displacement_x, leg_offset;
	double frontal_action = action(0);
	if (frontal_action == 0)
		displacement_x = 0;
	else if (frontal_action > 0)
		displacement_x = -0.04;
	else
		displacement_x = 0.04;


	//TODO Clean this shit
	int past_leg_id;
	double angular_tolerance = 0.2;
	if ((action(2) >= -M_PI_2 - angular_tolerance) && (action(2) <= -M_PI_2 + angular_tolerance)) {
		past_leg_id = 0;
		leg_offset = -0.06;
	} else if ((action(2) >= M_PI_2 - angular_tolerance) && (action(2) <= M_PI_2 + angular_tolerance)) {
		past_leg_id = 1;
		leg_offset = 0.06;
	} else if (action(2) > angular_tolerance) {
		past_leg_id = 0;
		leg_offset = 0;
	} else if (action(2) < -angular_tolerance) {
		past_leg_id = 1;
		leg_offset = 0;
	} else {
		past_leg_id = last_past_leg_;
		if (past_leg_id == 0)
			leg_offset = -0.06;
		else
			leg_offset = 0.06;
	}
	last_past_leg_ = past_leg_id;


	// Defining the stance position per leg
	std::vector<Eigen::Vector3d> nominal_stance;
	nominal_stance.resize(number_legs_);
	nominal_stance[LF] << 0.36 - leg_offset + displacement_x, 0.32, estimated_ground_from_body_;
	nominal_stance[RF] << 0.36 + leg_offset + displacement_x, -0.32, estimated_ground_from_body_;
	nominal_stance[LH] << -0.36 - leg_offset + displacement_x, 0.32, estimated_ground_from_body_;
	nominal_stance[RH] << -0.36 + leg_offset + displacement_x, -0.32, estimated_ground_from_body_;

	return nominal_stance;
}


std::vector<int> Robot::getPatternOfLocomotion()
{
	return pattern_locomotion_;
}


std::vector<SearchArea> Robot::getFootstepSearchAreas(Eigen::Vector3d action)
{
	std::vector<Eigen::Vector3d> nominal_stance = getNominalStance(action);

	std::vector<SearchArea> footstep_areas;
	SearchArea footstep_area;
	footstep_area.grid_resolution = 0.04;
	for (int i = 0; i < number_legs_; i++) {
		footstep_area.max_x = nominal_stance[i](0) + footstep_window_.max_x;
		footstep_area.min_x = nominal_stance[i](0) + footstep_window_.min_x;
		footstep_area.max_y = nominal_stance[i](1) + footstep_window_.max_y;
		footstep_area.min_y = nominal_stance[i](1) + footstep_window_.min_y;
		footstep_areas.push_back(footstep_area);
	}

	return footstep_areas;
}


double Robot::getExpectedGround(int leg_id)
{
	return current_pose_.position(2) + estimated_ground_from_body_;
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
