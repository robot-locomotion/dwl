#include <robot/Robot.h>
#include <behavior/BodyMotorPrimitives.h>
#include <utils/Math.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : body_behavior_(NULL), number_legs_(0), number_end_effectors_(0),
		estimated_ground_from_body_(-0.55), last_past_leg_(1), leg_lateral_offset_(0),
		displacement_(0)
{
	body_behavior_ = new behavior::BodyMotorPrimitives();
}


Robot::~Robot()
{

}


void Robot::read(std::string filepath)
{
	std::ifstream fin(filepath.c_str());

	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
		// Reading the name of the robot
		std::string robot_name;
		yaml_reader_.read(it.first(), robot_name);

		// Reading the robot properties
		if (const YAML::Node* probot = doc.FindValue(robot_name)) {
			const YAML::Node& robot = *probot;

			// Reading the general description of the robot
			if (const YAML::Node* pdescription = robot.FindValue("description")) {
				const YAML::Node& description = *pdescription;

				// Reading the end-effectors of the robot
				if (const YAML::Node* pend_effectors = description.FindValue("end_effectors")) {
					std::vector<std::string> end_effectors;
					yaml_reader_.read(*pend_effectors, end_effectors);
					for (unsigned int i = 0; i < end_effectors.size(); i++) {
						end_effectors_[i] = end_effectors[i];
					}
				} else
					printf(YELLOW "Warning: the end-effector was not read\n" COLOR_RESET);

				// Reading the feet of the robot
				if (const YAML::Node* pfeet = description.FindValue("feet")) {
					std::vector<std::string> feet;
					yaml_reader_.read(*pfeet, feet);
					number_legs_ = feet.size();
					for (unsigned int i = 0; i < number_legs_; i++) {
						feet_[i] = feet[i];
					}
				} else
					printf(YELLOW "Warning: the feet was not read\n" COLOR_RESET);

				// Reading the end-effectors description
				if (const YAML::Node* pee_description = description.FindValue("end_effector_descriptions")) {
					const YAML::Node& ee_description = *pee_description;

					for (EndEffectorMap::iterator e = end_effectors_.begin(); e != end_effectors_.end(); e++) {
						unsigned int ee_id = e->first;
						std::string ee_name = e->second;

						if (const YAML::Node* ppatchs = ee_description.FindValue(ee_name)) {
							std::vector<std::string> patchs;
							yaml_reader_.read(*ppatchs, patchs);
							patchs_[ee_id] = patchs;
						}
					}
				}
			} else
				printf(YELLOW "Warning: the description was not read\n" COLOR_RESET);


			// Reading the predefined properties
			if (const YAML::Node* pproperties = robot.FindValue("predefined_properties")) {
				const YAML::Node& properties = *pproperties;

				if (const YAML::Node* ppattern_locomotion = properties.FindValue("pattern_locomotion")) {
					const YAML::Node& pattern_locomotion = *ppattern_locomotion;

					for (EndEffectorMap::iterator f = feet_.begin(); f != feet_.end(); ++f) {
						unsigned int leg_id = f->first;
						std::string leg_name = f->second;

						if (const YAML::Node* pleg = pattern_locomotion.FindValue(leg_name)) {
							std::string next_leg_name;
							unsigned int next_leg_id = 0;
							yaml_reader_.read(*pleg, next_leg_name);
							for (EndEffectorMap::iterator l = feet_.begin(); l != feet_.end(); ++l) {
								std::string potential_leg_name = l->second;
								if (next_leg_name == potential_leg_name) {
									next_leg_id = l->first;
									break;
								}
							}

							pattern_locomotion_[leg_id] = next_leg_id;
						} else
							printf(YELLOW "Warning: the next leg of %s was not read\n" COLOR_RESET, leg_name.c_str());
					}
				} else
					printf(YELLOW "Warning: the pattern of locomotion was not read\n" COLOR_RESET);


				// Reading the nominal stance properties of the robot
				if (const YAML::Node* pnom_stance = properties.FindValue("nominal_stance")) {
					const YAML::Node& nom_stance = *pnom_stance;

					for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
						std::string leg_name = it->second;
						if (const YAML::Node* pleg = nom_stance.FindValue(leg_name)) {
							unsigned int leg_id = it->first;

							Eigen::Vector3d stance;
							yaml_reader_.read(*pleg, stance);
							stance(2) = estimated_ground_from_body_;
							nominal_stance_[leg_id] = stance;
						} else
							printf(YELLOW "Warning: the position of %s leg was not read\n" COLOR_RESET, leg_name.c_str());
					}

					if (const YAML::Node* pleg_offset = nom_stance.FindValue("lateral_offset"))
						yaml_reader_.read(*pleg_offset, leg_lateral_offset_);
					else
						printf(YELLOW "Warning: the lateral offset was not read\n" COLOR_RESET);

					if (const YAML::Node* pdisplacement = nom_stance.FindValue("displacement"))
						yaml_reader_.read(*pdisplacement, displacement_);
					else
						printf(YELLOW "Warning: the displacement was not read\n" COLOR_RESET);
				} else
					printf(YELLOW "Warning: the nominal stance description was not read\n" COLOR_RESET);


				// Reading the footstep search window
				if (const YAML::Node* pfootstep_area = properties.FindValue("footstep_search_window")) {
					const YAML::Node& footstep_area = *pfootstep_area;

					for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
						std::string leg_name = it->second;

						if (const YAML::Node* pfootstep = footstep_area.FindValue(leg_name)) {
							unsigned int leg_id = it->first;

							SearchArea footstep_area;
							yaml_reader_.read(*pfootstep, footstep_area);
							footstep_window_[leg_id] = footstep_area;
						} else
							printf(YELLOW "Warning: the footstep search window of %s leg was not read\n" COLOR_RESET, leg_name.c_str());
					}

//					yaml_reader_.read(*pfootstep_area, footstep_window_);
				} else
					printf(YELLOW "Warning: the footstep search window was not read\n" COLOR_RESET);


				// Reading the leg work window
				if (const YAML::Node* pleg_area = properties.FindValue("leg_workspace")) {
					const YAML::Node& leg_area = *pleg_area;

					for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
						std::string leg_name = it->second;

						if (const YAML::Node* pleg = leg_area.FindValue(leg_name)) {
							unsigned int leg_id = it->first;

							SearchArea leg_area;
							yaml_reader_.read(*pleg, leg_area);

							leg_workspaces_[leg_id] = leg_area;
						} else
							printf(YELLOW "Warning: the workspace of %s leg was not read\n" COLOR_RESET, leg_name.c_str());
					}
				} else
					printf(YELLOW "Warning: the leg workspace description was not read\n" COLOR_RESET);

				// Reading the body work window
				if (const YAML::Node* pbody_area = properties.FindValue("body_workspace")) {
					yaml_reader_.read(*pbody_area, body_workspace_);
				} else
					printf(YELLOW "Warning: the body workspace description was not read\n" COLOR_RESET);
			} else
				printf(YELLOW "Warning: the predefined properties was not read\n" COLOR_RESET);
		} else
			printf(YELLOW "Warning: the robot properties was not read\n" COLOR_RESET);
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


Pose Robot::getCurrentPose()
{
	return current_pose_;
}


std::vector<Contact> Robot::getCurrentContacts()
{
	return current_contacts_;
}


behavior::MotorPrimitives& Robot::getBodyMotorPrimitive()
{
	return *body_behavior_;
}


SearchArea Robot::getPredefinedBodyWorkspace()
{
	return body_workspace_;
}


SearchAreaMap Robot::getPredefinedLegWorkspaces()
{
	return leg_workspaces_;
}


Vector3dMap Robot::getStance(Eigen::Vector3d action) //TODO Virtual method
{
	int lateral_pattern, displacement_pattern;
	double frontal_action = action(0);
	if (frontal_action == 0)
		displacement_pattern = 0;
	else if (frontal_action > 0)
		displacement_pattern = -1;
	else
		displacement_pattern = 1;


	//TODO Clean this shit
	int past_leg_id;
	double angular_tolerance = 0.2;
	if ((action(2) >= -M_PI_2 - angular_tolerance) && (action(2) <= -M_PI_2 + angular_tolerance)) {
		past_leg_id = 0;//LF_foot;//
		lateral_pattern = -1;//leg_offset = -0.06;
	} else if ((action(2) >= M_PI_2 - angular_tolerance) && (action(2) <= M_PI_2 + angular_tolerance)) {
		past_leg_id = 1;//RF_foot;//
		lateral_pattern = 1;//leg_offset = 0.06;
	} else if (action(2) > angular_tolerance) {
		past_leg_id = 0;//LF_foot;//
		lateral_pattern = 0;//leg_offset = 0;
	} else if (action(2) < -angular_tolerance) {
		past_leg_id = 1;//RF_foot;//
		lateral_pattern = 0;//leg_offset = 0;
	} else {
		past_leg_id = last_past_leg_;
		if (past_leg_id == 0)
			lateral_pattern = -1;//leg_offset = -0.06;
		else
			lateral_pattern = 1;//leg_offset = 0.06;
	}
	last_past_leg_ = past_leg_id;


	// Defining the stance position per leg
	Vector3dMap stance;
	for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
		unsigned int leg_id = it->first;
		std::string leg_name = it->second;

		Eigen::Vector3d leg_position;
		if ((leg_name == "LF_foot") || (leg_name == "LH_foot"))
			leg_position(0) = nominal_stance_[leg_id](0) - lateral_pattern * leg_lateral_offset_ + displacement_pattern * displacement_;
		else
			leg_position(0) = nominal_stance_[leg_id](0) + lateral_pattern * leg_lateral_offset_ + displacement_pattern * displacement_;


		leg_position(1) = nominal_stance_[leg_id](1);
		leg_position(2) = estimated_ground_from_body_;
		stance[leg_id] = leg_position;
	}

	return stance;
}


PatternOfLocomotionMap Robot::getPatternOfLocomotion()
{
	return pattern_locomotion_;
}


SearchAreaMap Robot::getFootstepSearchAreas(Eigen::Vector3d action)
{
	Vector3dMap current_stance = getStance(action);


	SearchAreaMap footstep_areas;
	SearchArea stance_area;
	footstep_areas = getFootstepSearchSize(action);
	for (EndEffectorMap::iterator l = feet_.begin(); l != feet_.end(); l++) {
		unsigned int leg_id = l->first;
		footstep_areas[leg_id].max_x += current_stance[leg_id](0);
		footstep_areas[leg_id].min_x += current_stance[leg_id](0);
		footstep_areas[leg_id].max_y += current_stance[leg_id](1);
		footstep_areas[leg_id].min_y += current_stance[leg_id](1);
	}

	return footstep_areas;
}


SearchAreaMap Robot::getFootstepSearchSize(Eigen::Vector3d action)
{
	// Determining if the movements is forward or backward because the footstep search areas changes according the action
	int lateral_pattern, displacement_pattern;
	double frontal_action = action(0);
	if (frontal_action >= 0)
		displacement_pattern = 1;
	else
		displacement_pattern = -1;

	SearchAreaMap footstep_areas;
	SearchArea footstep_area;
	footstep_area.resolution = 0.04;
	for (EndEffectorMap::iterator l = feet_.begin(); l != feet_.end(); l++) {
		unsigned int leg_id = l->first;
		footstep_area.max_x = displacement_pattern * footstep_window_[leg_id].max_x;
		footstep_area.min_x = displacement_pattern * footstep_window_[leg_id].min_x;
		footstep_area.max_y = footstep_window_[leg_id].max_y;
		footstep_area.min_y = footstep_window_[leg_id].min_y;
		footstep_areas[leg_id] = footstep_area;
	}

	return footstep_areas;
}


double Robot::getExpectedGround(int leg_id)
{
	return current_pose_.position(2) + estimated_ground_from_body_;
}


double Robot::getNumberOfLegs()
{
	return number_legs_;
}


EndEffectorMap Robot::getEndEffectorMap()
{
	return end_effectors_;
}


EndEffectorMap Robot::getLegMap()
{
	return feet_;
}

} //@namespace robot
} //@namespace dwl
