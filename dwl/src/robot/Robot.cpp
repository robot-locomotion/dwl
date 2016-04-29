#include <dwl/robot/Robot.h>
#include <dwl/behavior/BodyMotorPrimitives.h>
#include <dwl/utils/Math.h>


namespace dwl
{

namespace robot
{

Robot::Robot() : body_behavior_(NULL), num_feet_(0), num_end_effectors_(0),
		estimated_ground_from_body_(-0.55), last_past_foot_(1),
		feet_lateral_offset_(0), displacement_(0)
{
	body_behavior_ = new behavior::BodyMotorPrimitives();
}


Robot::~Robot()
{

}


void Robot::read(std::string filename)
{
	// TODO The id doesn't follow the urdf order
	// Yaml reader
	dwl::YamlWrapper yaml_reader();
	yaml_reader_.setFile(filename);

	// Parsing the configuration file
	std::string robot_ns = "robot";
	printf("Reading the configuration parameters from the %s namespace\n",
			robot_ns.c_str());

	// Getting the different nodes
	std::vector<std::string> description_ns = {robot_ns, "description"};
	std::vector<std::string> ee_description_ns = {robot_ns, "description", "end_effector_descriptions"};
	std::vector<std::string> properties_ns = {robot_ns, "predefined_properties"};
	std::vector<std::string> pattern_ns = {robot_ns, "predefined_properties", "pattern_locomotion"};
	std::vector<std::string> stance_ns = {robot_ns, "predefined_properties", "nominal_stance"};
	std::vector<std::string> footregion_ns = {robot_ns, "predefined_properties", "footstep_search_window"};
	std::vector<std::string> footws_ns = {robot_ns, "predefined_properties", "foot_workspace"};

	// Reading the end-effectors of the robot
	std::vector<std::string> end_effectors;
	if (yaml_reader_.read(end_effectors, "end_effectors", description_ns)) {
		for (std::size_t i = 0; i < end_effectors.size(); i++)
			end_effectors_[i] = end_effectors[i];
	} else
		printf(YELLOW "Warning: the end-effector was not read\n" COLOR_RESET);

	// Reading the feet of the robot
	std::vector<std::string> feet;
	if (yaml_reader_.read(feet, "feet", description_ns)) {
		num_feet_ = feet.size();
		for (unsigned int i = 0; i < num_feet_; i++)
			feet_[i] = feet[i];
	} else
		printf(YELLOW "Warning: the feet was not read\n" COLOR_RESET);

	// Reading the end-effectors description
	for (EndEffectorMap::iterator e = end_effectors_.begin();
			e != end_effectors_.end(); e++) {
		unsigned int id = e->first;
		std::string name = e->second;

		std::vector<std::string> patchs;
		if (yaml_reader_.read(patchs, name, ee_description_ns)) {
			for (unsigned int k = 0; k < patchs.size(); k++)
				patchs_[id] = patchs;
		} else
			printf(YELLOW "Warning: the patches of %s was not read\n" COLOR_RESET,
					name.c_str());
	}

	// Reading the pattern of locomotion
	for (EndEffectorMap::iterator f = feet_.begin(); f != feet_.end(); ++f) {
		unsigned int id = f->first;
		std::string name = f->second;

		// Reading the next swing leg
		std::string next_feet_name;
		unsigned int next_feet_id = 0;
		if (yaml_reader_.read(next_feet_name, name, pattern_ns)) {
			for (EndEffectorMap::iterator l = feet_.begin(); l != feet_.end(); ++l) {
				std::string potential_leg_name = l->second;
				if (next_feet_name == potential_leg_name) {
					next_feet_id = l->first;
					break;
				}
			}
			pattern_locomotion_[id] = next_feet_id;
		} else
			printf(YELLOW "Warning: the next leg of %s was not read\n" COLOR_RESET,
					name.c_str());
	}

	// Reading the nominal stance position
	for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
		std::string name = it->second;

		// Reading the nominal stance position for the actual foot
		Eigen::Vector3d stance;
		if (yaml_reader_.read(stance, name, stance_ns)) {
			unsigned int id = it->first;

			stance(2) = estimated_ground_from_body_;
			nominal_stance_[id] = stance;
		} else
			printf(YELLOW "Warning: the position of %s leg was not read\n" COLOR_RESET,
					name.c_str());
	}

	// Reading the lateral offset
	if (!yaml_reader_.read(feet_lateral_offset_, "lateral_offset", stance_ns))
		printf(YELLOW "Warning: the lateral offset was not read\n" COLOR_RESET);

	// Reading the frontal displacement
	if (!yaml_reader_.read(displacement_, "displacement", stance_ns))
		printf(YELLOW "Warning: the displacement was not read\n" COLOR_RESET);

	// Reading the footstep search window
	for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
		std::string name = it->second;

		// Reading the search window for the actual foot
		SearchArea search_area;
		if (yaml_reader_.read(search_area, name, footregion_ns)) {
			unsigned int id = it->first;
			footstep_window_[id] = search_area;
		} else
			printf(YELLOW "Warning: the footstep search window of %s leg was not read\n"
					COLOR_RESET, name.c_str());
	}

	// Reading the leg work window
	for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
		std::string name = it->second;

		// Reading the work window for the actual foot
		SearchArea work_area;
		if (yaml_reader_.read(work_area, name, footws_ns)) {
			unsigned int id = it->first;
			foot_workspaces_[id] = work_area;
		} else
			printf(YELLOW "Warning: the workspace of %s foot was not read\n" COLOR_RESET,
					name.c_str());
	}

	// Reading the body work window
	if (!yaml_reader_.read(body_workspace_, "body_workspace", properties_ns))
		printf(YELLOW "Warning: the body workspace description was not read\n" COLOR_RESET);
}


void Robot::setCurrentPose(const Pose& pose)
{
	current_pose_ = pose;
}


void Robot::setCurrentContacts(const std::vector<Contact>& contacts)
{
	current_contacts_ = contacts;

	estimated_ground_from_body_ = 0;
	for (int i = 0; i < num_feet_; i++) //TODO
		estimated_ground_from_body_ += contacts[i].position(2);

	estimated_ground_from_body_ /= num_feet_;
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
	return foot_workspaces_;
}


Vector3dMap Robot::getStance(const Eigen::Vector3d& action) //TODO Virtual method
{
	int lateral_pattern, displacement_pattern;
	double frontal_action = action(0);
	if (frontal_action == 0)
		displacement_pattern = 0;
	else if (frontal_action > 0)
		displacement_pattern = -1;
	else
		displacement_pattern = 1;


	// Getting the initial leg and lateral displacement
	int past_foot_id;
	double angular_tolerance = 0.2;
	if ((action(2) >= -M_PI_2 - angular_tolerance) &&
			(action(2) <= -M_PI_2 + angular_tolerance)) {
		past_foot_id = 0;
		lateral_pattern = -1;
	} else if ((action(2) >= M_PI_2 - angular_tolerance) &&
			(action(2) <= M_PI_2 + angular_tolerance)) {
		past_foot_id = 1;
		lateral_pattern = 1;
	} else if (action(2) > angular_tolerance) {
		past_foot_id = 0;
		lateral_pattern = 0;
	} else if (action(2) < -angular_tolerance) {
		past_foot_id = 1;
		lateral_pattern = 0;
	} else {
		past_foot_id = last_past_foot_;
		if (past_foot_id == 0)
			lateral_pattern = -1;
		else
			lateral_pattern = 1;
	}
	last_past_foot_ = past_foot_id;


	// Defining the stance position per leg
	Vector3dMap stance;
	for (EndEffectorMap::iterator it = feet_.begin(); it != feet_.end(); ++it) {
		unsigned int id = it->first;
		std::string name = it->second;

		Eigen::Vector3d position;
		if ((name == "lf_foot") || (name == "lh_foot"))
			position(0) = nominal_stance_[id](0) -
				lateral_pattern * feet_lateral_offset_ +
				displacement_pattern * displacement_;
		else
			position(0) = nominal_stance_[id](0) +
				lateral_pattern * feet_lateral_offset_ +
				displacement_pattern * displacement_;

		position(1) = nominal_stance_[id](1);
		position(2) = estimated_ground_from_body_;
		stance[id] = position;
	}

	return stance;
}


Vector3dMap Robot::getNominalStance()
{
	return nominal_stance_;
}


PatternOfLocomotionMap Robot::getPatternOfLocomotion()
{
	return pattern_locomotion_;
}


SearchAreaMap Robot::getFootstepSearchAreas(const Eigen::Vector3d& action)
{
	// Getting the current stance
	Vector3dMap current_stance = getStance(action);

	// Getting the footstep search regions
	SearchAreaMap footstep_areas;
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


SearchAreaMap Robot::getFootstepSearchSize(const Eigen::Vector3d& action)
{
	// Determining if the movements is forward or backward because the footstep search areas
	// changes according the action
	int displacement_pattern;
	double frontal_action = action(0);
	if (frontal_action >= 0)
		displacement_pattern = 1;
	else
		displacement_pattern = -1;

	SearchAreaMap footstep_areas;
	SearchArea footstep_area;
	for (EndEffectorMap::iterator l = feet_.begin(); l != feet_.end(); l++) {
		unsigned int leg_id = l->first;
		footstep_area.resolution = footstep_window_[leg_id].resolution;
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
	return num_feet_;
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
