#include <planning/HierarchicalPlanning.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace planning
{

HierarchicalPlanning::HierarchicalPlanning()
{
	name_ = "Hierarchical";
}


HierarchicalPlanning::~HierarchicalPlanning()
{

}


bool HierarchicalPlanning::init()
{
	printf("Initialized the hierarchical planning algorithm\n");

	//solver_->init();

	return true;
}


void HierarchicalPlanning::resetGoal(Pose goal)
{
	// Resetting the goal pose
	goal_pose_ =  goal;
}


bool HierarchicalPlanning::compute(Pose current_pose)
{
	// Setting the pose in the robot properties
	robot_->setCurrentPose(current_pose);

	if (environment_->isTerrainInformation()) {
		// Cleaning global variables
		std::vector<Pose> empty_body_trajectory;
		body_path_.swap(empty_body_trajectory);
		std::vector<Contact> empty_contacts_sequence;
		contacts_sequence_.swap(empty_contacts_sequence);

		// Computing the body path using a graph searching algorithm
		if (!body_planner_->computeBodyPath(body_path_, current_pose, goal_pose_)) {
			printf(YELLOW "Could not found an approximated body path\n" COLOR_RESET);
			return false;
		}

		if (!footstep_planner_->computeContactSequence(contacts_sequence_, body_path_)) {
			printf(YELLOW "Could not computed the foothold sequence \n" COLOR_RESET);
			return false;
		}
	}

	return true;
}

} //@namespace planning
} //@namespace dwl
