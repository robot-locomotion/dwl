#include <planning/HierarchicalPlanning.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace planning
{

HierarchicalPlanning::HierarchicalPlanning() : goal_vertex_(0)
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
	// Converting the start and goal position to vertex ids
/*	initial_pose_ = start;*/
	goal_pose_ =  goal;

	if (environment_->isTerrainInformation())
		goal_vertex_ = environment_->getGridModel().coordToVertex((Eigen::Vector2d) goal.position.head(2));
}


bool HierarchicalPlanning::compute(Pose current_pose)
{
	if (environment_->isTerrainInformation()) {
		Vertex current_vertex = environment_->getGridModel().coordToVertex((Eigen::Vector2d) current_pose.position.head(2));

		// Cleaning global variables
		std::vector<Pose> empty_body_trajectory;
		body_path_.swap(empty_body_trajectory);
		std::vector<Contact> empty_contacts_sequence;
		contacts_sequence_.swap(empty_contacts_sequence);

		// Converting quaternion to roll, pitch and yaw angles
		double roll, pitch, yaw;
		Orientation orientation(current_pose.orientation);
		orientation.getRPY(roll, pitch, yaw);

		// Computing the body path using a graph searching algorithm
		if (!body_planner_->computeBodyPath(body_path_, current_pose, goal_pose_)) {
			printf(YELLOW "Could not found an approximated body path\n" COLOR_RESET);
			return false;
		}
		for (int i = 1; i < body_path_.size() - 1; i++) {
			if (!footstep_planner_->computeFootholds(contacts_sequence_, body_path_[i])) {//current_pose
				printf(YELLOW "Could not computed the footholds\n" COLOR_RESET);
				return false;
			}
		}
	} else {
		printf(YELLOW "Could not computed a locomotion plan because there is not terrain information\n" COLOR_RESET);
		return false;
	}


	return true;
}

} //@namespace planning
} //@namespace dwl
