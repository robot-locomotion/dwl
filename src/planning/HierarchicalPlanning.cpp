#include <planning/HierarchicalPlanning.h>
#include <utils/Orientation.h> //TODO


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
	// Converting the start and goal position to vertex ids
/*	initial_pose_ = start;*/
	goal_pose_ =  goal;
}


bool HierarchicalPlanning::compute(Pose current_pose)
{
	// Converting pose to 3d state (x,y,yaw)
	Eigen::Vector3d current_state;
	poseToState(current_state, current_pose);

	if (environment_->isTerrainInformation()) {
		Vertex current_vertex;
		environment_->getSpaceModel().stateToVertex(current_vertex, current_state);

		// Cleaning global variables
		std::vector<Pose> empty_body_trajectory;
		body_path_.swap(empty_body_trajectory);
		std::vector<Contact> empty_contacts_sequence;
		contacts_sequence_.swap(empty_contacts_sequence);

		double roll, pitch, yaw;
		Orientation orientation(current_pose.orientation);
		orientation.getRPY(roll, pitch, yaw);
		std::cout << "Current position = " << current_pose.position(0) << " " << current_pose.position(1) << " " << yaw << std::endl;

		// Computing the body path using a graph searching algorithm
		if (!body_planner_->computeBodyPath(body_path_, current_pose, goal_pose_)) {
			printf(YELLOW "Could not found an approximated body path\n" COLOR_RESET);
			return false;
		}

		for (int i = 1; i < body_path_.size(); i++) {//3; i++) {
			Orientation orientation(body_path_[i].orientation);
			double roll, pitch, yaw;
			orientation.getRPY(roll, pitch, yaw);
			std::cout << "Plan = " << body_path_[i].position(0) << " " << body_path_[i].position(1) << " " << yaw << std::endl;
			if (!footstep_planner_->computeFootholds(contacts_sequence_, body_path_[i])) {
				printf(YELLOW "Could not computed the footholds \n" COLOR_RESET);
				return false;
			}
		}
	} else {
		printf(YELLOW "Could not computed a locomotion plan because there is not terrain information\n" COLOR_RESET);
		return false;
	}

	return true;
}


void HierarchicalPlanning::poseToState(Eigen::Vector3d& state, Pose pose)
{
	double roll, pitch, yaw;
	Orientation orientation(pose.orientation);
	orientation.getRPY(roll, pitch, yaw);
	state << pose.position.head(2), yaw;
}

} //@namespace planning
} //@namespace dwl
