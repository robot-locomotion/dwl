#include <planning/SearchBasedBodyMotionPlanning.h>


namespace dwl
{

namespace planning
{

SearchBasedBodyMotionPlanning::SearchBasedBodyMotionPlanning()
{
	name_ = "Search-Based Body Motion";
}


SearchBasedBodyMotionPlanning::~SearchBasedBodyMotionPlanning()
{

}


bool SearchBasedBodyMotionPlanning::computePath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose)
{
	// Computing the yaw angle of the start and goal pose
	double start_roll, goal_roll, start_pitch, goal_pitch, start_yaw, goal_yaw;
	Orientation start_orientation(start_pose.orientation), goal_orientation(goal_pose.orientation);
	start_orientation.getRPY(start_roll, start_pitch, start_yaw);
	goal_orientation.getRPY(goal_roll, goal_pitch, goal_yaw);

	// Computing the start and goal state
	Eigen::Vector3d start_state, goal_state;
	start_state << start_pose.position.head(2), start_yaw;
	goal_state << goal_pose.position.head(2), goal_yaw;

	// Converting the states to vertexs
	Vertex start_vertex, goal_vertex;
	environment_->getTerrainSpaceModel().stateToVertex(start_vertex, start_state);
	environment_->getTerrainSpaceModel().stateToVertex(goal_vertex, goal_state);

	// Computing the body path using a graph searching algorithm
	if (!path_solver_->compute(start_vertex, goal_vertex, path_computation_time_))
		return false;

	// Getting the shortest path
	std::list<Vertex> shortest_path = path_solver_->getShortestPath(start_vertex, goal_vertex);

	std::cout << "Path = " << std::endl;
	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_pose;
		body_pose.position = Eigen::Vector3d::Zero();
		body_pose.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector3d path;
		environment_->getTerrainSpaceModel().vertexToState(path, *path_iter);

		std::cout << path(0) << " " << path(1) << " " << path(2) << std::endl;

		// Converting the yaw angle to quaternion
		Eigen::Quaterniond q;
		Orientation orientation(0, 0, path(2));
		orientation.getQuaternion(q);

		// Setting the planned body pose
		body_pose.position.head(2) = path.head(2);
		body_pose.orientation = q;

		body_path.push_back(body_pose);
	}

	return true;
}

} //@namespace planning
} //@namespace dwl
