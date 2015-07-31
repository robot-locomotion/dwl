#include <locomotion/SearchBasedBodyMotionPlanning.h>


namespace dwl
{

namespace locomotion
{

SearchBasedBodyMotionPlanning::SearchBasedBodyMotionPlanning()
{
	name_ = "Search-Based Body Motion";
}


SearchBasedBodyMotionPlanning::~SearchBasedBodyMotionPlanning()
{

}


bool SearchBasedBodyMotionPlanning::computePath(std::vector<Pose>& body_path,
												Pose start_pose,
												Pose goal_pose)
{
	// Computing the yaw angle of the start and goal pose
	double start_yaw = math::getYaw(math::getRPY(start_pose.orientation));
	double goal_yaw = math::getYaw(math::getRPY(goal_pose.orientation));

	// Computing the start and goal state
	Eigen::Vector3d start_state, goal_state;
	start_state << start_pose.position.head(2), start_yaw;
	goal_state << goal_pose.position.head(2), goal_yaw;

	// Converting the states to vertexes
	Vertex start_vertex, goal_vertex;
	environment_->getTerrainSpaceModel().stateToVertex(start_vertex, start_state);
	environment_->getTerrainSpaceModel().stateToVertex(goal_vertex, goal_state);

	// Computing the body path using a graph searching algorithm
	if (!path_solver_->compute(start_vertex, goal_vertex, path_computation_time_))
		return false;

	// Getting the shortest path
	std::list<Vertex> shortest_path = path_solver_->getShortestPath(start_vertex, goal_vertex);

	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_pose;
		body_pose.position = Eigen::Vector3d::Zero();
		body_pose.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector3d path;
		environment_->getTerrainSpaceModel().vertexToState(path, *path_iter);

		// Setting the planned body pose
		body_pose.position.head(2) = path.head(2);

		// Converting the yaw angle to quaternion
		Eigen::Vector3d rpy(0,0,path(2));
		body_pose.orientation = math::getQuaternion(rpy);

		body_path.push_back(body_pose);
	}

	return true;
}

} //@namespace locomotion
} //@namespace dwl
