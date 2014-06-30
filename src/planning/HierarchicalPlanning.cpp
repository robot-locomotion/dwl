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
	printf("Initialized the HierarchicalPlanning algortihm\n");

	solver_->init();

	return true;
}


void HierarchicalPlanning::resetGoal(Pose goal)
{
	// Converting the start and goal position to vertex ids
/*	initial_pose_ = start;
	goal_pose_ =  goal;*/

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

		// Converting quaternion to roll, pitch and yaw angles
		double roll, pitch, yaw;
		Orientation orientation(current_pose.orientation);
		orientation.getRPY(roll, pitch, yaw);

		// Computing the body path using a graph searching algorithm
		if (solver_->compute(current_vertex, goal_vertex_, yaw))
			recordBodyPath().swap(body_path_);
	} else {
		printf(YELLOW "Could not compute a locomotion plan because is not terrain information\n" COLOR_RESET);
		return false;
	}


	return true;
}


std::vector<Pose> HierarchicalPlanning::recordBodyPath()
{
	std::vector<Pose> body_path;

	std::list<Vertex> shortest_path = solver_->getShortestPath(goal_vertex_);

	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_pose;
		body_pose.position = Eigen::Vector3d::Zero();
		body_pose.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector2d path = environment_->getGridModel().vertexToCoord(*path_iter);
		body_pose.position.head(2) = path;

		body_path.push_back(body_pose);
	}

	return body_path;
}


} //@namespace planning
} //@namespace dwl
