#include <planning/BodyPlanner.h>


namespace dwl
{

namespace planning
{

BodyPlanner::BodyPlanner() : environment_(NULL), path_solver_(NULL), pose_solver_(NULL)
{

}


BodyPlanner::~BodyPlanner()
{
	delete path_solver_;//, pose_solver_;
}


void BodyPlanner::reset(environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the environment information in the body planner\n" COLOR_RESET);
	path_solver_->reset(environment);
	//pose_solver_->reset(environment); TODO
	environment_ = environment;
}


void BodyPlanner::reset(Solver* path_solver)
{
	printf(BLUE "Setting the %s path solver in the body planner\n" COLOR_RESET, path_solver->getName().c_str());
	path_solver_ = path_solver;
	path_solver_->init();
}


bool BodyPlanner::computeBodyPath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose)
{
	// Converting pose to vertex
	Vertex start_vertex = environment_->getGridModel().coordToVertex((Eigen::Vector2d) start_pose.position.head(2));
	Vertex goal_vertex = environment_->getGridModel().coordToVertex((Eigen::Vector2d) goal_pose.position.head(2));

	// Setting the current pose to solver
	path_solver_->setCurrentPose(start_pose);

	// Computing the body path using a graph searching algorithm
	double computation_time = 0.55;
	if (!path_solver_->compute(start_vertex, goal_vertex, computation_time))
		return false;

	// Getting the shortest path
	std::list<Vertex> shortest_path = path_solver_->getShortestPath(start_vertex, goal_vertex);
	std::cout << "Size of path = " << shortest_path.size() << std::endl; //TODO

	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	Eigen::Vector2d old_point = Eigen::Vector2d::Zero();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_pose;
		body_pose.position = Eigen::Vector3d::Zero();
		body_pose.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector2d path = environment_->getGridModel().vertexToCoord(*path_iter);
		body_pose.position.head(2) = path;

		// Computing the roll, pitch and yaw angles
		double roll = 0;
		double pitch = 0;
		double yaw = atan2((double) (path(1) - old_point(1)), (double) (path(0) - old_point(0)));

		// Converting rpy angles to quaternion
		Orientation orientation(roll, pitch, yaw);
		orientation.getQuaternion(body_pose.orientation);

		body_path.push_back(body_pose);
		old_point = path;
	}

	return true;
}

} //@namespace planning
} //@namespace dwl
