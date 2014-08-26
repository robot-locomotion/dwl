#include <planning/BodyPlanner.h>


namespace dwl
{

namespace planning
{

BodyPlanner::BodyPlanner() : environment_(NULL), robot_(NULL), path_solver_(NULL), pose_solver_(NULL), path_computation_time_(std::numeric_limits<double>::max()),
		pose_computation_time_(std::numeric_limits<double>::max())
{

}


BodyPlanner::~BodyPlanner()
{
	delete path_solver_;
	delete pose_solver_;
}


void BodyPlanner::reset(robot::Robot* robot, environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot properties in the contact planner \n" COLOR_RESET);
	robot_ = robot;

	printf(BLUE "Setting the environment information in the body planner\n" COLOR_RESET);
	environment_ = environment;

	path_solver_->reset(robot, environment);
	//pose_solver_->reset(environment); TODO Develop a pose solver
}


void BodyPlanner::reset(Solver* path_solver)
{
	printf(BLUE "Setting the %s path solver in the body planner\n" COLOR_RESET, path_solver->getName().c_str());
	path_solver_ = path_solver;
	path_solver_->init();
}


bool BodyPlanner::computeBodyPath(std::vector<Pose>& body_path, Pose start_pose, Pose goal_pose)
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
	Eigen::Vector3d test;
	environment_->getTerrainSpaceModel().vertexToState(test, goal_vertex);
	std::cout << "Goal = " << goal_state(0) << " " << goal_state(1) << " " << goal_state(2) << " " << test(0) << " " << test(1) << " " << test(2) << std::endl;

	// Computing the body path using a graph searching algorithm
	if (!path_solver_->compute(start_vertex, goal_vertex, path_computation_time_))
		return false;

	// Getting the shortest path
	std::list<Vertex> shortest_path = path_solver_->getShortestPath(start_vertex, goal_vertex);
	std::cout << "Size of path = " << shortest_path.size() << std::endl; //TODO Delete this message

	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_pose;
		body_pose.position = Eigen::Vector3d::Zero();
		body_pose.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector3d path;
		environment_->getTerrainSpaceModel().vertexToState(path, *path_iter);

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


void BodyPlanner::setComputationTime(double computation_time, bool path_solver)
{
	if (path_solver) {
		printf("Setting the allowed computation time of the body path solver to %f \n", computation_time);
		path_computation_time_ = computation_time;
	}
	else {
		printf("Setting the allowed computation time of the body pose solver to %f \n", computation_time);
		pose_computation_time_ = computation_time;
	}
}

} //@namespace planning
} //@namespace dwl
