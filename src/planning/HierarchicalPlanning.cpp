#include <planning/HierarchicalPlanning.h>
#include <utils/Orientation.h>


namespace dwl
{

namespace planning
{


HierarchicalPlanning::HierarchicalPlanning() : start_id_(0), goal_id_(0)
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


void HierarchicalPlanning::update(Pose start, Pose goal)
{
	// Converting the start and goal position to vertex ids
	Key start_key, goal_key;

	start_id_ = gridmap_.coordToVertex((Eigen::Vector2d) start.position.head(2));
	goal_id_ = gridmap_.coordToVertex((Eigen::Vector2d) goal.position.head(2));
}


bool HierarchicalPlanning::compute(Pose robot_state)
{
	// Cleaning global variables
	std::vector<Pose> empty_body_trajectory;
	body_trajectory_.swap(empty_body_trajectory);

	// Computing a body path
	SolverInterface solver;
	solver.searcher.source = start_id_;
	solver.searcher.target = goal_id_;

	// Converting quaternion to roll, pitch and yaw angles
	double roll, pitch, yaw;
	Orientation orientation(robot_state.orientation);
	orientation.getRPY(roll, pitch, yaw);

	// Setting the 3d position of the robot
	solver.searcher.position(0) = robot_state.position(0);
	solver.searcher.position(1) = robot_state.position(1);
	solver.searcher.position(2) = yaw;

	// Computing the body path using a graph searching algorithm
	solver_->compute(solver);
	std::list<Vertex> shortest_path = solver_->getShortestPath(goal_id_);






	// Recording the body path
	std::list<Vertex>::iterator path_iter = shortest_path.begin();
	for(; path_iter != shortest_path.end(); path_iter++) {
		Pose body_path;
		body_path.position = Eigen::Vector3d::Zero();
		body_path.orientation = Eigen::Vector4d::Zero();

		Eigen::Vector2d path = gridmap_.vertexToCoord(*path_iter);
		body_path.position.head(2) = path;

		body_trajectory_.push_back(body_path);
	}


	return true;
}


void HierarchicalPlanning::checkStartAndGoalVertex(AdjacencyMap& adjacency_map)
{
	Key key;

	// Checking if the start and goal vertex are part of the adjacency map
	bool is_there_start_vertex, is_there_goal_vertex = false;
	std::vector<Vertex> vertex_map;
	for (AdjacencyMap::iterator vertex_iter = adjacency_map.begin();
			vertex_iter != adjacency_map.end();
			vertex_iter++)
	{
		if (start_id_ == vertex_iter->first)
			is_there_start_vertex = true;

		if (goal_id_ == vertex_iter->first)
			is_there_goal_vertex = true;

		vertex_map.push_back(vertex_iter->first);
	}

	// Start and goal position
	Eigen::Vector2d start_position, goal_position;
	start_position = gridmap_.vertexToCoord(start_id_);
	goal_position = gridmap_.vertexToCoord(goal_id_);


	double start_closest_distant = std::numeric_limits<double>::max();
	double goal_closest_distant = std::numeric_limits<double>::max();
	Vertex start_closest_vertex, goal_closest_vertex;
	if ((!is_there_start_vertex) && (!is_there_goal_vertex)) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from start and goal positions
			double start_distant = (start_position - vertex_position).norm();
			double goal_distant = (goal_position - vertex_position).norm();

			// Recording the closest vertex from the start position
			if (start_distant < start_closest_distant) {
				start_closest_vertex = vertex_map[i];
				start_closest_distant = start_distant;
			}

			// Recording the closest vertex from the goal position
			if (goal_distant < goal_closest_distant) {
				goal_closest_vertex = vertex_map[i];
				goal_closest_distant = goal_distant;
			}
		}

		// Adding the goal to the adjacency map
		adjacency_map[start_id_].push_back(Edge(start_closest_vertex, 0.0));
		adjacency_map[start_closest_vertex].push_back(Edge(start_id_, 0.0));
		adjacency_map[goal_id_].push_back(Edge(goal_closest_vertex, 0.0));
		adjacency_map[goal_closest_vertex].push_back(Edge(goal_id_, 0.0));

	} else if (!is_there_start_vertex) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from start position
			double start_distant = (start_position - vertex_position).norm();

			// Recording the closest vertex from the start position
			if (start_distant < start_closest_distant) {
				start_closest_vertex = vertex_map[i];
				start_closest_distant = start_distant;
			}
		}

		// Adding the start to the adjacency map
		adjacency_map[start_id_].push_back(Edge(start_closest_vertex, 0.0));
		adjacency_map[start_closest_vertex].push_back(Edge(start_id_, 0.0));

	} else if (!is_there_goal_vertex) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from goal position
			double goal_distant = (goal_position - vertex_position).norm();

			// Recording the closest vertex from the goal position
			if (goal_distant < goal_closest_distant) {
				goal_closest_vertex = vertex_map[i];
				goal_closest_distant = goal_distant;
			}
		}

		// Adding the goal to the adjacency map
		adjacency_map[goal_id_].push_back(Edge(goal_closest_vertex, 0.0));
		adjacency_map[goal_closest_vertex].push_back(Edge(goal_id_, 0.0));
	}

}

} //@namespace planning

} //@namespace dwl
