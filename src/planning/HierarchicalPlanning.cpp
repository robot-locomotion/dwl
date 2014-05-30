#include <planning/HierarchicalPlanning.h>


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


void HierarchicalPlanning::update(BodyPose start, BodyPose goal)
{
	// Converting the start and goal position to vertex ids
	dwl::environment::Key start_key, goal_key;

	start_id_ = gridmap_.coordToVertex((Eigen::Vector2d) start.position.head(2));
	goal_id_ = gridmap_.coordToVertex((Eigen::Vector2d) goal.position.head(2));
}


bool HierarchicalPlanning::compute()
{
	AdjacencyMap adjacency_map;

	// Getting the adjacency map
	bool cost_map = false;
	Eigen::VectorXd state;
	for (int i = 0; i < costs_.size(); i++) {
		if (costs_[i]->isCostMap()) {
			costs_[i]->get(adjacency_map);

			cost_map = true;
			break;
		}
	}
	if (!cost_map) {
		printf(RED "Could not computed the Dijkstrap algorithm because it was not defined a cost map (adjacency map)\n" COLOR_RESET);
		return false;
	}

	// Check if the start and goal position belong to the adjacency map, in negative case, these are added to the adjacency map
	checkStartAndGoalVertex(adjacency_map);


	SolverInterface solver;
	solver.searcher.source = start_id_;
	solver.searcher.target = goal_id_;
	solver.searcher.adjacency_map = adjacency_map;


	// computing a body path
	solver_->compute(solver);
	std::list<Vertex> path = solver_->getShortestPath(goal_id_);





	//TODO only for debugging
	Eigen::Vector2d start_position, goal_position;
	start_position = gridmap_.vertexToCoord(start_id_);
	goal_position = gridmap_.vertexToCoord(goal_id_);
	std::cout << "Total cost to [" << goal_position(0) << " " << goal_position(1) << "] from [" << start_position(0);
	std::cout << " " << (double) start_position(1) << "]: " << solver_->getMinimumCost() << std::endl;

	//TODO Some line for debugging
	std::list<Vertex>::iterator path_iter = path.begin();
	std::cout << "Path: ";
	for( ; path_iter != path.end(); path_iter++) {
		Eigen::Vector3d body_path = Eigen::Vector3d::Zero();

		Eigen::Vector2d path = gridmap_.vertexToCoord(*path_iter);
		body_path(0) = path(0);
		body_path(1) = path(1);
		body_path(2) = 0;

		body_path_.push_back(body_path);

		std::cout << "[" << path(0) << " " << path(1) << "] | ";
	}
	std::cout << std::endl;


	return true;
}


void HierarchicalPlanning::checkStartAndGoalVertex(AdjacencyMap& adjacency_map)
{
	dwl::environment::Key key;

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
