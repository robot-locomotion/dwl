#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

AdjacencyEnvironment::AdjacencyEnvironment() : average_cost_(0), gridmap_(0.04, 0.02), is_there_terrain_information_(false)
{

}


AdjacencyEnvironment::~AdjacencyEnvironment()
{

}


void AdjacencyEnvironment::setEnvironmentInformation(std::vector<Cell> reward_map)
{
	// Cleaning old information
	std::map<Vertex, Weight> empty_terrain_cost_per_vertex;
	terrain_cost_map_.swap(empty_terrain_cost_per_vertex);
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	unsigned int vertex_id;
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		terrain_cost_map_[vertex_id] = - reward_map[i].reward;
		average_cost_ += - reward_map[i].reward;
	}
	average_cost_ /= reward_map.size();

	is_there_terrain_information_ = true;
}


void AdjacencyEnvironment::checkStartAndGoalVertex(AdjacencyMap& adjacency_map, Vertex source, Vertex target)
{
	// Checking if the start and goal vertex are part of the adjacency map
	bool is_there_start_vertex, is_there_goal_vertex = false;
	std::vector<Vertex> vertex_map;
	for (AdjacencyMap::iterator vertex_iter = adjacency_map.begin();
			vertex_iter != adjacency_map.end();
			vertex_iter++)
	{
		if (source == vertex_iter->first)
			is_there_start_vertex = true;

		if (target == vertex_iter->first)
			is_there_goal_vertex = true;

		vertex_map.push_back(vertex_iter->first);
	}

	// Start and goal position
	Eigen::Vector2d start_position, goal_position;
	start_position = gridmap_.vertexToCoord(source);
	goal_position = gridmap_.vertexToCoord(target);


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
		adjacency_map[source].push_back(Edge(start_closest_vertex, 0.0));
		adjacency_map[start_closest_vertex].push_back(Edge(source, 0.0));
		adjacency_map[target].push_back(Edge(goal_closest_vertex, 0.0));
		adjacency_map[goal_closest_vertex].push_back(Edge(target, 0.0));

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
		adjacency_map[source].push_back(Edge(start_closest_vertex, 0.0));
		adjacency_map[start_closest_vertex].push_back(Edge(source, 0.0));

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
		adjacency_map[target].push_back(Edge(goal_closest_vertex, 0.0));
		adjacency_map[goal_closest_vertex].push_back(Edge(target, 0.0));
	}
}


std::string AdjacencyEnvironment::getName()
{
	return name_;
}

} //@namespace environment

} //@namespace dwl
