#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{


AdjacencyEnvironment::AdjacencyEnvironment() : average_cost_(0), gridmap_(0.04, 0.02), is_there_terrain_information_(false), is_lattice_(false)
{

}


AdjacencyEnvironment::~AdjacencyEnvironment()
{

}


void AdjacencyEnvironment::setEnvironmentInformation(std::vector<Cell> reward_map)
{
	// Cleaning the old information
	std::map<Vertex, Weight> empty_terrain_cost_per_vertex;
	terrain_cost_map_.swap(empty_terrain_cost_per_vertex);
	average_cost_ = 0;

	// Storing the cost-map data according the vertex id
	unsigned int vertex_id;
	double resolution = std::numeric_limits<double>::max();
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		terrain_cost_map_[vertex_id] = - reward_map[i].reward;
		average_cost_ += - reward_map[i].reward;
		if (reward_map[i].size < resolution) {
			resolution = reward_map[i].size;
		}
	}
	average_cost_ /= reward_map.size();

	// Setting the resolution of the environment
	setResolution(resolution, true);
	setResolution(resolution, false);

	is_there_terrain_information_ = true;
}


void AdjacencyEnvironment::setResolution(double resolution, bool gridmap)
{
	gridmap_.setResolution(resolution, gridmap);
}


void AdjacencyEnvironment::computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target, double orientation)//Eigen::Vector3d position)
{
	printf(YELLOW "Could compute the whole adjacency map because it was not defined an adjacency model\n" COLOR_RESET);
}


void AdjacencyEnvironment::getSuccessors(std::list<Edge>& successors, Vertex vertex, double orientation)
{
	printf(YELLOW "Could get the successors because it was not defined an adjacency model\n" COLOR_RESET);
}


void AdjacencyEnvironment::getTheClosestStartAndGoalVertex(Vertex& closest_source, Vertex& closest_target, Vertex source, Vertex target)
{
	// Checking if the start and goal vertex are part of the terrain information
	bool is_there_start_vertex, is_there_goal_vertex = false;
	std::vector<Vertex> vertex_map;
	if (is_there_terrain_information_) {
		for (CostMap::iterator vertex_iter = terrain_cost_map_.begin();
				vertex_iter != terrain_cost_map_.end();
				vertex_iter++)
		{
			Vertex current_vertex = vertex_iter->first;
			if (source == current_vertex) {
				is_there_start_vertex = true;
				closest_source = current_vertex;
			}

			if (target == current_vertex) {
				is_there_goal_vertex = true;
				closest_target = current_vertex;
			}

			if ((is_there_start_vertex) && (is_there_goal_vertex))
				return;

			vertex_map.push_back(vertex_iter->first);
		}
	} else {
		printf(RED "Couldn't get the closest start and goal vertex because there isn't terrain information \n" COLOR_RESET);
		return;
	}

	// Start and goal position
	Eigen::Vector2d start_position, goal_position;
	start_position = gridmap_.vertexToCoord(source);
	goal_position = gridmap_.vertexToCoord(target);

	double closest_source_distant = std::numeric_limits<double>::max();
	double closest_target_distant = std::numeric_limits<double>::max();
	Vertex start_closest_vertex, goal_closest_vertex;
	if ((!is_there_start_vertex) && (!is_there_goal_vertex)) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from start and goal positions
			double start_distant = (start_position - vertex_position).norm();
			double goal_distant = (goal_position - vertex_position).norm();

			// Recording the closest vertex from the start position
			if (start_distant < closest_source_distant) {
				start_closest_vertex = vertex_map[i];
				closest_source_distant = start_distant;
			}

			// Recording the closest vertex from the goal position
			if (goal_distant < closest_target_distant) {
				goal_closest_vertex = vertex_map[i];
				closest_target_distant = goal_distant;
			}
		}

		// Adding the goal to the adjacency map
		closest_source = start_closest_vertex;
		closest_target = goal_closest_vertex;

	} else if (!is_there_start_vertex) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from start position
			double start_distant = (start_position - vertex_position).norm();

			// Recording the closest vertex from the start position
			if (start_distant < closest_source_distant) {
				start_closest_vertex = vertex_map[i];
				closest_source_distant = start_distant;
			}
		}

		// Adding the start to the adjacency map
		closest_source = start_closest_vertex;

	} else if (!is_there_goal_vertex) {
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

			// Calculating the distant to the vertex from goal position
			double goal_distant = (goal_position - vertex_position).norm();

			// Recording the closest vertex from the goal position
			if (goal_distant < closest_target_distant) {
				goal_closest_vertex = vertex_map[i];
				closest_target_distant = goal_distant;
			}
		}

		// Adding the goal to the adjacency map
		closest_target = goal_closest_vertex;
	}
}


void AdjacencyEnvironment::getTheClosestVertex(Vertex& closest_vertex, Vertex vertex)
{
	// Checking if the  vertex is part of the terrain information
	bool is_there_vertex = false;
	std::vector<Vertex> vertex_map;
	if (is_there_terrain_information_) {
		for (CostMap::iterator vertex_iter = terrain_cost_map_.begin();
				vertex_iter != terrain_cost_map_.end();
				vertex_iter++)
		{
			Vertex current_vertex = vertex_iter->first;
			if (vertex == current_vertex) {
				is_there_vertex = true;
				closest_vertex = current_vertex;

				return;
			}

			vertex_map.push_back(vertex_iter->first);
		}
	} else {
		printf(RED "Couldn't get the closest start and goal vertex because there isn't terrain information \n" COLOR_RESET);
		return;
	}

	// Start and goal position
	Eigen::Vector2d vertex_position;
	vertex_position = gridmap_.vertexToCoord(vertex);

	double closest_distant = std::numeric_limits<double>::max();
	for (int i = 0; i < vertex_map.size(); i++) {
		// Calculating the vertex position
		Eigen::Vector2d current_vertex_position = gridmap_.vertexToCoord(vertex_map[i]);

		// Calculating the distant to the current vertex
		double start_distant = (vertex_position - current_vertex_position).norm();

		// Recording the closest vertex from the start position
		if (start_distant < closest_distant) {
			closest_vertex = vertex_map[i];
			closest_distant = start_distant;
		}
	}
}


double AdjacencyEnvironment::heuristicCostEstimate(Vertex source, Vertex target) //TODO
{
	Eigen::Vector2d source_position = getPosition(source);
	Eigen::Vector2d target_position = getPosition(target);

	double distance = (target_position - source_position).squaredNorm();
	double dy = target_position(1) - source_position(1);
	//double dx = target_position(0) - source_position(0);
	//double heading = abs(atan(dy / dx));

	return 0.5 * (0.8 * distance + 0.2 * abs(dy));
}


bool AdjacencyEnvironment::isReachedGoal(Vertex target, Vertex current)
{
	if (isLatticeRepresentation()) {
		double epsilon = 0.1; //TODO
		Eigen::Vector2d current_position = getPosition(current);
		Eigen::Vector2d target_position = getPosition(target);
		double distant = (target_position - current_position).squaredNorm();
		if (distant < epsilon) {
			// Reconstructing path
			std::cout << "Reached goal = " << current_position(0) << " " << current_position(1) << " | Goal = " << target_position(0) << " " << target_position(1) << " | dist = " << distant << std::endl;
			return true;
		}
	} else {
		if (current == target) {
			// Reconstructing path
			return true;
		}
	}

	return false;
}


bool AdjacencyEnvironment::isLatticeRepresentation()
{
	return is_lattice_;
}

Eigen::Vector2d AdjacencyEnvironment::getPosition(Vertex vertex)
{
	return gridmap_.vertexToCoord(vertex);
}


Vertex AdjacencyEnvironment::getVertex(Pose pose)
{
	return gridmap_.coordToVertex((Eigen::Vector2d) pose.position.head(2));
}


std::string AdjacencyEnvironment::getName()
{
	return name_;
}

} //@namespace environment

} //@namespace dwl
