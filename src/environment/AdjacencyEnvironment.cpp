#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

AdjacencyEnvironment::AdjacencyEnvironment() : environment_(NULL), is_lattice_(false), uncertainty_factor_(1.15)
{

}


AdjacencyEnvironment::~AdjacencyEnvironment()
{
	//delete environment_;
}


void AdjacencyEnvironment::reset(EnvironmentInformation* environment)
{
	environment_ = environment;
}


void AdjacencyEnvironment::setCurrentPose(Pose current_pose)
{
	current_pose_ = current_pose;
}


void AdjacencyEnvironment::computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target)
{
	printf(YELLOW "Could compute the whole adjacency map because it was not defined an adjacency model\n" COLOR_RESET);
}


void AdjacencyEnvironment::getSuccessors(std::list<Edge>& successors, Vertex vertex)
{
	printf(YELLOW "Could get the successors because it was not defined an adjacency model\n" COLOR_RESET);
}


void AdjacencyEnvironment::getTheClosestStartAndGoalVertex(Vertex& closest_source, Vertex& closest_target, Vertex source, Vertex target)
{
	// Checking if the start and goal vertex are part of the terrain information
	bool is_there_start_vertex, is_there_goal_vertex = false;
	std::vector<Vertex> vertex_map;
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (CostMap::iterator vertex_iter = terrain_costmap.begin();
				vertex_iter != terrain_costmap.end();
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
	Eigen::Vector3d start_state, goal_state;
	environment_->getSpaceModel().vertexToState(start_state, source);
	environment_->getSpaceModel().vertexToCoord(goal_state, target);

	double closest_source_distant = std::numeric_limits<double>::max();
	double closest_target_distant = std::numeric_limits<double>::max();
	Vertex start_closest_vertex, goal_closest_vertex;
	if ((!is_there_start_vertex) && (!is_there_goal_vertex)) {
		Eigen::Vector3d current_state;
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getSpaceModel().vertexToState(current_state, vertex_map[i]);

			// Calculating the distant to the vertex from start and goal positions
			double start_distant = (start_state.head(2) - current_state.head(2)).norm();
			double goal_distant = (goal_state.head(2) - current_state.head(2)).norm();

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
		Eigen::Vector3d current_state;
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getSpaceModel().vertexToState(current_state, vertex_map[i]);

			// Calculating the distant to the vertex from start position
			double start_distant = (start_state.head(2) - current_state.head(2)).norm();

			// Recording the closest vertex from the start position
			if (start_distant < closest_source_distant) {
				start_closest_vertex = vertex_map[i];
				closest_source_distant = start_distant;
			}
		}

		// Adding the start to the adjacency map
		closest_source = start_closest_vertex;

	} else if (!is_there_goal_vertex) {
		Eigen::Vector3d current_state;
		for (int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getSpaceModel().vertexToState(current_state, vertex_map[i]);

			// Calculating the distant to the vertex from goal position
			double goal_distant = (goal_state.head(2) - current_state.head(2)).norm();

			// Recording the closest vertex from the goal position
			if (goal_distant < closest_target_distant) {
				goal_closest_vertex = vertex_map[i];
				closest_target_distant = goal_distant;
			}
		}

		// Adding the goal to the adjacency map
		closest_target = goal_closest_vertex;
	}

	//TODO
/*	std::cout << "Closest source point = " << environment_->getGridModel().vertexToCoord(closest_source) << std::endl;
	std::cout << "Closest target point = " << environment_->getGridModel().vertexToCoord(closest_target) << std::endl;*/
}


void AdjacencyEnvironment::getTheClosestVertex(Vertex& closest_vertex, Vertex vertex)
{
	// Checking if the  vertex is part of the terrain information
	bool is_there_vertex = false;
	std::vector<Vertex> vertex_map;
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (CostMap::iterator vertex_iter = terrain_costmap.begin();
				vertex_iter != terrain_costmap.end();
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
		printf(RED "Could not get the closest start and goal vertex because there is not terrain information \n" COLOR_RESET);
		return;
	}

	// Start and goal position
	Eigen::Vector3d state_vertex;
	environment_->getSpaceModel().vertexToState(state_vertex, vertex);

	double closest_distant = std::numeric_limits<double>::max();
	Eigen::Vector3d current_state_vertex;
	for (int i = 0; i < vertex_map.size(); i++) {
		// Calculating the vertex position
		environment_->getSpaceModel().vertexToState(current_state_vertex, vertex_map[i]);//TODO

		// Calculating the distant to the current vertex
		double start_distant = (state_vertex.head(2) - current_state_vertex.head(2)).norm();

		// Recording the closest vertex from the start position
		if (start_distant < closest_distant) {
			closest_vertex = vertex_map[i];
			closest_distant = start_distant;
		}
	}
}


double AdjacencyEnvironment::heuristicCostEstimate(Vertex source, Vertex target) //TODO
{
	Eigen::Vector3d source_position = getPosition(source);
	Eigen::Vector3d target_position = getPosition(target);

	double distance = (target_position - source_position).squaredNorm();
	double dy = target_position(1) - source_position(1);
	//double dx = target_position(0) - source_position(0);
	//double heading = abs(atan(dy / dx));

	double heuristic = 2.5 * distance * uncertainty_factor_ * environment_->getAverageCostOfTerrain(); //TODO

	return heuristic;
}


bool AdjacencyEnvironment::isReachedGoal(Vertex target, Vertex current)
{
	if (isLatticeRepresentation()) {
		double epsilon = 0.1; //TODO

		//TODO
		Eigen::Vector3d current_position = getPosition(current);
		Eigen::Vector3d target_position = getPosition(target);
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

//TODO
/*
Eigen::Vector3d AdjacencyEnvironment::getPosition(Vertex vertex)
{
	return environment_->getGridModel().vertexToCoord(vertex);
}


Vertex AdjacencyEnvironment::getVertex(Pose pose)
{
	return environment_->getGridModel().stateToVertex((Eigen::Vector2d) pose.position.head(2));//TODO
}


Pose AdjacencyEnvironment::getPose(Vertex vertex)
{
	Pose pose;
	pose.position.head(2) = environment_->getGridModel().vertexToCoord(vertex);
	return pose;
}


const std::map<Vertex, double>& AdjacencyEnvironment::getOrientations() const
{
	return orientations_;
}*/


std::string AdjacencyEnvironment::getName()
{
	return name_;
}

} //@namespace environment
} //@namespace dwl
