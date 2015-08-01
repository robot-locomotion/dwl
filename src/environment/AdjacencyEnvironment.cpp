#include <environment/AdjacencyEnvironment.h>


namespace dwl
{

namespace environment
{

AdjacencyModel::AdjacencyModel() :	robot_(NULL), environment_(NULL), is_lattice_(false),
		is_added_feature_(false), uncertainty_factor_(1.15)
{

}


AdjacencyModel::~AdjacencyModel()
{

}


void AdjacencyModel::reset(robot::Robot* robot,
						   environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot information in the %s adjacency model \n" COLOR_RESET,
			name_.c_str());
	robot_ = robot;

	printf(BLUE "Setting the environment information in the %s adjacency model \n" COLOR_RESET,
			name_.c_str());
	environment_ = environment;

	for (int i = 0; i < (int) features_.size(); i++)
		features_[i]->reset(robot);
}


void AdjacencyModel::computeAdjacencyMap(AdjacencyMap& adjacency_map,
										 Vertex source,
										 Vertex target)
{
	printf(YELLOW "Could compute the whole adjacency map because it was not defined an adjacency "
			"model\n" COLOR_RESET);
}


void AdjacencyModel::getTheClosestStartAndGoalVertex(Vertex& closest_source,
													 Vertex& closest_target,
													 Vertex source,
													 Vertex target)
{
	// Checking if the start and goal vertex are part of the terrain information
	bool is_there_start_vertex, is_there_goal_vertex = false;
	std::vector<Vertex> vertex_map;
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (CostMap::iterator vertex_iter = terrain_costmap.begin();
				vertex_iter != terrain_costmap.end(); vertex_iter++) {
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
		printf(RED "Could not get the closest start and goal vertex because there is not terrain "
				"information \n" COLOR_RESET);
		return;
	}

	// Start and goal state
	Eigen::Vector3d start_state, goal_state;
	environment_->getTerrainSpaceModel().vertexToState(start_state, source);
	environment_->getTerrainSpaceModel().vertexToState(goal_state, target);

	double closest_source_distant = std::numeric_limits<double>::max();
	double closest_target_distant = std::numeric_limits<double>::max();
	Vertex start_closest_vertex = 0, goal_closest_vertex = 0;
	if ((!is_there_start_vertex) && (!is_there_goal_vertex)) {
		Eigen::Vector3d current_state;
		for (unsigned int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getTerrainSpaceModel().vertexToState(current_state, vertex_map[i]);

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
		for (unsigned int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getTerrainSpaceModel().vertexToState(current_state, vertex_map[i]);

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
		for (unsigned int i = 0; i < vertex_map.size(); i++) {
			// Calculating the vertex position
			environment_->getTerrainSpaceModel().vertexToState(current_state, vertex_map[i]);

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
}


void AdjacencyModel::getTheClosestVertex(Vertex& closest_vertex,
										 Vertex vertex)
{
	// Checking if the  vertex is part of the terrain information
	std::vector<Vertex> vertex_map;
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (CostMap::iterator vertex_iter = terrain_costmap.begin();
				vertex_iter != terrain_costmap.end(); vertex_iter++) {
			Vertex current_vertex = vertex_iter->first;
			if (vertex == current_vertex) {
				closest_vertex = current_vertex;

				return;
			}

			vertex_map.push_back(vertex_iter->first);
		}
	} else {
		printf(RED "Could not get the closest start and goal vertex because there is not terrain "
				"information \n" COLOR_RESET);
		return;
	}

	// State
	Eigen::Vector3d state_vertex;
	environment_->getTerrainSpaceModel().vertexToState(state_vertex, vertex);

	double closest_distant = std::numeric_limits<double>::max();
	Eigen::Vector3d current_state_vertex;
	for (unsigned int i = 0; i < vertex_map.size(); i++) {
		// Calculating the vertex position
		environment_->getTerrainSpaceModel().vertexToState(current_state_vertex,
				vertex_map[i]);

		// Calculating the distant to the current vertex
		double start_distant = (state_vertex.head(2)
				- current_state_vertex.head(2)).norm();

		// Recording the closest vertex from the start position
		if (start_distant < closest_distant) {
			closest_vertex = vertex_map[i];
			closest_distant = start_distant;
		}
	}
}


double AdjacencyModel::heuristicCost(Vertex source,
									 Vertex target)
{
	Eigen::Vector3d source_state, target_state;
	environment_->getTerrainSpaceModel().vertexToState(source_state, source);
	environment_->getTerrainSpaceModel().vertexToState(target_state, target);

	// Normalizing the angles for a range of [-pi,pi]
	double current_angle = source_state(2), target_angle = target_state(2);
	math::normalizeAngle(current_angle, MinusPiToPi);
	math::normalizeAngle(target_angle, MinusPiToPi);
	source_state(2) = current_angle;
	target_state(2) = target_angle;

	// Computing the distance
	double distance = (target_state.head(2) - source_state.head(2)).norm();
	double dist_orientation = sqrt(
			pow(((double) target_state(2) - (double) source_state(2)), 2));

	double heuristic = (5 * distance + 2.5 * dist_orientation)
			* uncertainty_factor_ * (environment_->getAverageCostOfTerrain() + 0.1);

	return heuristic;
}


bool AdjacencyModel::isReachedGoal(Vertex target,
								   Vertex current)
{
	if (isLatticeRepresentation()) {
		double min_error = 0.1;

		Eigen::Vector3d current_state, target_state;
		environment_->getTerrainSpaceModel().vertexToState(current_state, current);
		environment_->getTerrainSpaceModel().vertexToState(target_state, target);

		// Normalizing the angles for a range of [-pi,pi]
		double current_angle = current_state(2), target_angle = target_state(2);
		math::normalizeAngle(current_angle, MinusPiToPi);
		math::normalizeAngle(target_angle, MinusPiToPi);
		current_state(2) = current_angle;
		target_state(2) = target_angle;

		double distant = ((Eigen::Vector2d) (target_state.head(2) - current_state.head(2))).norm();
		if (distant < min_error) {
			double angular_error = (double) target_state(2) - (double) current_state(2);

			if (std::abs(angular_error) < min_error) {
				// Reconstructing path
				return true;
			}
		}
	} else {
		if (current == target) {
			// Reconstructing path
			return true;
		}
	}

	return false;
}


bool AdjacencyModel::isFreeOfObstacle(Vertex state_vertex,
									  TypeOfState state_representation,
									  bool body)
{
	return true;
}


void AdjacencyModel::addFeature(Feature* feature)
{
	double weight;
	feature->getWeight(weight);
	printf(GREEN "Adding the %s feature with a weight of %f\n" COLOR_RESET,
			feature->getName().c_str(), weight);
	features_.push_back(feature);
	is_added_feature_ = true;
}


bool AdjacencyModel::isLatticeRepresentation()
{
	return is_lattice_;
}


std::string AdjacencyModel::getName()
{
	return name_;
}

} //@namespace environment
} //@namespace dwl
