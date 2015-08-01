#include <environment/GridBasedBodyAdjacency.h>


namespace dwl
{

namespace environment
{

GridBasedBodyAdjacency::GridBasedBodyAdjacency() : is_stance_adjacency_(true),
		neighboring_definition_(3), number_top_reward_(5), uncertainty_factor_(1.15)
{
	name_ = "Grid-based Body";
	is_lattice_ = false;
}


GridBasedBodyAdjacency::~GridBasedBodyAdjacency()
{

}


void GridBasedBodyAdjacency::computeAdjacencyMap(AdjacencyMap& adjacency_map,
												 Vertex source, Vertex target)
{
	// Computing a default stance areas
	Eigen::Vector3d full_action = Eigen::Vector3d::Zero();
	stance_areas_ = robot_->getFootstepSearchAreas(full_action);

	// Getting the body orientation
	Eigen::Vector3d initial_state;
	environment_->getTerrainSpaceModel().vertexToState(initial_state, source);
	unsigned short int key_yaw;
	environment_->getTerrainSpaceModel().stateToKey(key_yaw, (double) initial_state(2), false);
	double yaw;
	environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);

	if (environment_->isTerrainInformation()) {
		// Adding the source and target vertex if it is outside the information terrain
		Vertex closest_source, closest_target;
		getTheClosestStartAndGoalVertex(closest_source, closest_target, source, target);
		if (closest_source != source) {
			adjacency_map[source].push_back(Edge(closest_source, 0));
		}
		if (closest_target != target) {
			adjacency_map[closest_target].push_back(Edge(target, 0));
		}

		// Computing the adjacency map given the terrain information
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (CostMap::iterator vertex_iter = terrain_costmap.begin();
				vertex_iter != terrain_costmap.end();
				vertex_iter++)
		{
			Vertex vertex = vertex_iter->first;
			Eigen::Vector2d current_coord;

			Vertex state_vertex;
			environment_->getTerrainSpaceModel().vertexToCoord(current_coord, vertex);
			Eigen::Vector3d current_state;
			current_state << current_coord, yaw;
			environment_->getTerrainSpaceModel().stateToVertex(state_vertex, current_state);

			if (!isStanceAdjacency()) {
				double terrain_cost = vertex_iter->second;

				// Searching the neighbor actions
				std::vector<Vertex> neighbor_actions;
				searchNeighbors(neighbor_actions, state_vertex);
				for (unsigned int i = 0; i < neighbor_actions.size(); i++)
					adjacency_map[neighbor_actions[i]].push_back(Edge(state_vertex, terrain_cost));
			} else {
				// Computing the body cost
				double body_cost;
				computeBodyCost(body_cost, state_vertex);

				// Searching the neighbor actions
				std::vector<Vertex> neighbor_actions;
				searchNeighbors(neighbor_actions, state_vertex);
				for (unsigned int i = 0; i < neighbor_actions.size(); i++)
					adjacency_map[neighbor_actions[i]].push_back(Edge(state_vertex, body_cost));
			}
		}
	} else
		printf(RED "Could not computed the adjacency map because there is not terrain information \n"
				COLOR_RESET);
}


void GridBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors,
										   Vertex state_vertex)
{
	Eigen::Vector3d state;
	environment_->getTerrainSpaceModel().vertexToState(state, state_vertex);

	std::vector<Vertex> neighbor_actions;
	searchNeighbors(neighbor_actions, state_vertex);
	if (environment_->isTerrainInformation()) {
		// Getting the terrain costmap
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);

		unsigned int action_size = neighbor_actions.size();
		for (unsigned int i = 0; i < action_size; i++) {
			// Converting the state vertex (x,y,yaw) to a terrain vertex (x,y)
			Vertex terrain_vertex;
			environment_->getTerrainSpaceModel().stateVertexToEnvironmentVertex(terrain_vertex,
					neighbor_actions[i], XY_Y);

			if (!isStanceAdjacency()) {
				double terrain_cost = terrain_costmap.find(terrain_vertex)->second;
				successors.push_back(Edge(neighbor_actions[i], terrain_cost));
			} else {
				// Computing the body cost
				double body_cost;
				computeBodyCost(body_cost, neighbor_actions[i]);
				successors.push_back(Edge(neighbor_actions[i], body_cost));
			}
		}
	} else
		printf(RED "Could not computed the successors because there is not terrain information \n"
				COLOR_RESET);
}


void GridBasedBodyAdjacency::searchNeighbors(std::vector<Vertex>& neighbor_states,
											 Vertex state_vertex)
{
	// Getting the key of yaw
	unsigned short int key_yaw;
	Eigen::Vector3d state;
	environment_->getTerrainSpaceModel().vertexToState(state, state_vertex);
	environment_->getTerrainSpaceModel().stateToKey(key_yaw, (double) state(2), false);

	// Getting the key for x and y axis
	Key terrain_key;
	Vertex terrain_vertex;
	environment_->getTerrainSpaceModel().stateVertexToEnvironmentVertex(terrain_vertex, state_vertex, XY_Y);
	environment_->getTerrainSpaceModel().vertexToKey(terrain_key, terrain_vertex, true);

	// Searching the closed neighbors around 3-neighboring area
	bool is_found_neighbor_positive_x = false, is_found_neighbor_negative_x = false;
	bool is_found_neighbor_positive_y = false, is_found_neighbor_negative_y = false;
	bool is_found_neighbor_positive_xy = false, is_found_neighbor_negative_xy = false;
	bool is_found_neighbor_positive_yx = false, is_found_neighbor_negative_yx = false;
	if (environment_->isTerrainInformation()) {
		// Getting the terrain cost map
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);

		double x, y, yaw;

		// Searching the states neighbors
		for (int r = 1; r <= neighboring_definition_; r++) {
			Key searching_key;
			Vertex neighbor_vertex, neighbor_state_vertex;

			// Searching the neighbors in the positive x-axis
			searching_key.x = terrain_key.x + r;
			searching_key.y = terrain_key.y;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_positive_x)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_positive_x = true;
			}

			// Searching the neighbor in the negative x-axis
			searching_key.x = terrain_key.x - r;
			searching_key.y = terrain_key.y;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_negative_x)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_negative_x = true;
			}

			// Searching the neighbor in the positive y-axis
			searching_key.x = terrain_key.x;
			searching_key.y = terrain_key.y + r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_positive_y)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_positive_y = true;
			}

			// Searching the neighbor in the negative y-axis
			searching_key.x = terrain_key.x;
			searching_key.y = terrain_key.y - r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_negative_y)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_negative_y = true;
			}

			// Searching the neighbor in the positive xy-axis
			searching_key.x = terrain_key.x + r;
			searching_key.y = terrain_key.y + r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_positive_xy)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_positive_xy = true;
			}

			// Searching the neighbor in the negative xy-axis
			searching_key.x = terrain_key.x - r;
			searching_key.y = terrain_key.y - r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_negative_xy)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_negative_xy = true;
			}

			// Searching the neighbor in the positive yx-axis
			searching_key.x = terrain_key.x - r;
			searching_key.y = terrain_key.y + r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_positive_yx)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_positive_yx = true;
			}

			// Searching the neighbor in the negative yx-axis
			searching_key.x = terrain_key.x + r;
			searching_key.y = terrain_key.y - r;
			environment_->getTerrainSpaceModel().keyToVertex(neighbor_vertex, searching_key, true);
			if ((terrain_costmap.count(neighbor_vertex) > 0) && (!is_found_neighbor_negative_yx)) {
				// Getting the state vertex of the neighbor
				environment_->getTerrainSpaceModel().keyToState(x, searching_key.x, true);
				environment_->getTerrainSpaceModel().keyToState(y, searching_key.y, true);
				environment_->getTerrainSpaceModel().keyToState(yaw, key_yaw, false);
				state << x, y, yaw;
				environment_->getTerrainSpaceModel().stateToVertex(neighbor_state_vertex, state);

				neighbor_states.push_back(neighbor_state_vertex);
				is_found_neighbor_negative_yx = true;
			}
		}
	} else
		printf(RED "Could not searched the neighbors because there is not terrain information \n"
				COLOR_RESET);
}


void GridBasedBodyAdjacency::computeBodyCost(double& cost,
											 Vertex state_vertex)
{
	// Converting the vertex to state (x,y,yaw)
	Eigen::Vector3d state;
	environment_->getTerrainSpaceModel().vertexToState(state, state_vertex);

	// Getting the terrain cost map
	CostMap terrain_costmap;
	environment_->getTerrainCostMap(terrain_costmap);

	// Computing the terrain cost
	double terrain_cost = 0;
	unsigned int area_size = stance_areas_.size();
	for (unsigned int n = 0; n < area_size; n++) {
		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = stance_areas_[n].min_x + state(0);
		boundary_min(1) = stance_areas_[n].min_y + state(1);
		boundary_max(0) = stance_areas_[n].max_x + state(0);
		boundary_max(1) = stance_areas_[n].max_y + state(1);

		// Computing the stance cost
		std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > stance_cost_queue;
		double stance_cost = 0;
		double resolution = stance_areas_[n].resolution;
		for (double y = boundary_min(1); y <= boundary_max(1); y += resolution ) {
			for (double x = boundary_min(0); x <= boundary_max(0); x += resolution) {
				// Computing the rotated coordinate according to the orientation of the body
				Eigen::Vector2d point_position;
				point_position(0) = (x - state(0)) * cos((double) state(2)) -
						(y - state(1)) * sin((double) state(2)) + state(0);
				point_position(1) = (x - state(0)) * sin((double) state(2)) +
						(y - state(1)) * cos((double) state(2)) + state(1);

				Vertex current_2d_vertex;
				environment_->getTerrainSpaceModel().coordToVertex(current_2d_vertex, point_position);

				// Inserts the element in an organized vertex queue, according to the maximum value
				if (terrain_costmap.count(current_2d_vertex) > 0)
					stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_costmap.find(current_2d_vertex)->second,
							current_2d_vertex));
			}
		}

		// Averaging the 5-best (lowest) cost
		unsigned int number_top_reward = number_top_reward_;
		if (stance_cost_queue.size() < number_top_reward)
			number_top_reward = stance_cost_queue.size();

		if (number_top_reward == 0) {
			stance_cost += uncertainty_factor_ * environment_->getAverageCostOfTerrain();
		} else {
			for (unsigned int i = 0; i < number_top_reward; i++) {
				stance_cost += stance_cost_queue.begin()->first;
				stance_cost_queue.erase(stance_cost_queue.begin());
			}

			stance_cost /= number_top_reward;
		}

		terrain_cost += stance_cost;
	}
	terrain_cost /= stance_areas_.size();


	// Getting the height map
	HeightMap heightmap;
	environment_->getTerrainHeightMap(heightmap);

	// Getting robot and terrain information
	RobotAndTerrain info;
	Eigen::Vector3d default_action;
	default_action << 1, 0, 0;
	info.body_action = default_action;
	info.pose.position = (Eigen::Vector2d) state.head(2);
	info.pose.orientation = (double) state(2);
	info.height_map = heightmap;
	info.resolution = environment_->getTerrainResolution();

	// Computing the cost of the body features
	cost = terrain_cost;
	unsigned int feature_size = features_.size();
	for (unsigned int i = 0; i < feature_size; i++) {
		// Computing the cost associated with body path features
		double feature_reward, weight;
		features_[i]->computeReward(feature_reward, info);
		features_[i]->getWeight(weight);

		// Computing the cost of the body feature
		cost -= weight * feature_reward;
	}
}


bool GridBasedBodyAdjacency::isStanceAdjacency()
{
	return is_stance_adjacency_;
}

} //@namespace environment
} //@namespace dwl
