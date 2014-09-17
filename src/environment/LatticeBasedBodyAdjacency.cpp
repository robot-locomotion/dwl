#include <environment/LatticeBasedBodyAdjacency.h>
#include <behavior/BodyMotorPrimitives.h>


namespace dwl
{

namespace environment
{

LatticeBasedBodyAdjacency::LatticeBasedBodyAdjacency() : behavior_(NULL), is_stance_adjacency_(true), number_top_reward_(5)
{
	name_ = "Lattice-based Body";
	is_lattice_ = true;

	behavior_ = new behavior::BodyMotorPrimitives(); //TODO Evaluates if it's necessary to add this externaly
}


LatticeBasedBodyAdjacency::~LatticeBasedBodyAdjacency()
{
	delete behavior_;
}


void LatticeBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors, Vertex state_vertex)
{
	stance_areas_ = robot_->getStanceAreas();

	// Getting the 3d pose for generating the actions
	std::vector<Action3d> actions;
	Eigen::Vector3d current_state;
	environment_->getTerrainSpaceModel().vertexToState(current_state, state_vertex);

	// Converting state to pose
	Pose3d current_pose;
	current_pose.position = current_state.head(2);
	current_pose.orientation = current_state(2);

	// Gets actions according the defined motor primitives of the body
	behavior_->generateActions(actions, current_pose);

	// Evaluating every action (body motor primitives)
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		unsigned int action_size = actions.size();
		for (unsigned int i = 0; i < action_size; i++) {
			// Converting the action to current vertex
			Eigen::Vector3d action_state;
			action_state << actions[i].pose.position, actions[i].pose.orientation;
			Vertex current_action_vertex, environment_vertex;
			environment_->getTerrainSpaceModel().stateToVertex(current_action_vertex, action_state);

			// Converting state vertex to environment vertex
			environment_->getTerrainSpaceModel().stateVertexToEnvironmentVertex(environment_vertex, current_action_vertex, XY_Y);

			// Checks if there is an obstacle
			if (isFreeOfObstacle(current_action_vertex, XY_Y, true)) {
				if (!isStanceAdjacency()) {
					double terrain_cost;
					if (terrain_costmap.find(environment_vertex)->first != environment_vertex)
						terrain_cost = uncertainty_factor_ * environment_->getAverageCostOfTerrain();
					else
						terrain_cost = terrain_costmap.find(environment_vertex)->second;

					successors.push_back(Edge(current_action_vertex, terrain_cost));
				} else {
					// Computing the body cost
					double body_cost;
					computeBodyCost(body_cost, action_state);
					body_cost += actions[i].cost;
					successors.push_back(Edge(current_action_vertex, body_cost));
				}
			}
		}
	} else
		printf(RED "Could not computed the successors because there is not terrain information \n" COLOR_RESET);
}


void LatticeBasedBodyAdjacency::computeBodyCost(double& cost, Eigen::Vector3d state)
{
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
		double resolution = stance_areas_[n].grid_resolution;
		for (double y = boundary_min(1); y < boundary_max(1); y += resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += resolution) {
				// Computing the rotated coordinate according to the orientation of the body
				Eigen::Vector2d point_position;
				point_position(0) = (x - state(0)) * cos((double) state(2)) - (y - state(1)) * sin((double) state(2)) + state(0);
				point_position(1) = (x - state(0)) * sin((double) state(2)) + (y - state(1)) * cos((double) state(2)) + state(1);

				Vertex current_2d_vertex;
				environment_->getTerrainSpaceModel().coordToVertex(current_2d_vertex, point_position);

				// Inserts the element in an organized vertex queue, according to the maximun value
				if (terrain_costmap.find(current_2d_vertex)->first == current_2d_vertex) {
					stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_costmap.find(current_2d_vertex)->second, current_2d_vertex));
				}
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
	info.pose.position = state.head(2);
	info.pose.orientation = state(2);
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


bool LatticeBasedBodyAdjacency::isFreeOfObstacle(Vertex state_vertex, TypeOfState state_representation, bool body)
{
	// Getting the terrain obstacle map
	ObstacleMap obstacle_map;
	environment_->getObstacleMap(obstacle_map);

	// Converting the vertex to state (x,y,yaw)
	Eigen::Vector3d state_3d;
	Eigen::Vector2d state_2d;
	double current_x, current_y, current_yaw;
	if (state_representation == XY) {
		environment_->getObstacleSpaceModel().vertexToState(state_2d, state_vertex);
		current_x = state_2d(0);
		current_y = state_2d(1);
		current_yaw = 0;
	} else {
		environment_->getObstacleSpaceModel().vertexToState(state_3d, state_vertex);
		current_x = state_3d(0);
		current_y = state_3d(1);
		current_yaw = state_3d(2);
	}

	bool is_free = true;
	if (environment_->isObstacleInformation()) {
		if (body) {
			// Getting the body area of the robot
			Area body_area = robot_->getBodyArea();

			// Computing the boundary of stance area
			Eigen::Vector2d boundary_min, boundary_max;
			boundary_min(0) = body_area.min_x + current_x;
			boundary_min(1) = body_area.min_y + current_y;
			boundary_max(0) = body_area.max_x + current_x;
			boundary_max(1) = body_area.max_y + current_y;

			// Getting the resolution of the obstacle map
			double obstacle_resolution = environment_->getObstacleResolution();

			for (double y = boundary_min(1); y < boundary_max(1); y += obstacle_resolution) {
				for (double x = boundary_min(0); x < boundary_max(0); x += obstacle_resolution) {
					// Computing the rotated coordinate according to the orientation of the body
					Eigen::Vector2d point_position;
					point_position(0) = (x - current_x) * cos(current_yaw) - (y - current_y) * sin(current_yaw) + current_x;
					point_position(1) = (x - current_x) * sin(current_yaw) + (y - current_y) * cos(current_yaw) + current_y;

					Vertex current_2d_vertex;
					environment_->getObstacleSpaceModel().coordToVertex(current_2d_vertex, point_position);

					// Checking if there is an obstacle
					if (obstacle_map.find(current_2d_vertex)->first == current_2d_vertex) {
						if (obstacle_map.find(current_2d_vertex)->second) {
							is_free = false;
							std::cout << "is_free = " << is_free << " | = " <<  current_x << " " << current_y << " " << current_yaw << std::endl; //TODO Delete this message
							goto found_obstacle;
						}
					}
				}
			}
		} else {
			// Converting the state vertex to terrain vertex
			Vertex environment_vertex;
			environment_->getObstacleSpaceModel().stateVertexToEnvironmentVertex(environment_vertex, state_vertex, state_representation);

			if (obstacle_map.find(environment_vertex)->first == environment_vertex) {
				if (obstacle_map.find(environment_vertex)->second)
					is_free = false;
			}
		}
	}

	found_obstacle:
	return is_free;
}


bool LatticeBasedBodyAdjacency::isStanceAdjacency()
{
	return is_stance_adjacency_;
}

} //@namespace environment
} //@namespace dwl
