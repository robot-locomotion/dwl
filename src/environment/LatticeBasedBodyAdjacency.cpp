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

	behavior_ = new behavior::BodyMotorPrimitives();

	//TODO stance area
	stance_areas_ = robot_.getStanceAreas();
}


LatticeBasedBodyAdjacency::~LatticeBasedBodyAdjacency()
{
	delete behavior_;
}


void LatticeBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors, Vertex state_vertex)
{
	// Getting the 3d pose for generating the actions
	std::vector<Action3d> actions;
	Eigen::Vector3d current_state;
	environment_->getSpaceModel().vertexToState(current_state, state_vertex);

	// Converting state to pose
	Pose3d current_pose;
	current_pose.position = current_state.head(2);
	current_pose.orientation = current_state(2);

	// Gets actions according the defined motor primitives of the body
	behavior_->generateActions(actions, current_pose);
	if (environment_->isTerrainCostInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (int i = 0; i < actions.size(); i++) {
			// Converting the action to current vertex
			Eigen::Vector3d action_states;
			action_states << actions[i].pose.position, actions[i].pose.orientation;
			Vertex current_action_vertex, environment_vertex;
			environment_->getSpaceModel().stateToVertex(current_action_vertex, action_states);

			// Converting state vertex to environment vertex
			environment_->getSpaceModel().stateVertexToEnvironmentVertex(environment_vertex, current_action_vertex, XY_Y);

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
					computeBodyCost(body_cost, current_action_vertex);
					body_cost += actions[i].cost; //TODO
					successors.push_back(Edge(current_action_vertex, body_cost));
				}
			}
		}
	} else
		printf(RED "Could not computed the successors because there isn't terrain information \n" COLOR_RESET);
}


void LatticeBasedBodyAdjacency::computeBodyCost(double& cost, Vertex state_vertex)
{
	// Converting the vertex to state (x,y,yaw)
	Eigen::Vector3d state;
	environment_->getSpaceModel().vertexToState(state, state_vertex);

	// Getting the terrain cost map
	CostMap terrain_costmap;
	environment_->getTerrainCostMap(terrain_costmap);

	// Computing the body cost
	double body_cost;
	for (int n = 0; n < stance_areas_.size(); n++) {
		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = stance_areas_[n].min_x + state(0);
		boundary_min(1) = stance_areas_[n].min_y + state(1);
		boundary_max(0) = stance_areas_[n].max_x + state(0);
		boundary_max(1) = stance_areas_[n].max_y + state(1);

		// Computing the stance cost
		std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > stance_cost_queue;
		double stance_cost = 0;
		for (double y = boundary_min(1); y < boundary_max(1); y += stance_areas_[n].grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += stance_areas_[n].grid_resolution) {
				// Computing the rotated coordinate according to the orientation of the body
				Eigen::Vector2d point_position;
				point_position(0) = (x - state(0)) * cos((double) state(2)) - (y - state(1)) * sin((double) state(2)) + state(0);
				point_position(1) = (x - state(0)) * sin((double) state(2)) + (y - state(1)) * cos((double) state(2)) + state(1);

				Vertex current_2d_vertex;
				environment_->getSpaceModel().coordToVertex(current_2d_vertex, point_position);

				// Inserts the element in an organized vertex queue, according to the maximun value
				if (terrain_costmap.find(current_2d_vertex)->first == current_2d_vertex)
					stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_costmap.find(current_2d_vertex)->second, current_2d_vertex));
			}
		}

		// Averaging the 5-best (lowest) cost
		int number_top_reward = number_top_reward_;
		if (stance_cost_queue.size() < number_top_reward)
			number_top_reward = stance_cost_queue.size();

		if (number_top_reward == 0) {
			stance_cost += uncertainty_factor_ * environment_->getAverageCostOfTerrain();
		} else {
			for (int i = 0; i < number_top_reward; i++) {
				stance_cost += stance_cost_queue.begin()->first;
				stance_cost_queue.erase(stance_cost_queue.begin());
			}

			stance_cost /= number_top_reward;
		}

		body_cost += stance_cost;
	}
	body_cost /= stance_areas_.size();
	cost = body_cost;
}


bool LatticeBasedBodyAdjacency::isFreeOfObstacle(Vertex state_vertex, State state_representation, bool body)
{
	// Getting the terrain obstacle map
	ObstacleMap obstacle_map;
	environment_->getTerrainObstacleMap(obstacle_map);

	// Converting the vertex to state (x,y,yaw)
	Eigen::Vector3d state;
	environment_->getSpaceModel().vertexToState(state, state_vertex);

	bool is_free;
	if (body) {
		// Getting the body area of the robot
		SearchArea body_area = robot_.getBodyArea();

		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = body_area.min_x + state(0);
		boundary_min(1) = body_area.min_y + state(1);
		boundary_max(0) = body_area.max_x + state(0);
		boundary_max(1) = body_area.max_y + state(1);

		std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > stance_cost_queue;
		double stance_cost = 0;
		for (double y = boundary_min(1); y < boundary_max(1); y += body_area.grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += body_area.grid_resolution) {
				// Computing the rotated coordinate according to the orientation of the body
				Eigen::Vector2d point_position;
				point_position(0) = (x - state(0)) * cos((double) state(2)) - (y - state(1)) * sin((double) state(2)) + state(0);
				point_position(1) = (x - state(0)) * sin((double) state(2)) + (y - state(1)) * cos((double) state(2)) + state(1);

				Vertex current_2d_vertex;
				environment_->getSpaceModel().coordToVertex(current_2d_vertex, point_position);

				// Checking if there is an obstacle
				//std::cout << obstacle_map.find(current_2d_vertex)->second << std::endl;
				if (obstacle_map.find(current_2d_vertex)->second) {
					std::cout << obstacle_map.find(current_2d_vertex)->second << std::endl;
					std::cout << "Obstacle" << std::endl;
					is_free = false;
					goto found_obstacle;
				}
				else
					is_free = true;
			}
		}

	} else {
		// Converting the state vertex to terrain vertex
		Vertex environment_vertex;
		environment_->getSpaceModel().stateVertexToEnvironmentVertex(environment_vertex, state_vertex, state_representation);

		if (obstacle_map.find(environment_vertex)->second)
			is_free = false;
		else
			is_free = true;
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
