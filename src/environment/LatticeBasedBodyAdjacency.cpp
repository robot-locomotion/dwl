#include <environment/LatticeBasedBodyAdjacency.h>
#include <behavior/BodyMotorPrimitives.h>


namespace dwl
{

namespace environment
{

LatticeBasedBodyAdjacency::LatticeBasedBodyAdjacency() : behavior_(NULL), is_stance_adjacency_(true), number_top_reward_(5)
{
	name_ = "lattice-based body";
	is_lattice_ = true;

	behavior_ = new behavior::BodyMotorPrimitives();

	//TODO stance area
	stance_areas_ = robot_.getStanceAreas();
}


LatticeBasedBodyAdjacency::~LatticeBasedBodyAdjacency()
{
	delete behavior_;
}


void LatticeBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors, Vertex vertex)
{
	// Getting the 3d pose for generating the actions
	std::vector<Action3d> actions;
	Pose3d state;
	state.position = environment_->getGridModel().vertexToCoord(vertex);
	if (orientations_.find(vertex)->first == vertex) {
		// Adding the orientation of the action
		state.orientation = orientations_.find(vertex)->second;
	} else {
		// Adding the initial orientation
		// Converting quaternion to roll, pitch and yaw angles
		double roll, pitch, yaw;
		Orientation orientation(current_pose_.orientation);
		orientation.getRPY(roll, pitch, yaw);
		state.orientation = yaw;
	}

	// Gets actions according the defined motor primitives of the body
	behavior_->generateActions(actions, state);

	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (int i = 0; i < actions.size(); i++) {
			// Converting the action to vertex
			Vertex current = environment_->getGridModel().coordToVertex(actions[i].pose.position);

			// Recording orientations of the actions
			orientations_[current] = actions[i].pose.orientation;

			if (!isStanceAdjacency()) {
				double terrain_cost;
				if (terrain_costmap.find(current)->first != current)
					terrain_cost = uncertainty_factor_ * environment_->getAverageCostOfTerrain();
				else
					terrain_cost = terrain_costmap.find(current)->second;

				successors.push_back(Edge(current, terrain_cost));
			} else {
				// Computing the body cost
				double body_cost;
				computeBodyCost(body_cost, current, actions[i].pose.orientation);
				body_cost += actions[i].cost;
				successors.push_back(Edge(current, body_cost));
				std::cout << "Succesors (vertex | position | cost) = " << current << " | " << actions[i].pose.position(0) << " " << actions[i].pose.position(1) << " | " << body_cost << std::endl;
			}
		}
	} else
		printf(RED "Couldn't compute the successors because there isn't terrain information \n" COLOR_RESET);

	std::cout << "----------------------------------------------------------" << std::endl;
}


void LatticeBasedBodyAdjacency::computeBodyCost(double& cost, Vertex vertex, double orientation)
{
	Eigen::Vector2d vertex_position = environment_->getGridModel().vertexToCoord(vertex);
	CostMap terrain_costmap;
	environment_->getTerrainCostMap(terrain_costmap);

	double body_cost;
	for (int n = 0; n < stance_areas_.size(); n++) {
		// Computing the boundary of stance area
		Eigen::Vector2d boundary_min, boundary_max;
		boundary_min(0) = stance_areas_[n].min_x + vertex_position(0);
		boundary_min(1) = stance_areas_[n].min_y + vertex_position(1);
		boundary_max(0) = stance_areas_[n].max_x + vertex_position(0);
		boundary_max(1) = stance_areas_[n].max_y + vertex_position(1);

		std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > stance_cost_queue;
		double stance_cost = 0;
		for (double y = boundary_min(1); y < boundary_max(1); y += stance_areas_[n].grid_resolution) {
			for (double x = boundary_min(0); x < boundary_max(0); x += stance_areas_[n].grid_resolution) {
				// Computing the rotated coordinate of the point inside the search area
				Eigen::Vector2d point_position;
				point_position(0) = (x - vertex_position(0)) * cos(orientation) - (y - vertex_position(1)) * sin(orientation) + vertex_position(0);
				point_position(1) = (x - vertex_position(0)) * sin(orientation) + (y - vertex_position(1)) * cos(orientation) + vertex_position(1);

				Vertex point = environment_->getGridModel().coordToVertex(point_position);

				// Inserts the element in an organized vertex queue, according to the maximun value
				if (terrain_costmap.find(point)->first == point)
					stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_costmap.find(point)->second, point));
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

	cost = body_cost;//TODO compute cost assoacited to support triangle and other features
}


bool LatticeBasedBodyAdjacency::isStanceAdjacency()
{
	return is_stance_adjacency_;
}

} //@namespace environment
} //@namespace dwl
