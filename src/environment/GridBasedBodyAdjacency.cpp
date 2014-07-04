#include <environment/GridBasedBodyAdjacency.h>


namespace dwl
{

namespace environment
{

GridBasedBodyAdjacency::GridBasedBodyAdjacency() : is_stance_adjacency_(true), neighboring_definition_(3), number_top_reward_(5), uncertainty_factor_(1.15)
{
	name_ = "grid-based body";
	is_lattice_ = false;

	//TODO stance area
	stance_areas_ = robot_.getStanceAreas();
}


GridBasedBodyAdjacency::~GridBasedBodyAdjacency()
{

}


void GridBasedBodyAdjacency::computeAdjacencyMap(AdjacencyMap& adjacency_map, Vertex source, Vertex target)
{
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

			if (!isStanceAdjacency()) {
				double terrain_cost = vertex_iter->second;

				// Searching the neighbours
				std::vector<Vertex> neighbors;
				searchNeighbors(neighbors, vertex);
				for (int i = 0; i < neighbors.size(); i++)
					adjacency_map[neighbors[i]].push_back(Edge(vertex, terrain_cost));
			} else {
				// Computing the body cost
				double body_cost;
				// Adding the initial orientation
				// Converting quaternion to roll, pitch and yaw angles
				double roll, pitch, yaw;
				Orientation orientation(current_pose_.orientation);
				orientation.getRPY(roll, pitch, yaw);
				computeBodyCost(body_cost, vertex, yaw);

				// Searching the neighbours
				std::vector<Vertex> neighbors;
				searchNeighbors(neighbors, vertex);
				for (int i = 0; i < neighbors.size(); i++)
					adjacency_map[neighbors[i]].push_back(Edge(vertex, body_cost));
			}
		}
	} else
		printf(RED "Couldn't compute the adjacency map because there isn't terrain information \n" COLOR_RESET);
}


void GridBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors, Vertex vertex)
{
	// Getting the 3d pose for generating the actions
	std::vector<Pose3d> actions;
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

	std::vector<Vertex> neighbors;
	searchNeighbors(neighbors, vertex);
	if (environment_->isTerrainInformation()) {
		// Getting the terrain costmap
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);

		// Computing the vertex position
		Eigen::Vector2d vertex_position = environment_->getGridModel().vertexToCoord(vertex);

		for (int i = 0; i < neighbors.size(); i++) {
			// Computing the orientation between the current vertex and its neighbors
			Eigen::Vector2d neighbor_position = environment_->getGridModel().vertexToCoord(neighbors[i]);
			double delta_x = neighbor_position(0) - vertex_position(0);
			double delta_y = neighbor_position(1) - vertex_position(1);
			double orientation = atan2(delta_y, delta_x);
			orientations_[neighbors[i]] = orientation;

			if (!isStanceAdjacency()) {
				double terrain_cost = terrain_costmap.find(neighbors[i])->second;
				successors.push_back(Edge(neighbors[i], terrain_cost));
			} else {
				// Computing the body cost
				double body_cost;
				computeBodyCost(body_cost, neighbors[i], orientation);
				successors.push_back(Edge(neighbors[i], body_cost));
			}
		}
	} else
		printf(RED "Couldn't compute the successors because there isn't terrain information \n" COLOR_RESET);
}


void GridBasedBodyAdjacency::searchNeighbors(std::vector<Vertex>& neighbors, Vertex vertex_id)
{
	Key vertex_key = environment_->getGridModel().vertexToGridMapKey(vertex_id);

	// Searching the closed neighbors around 3-neighboring area
	bool is_found_neighbor_positive_x = false, is_found_neighbor_negative_x = false;
	bool is_found_neighbor_positive_y = false, is_found_neighbor_negative_y = false;
	bool is_found_neighbor_positive_xy = false, is_found_neighbor_negative_xy = false;
	bool is_found_neighbor_positive_yx = false, is_found_neighbor_negative_yx = false;
	if (environment_->isTerrainInformation()) {
		CostMap terrain_costmap;
		environment_->getTerrainCostMap(terrain_costmap);
		for (int r = 1; r <= neighboring_definition_; r++) {
			Key searching_key;
			Vertex neighbor_vertex;

			// Searching the neighbour in the positive x-axis
			searching_key.key[0] = vertex_key.key[0] + r;
			searching_key.key[1] = vertex_key.key[1];
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_positive_x)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_positive_x = true;
			}

			// Searching the neighbour in the negative x-axis
			searching_key.key[0] = vertex_key.key[0] - r;
			searching_key.key[1] = vertex_key.key[1];
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_negative_x)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_negative_x = true;
			}

			// Searching the neighbour in the positive y-axis
			searching_key.key[0] = vertex_key.key[0];
			searching_key.key[1] = vertex_key.key[1] + r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_positive_y)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_positive_y = true;
			}

			// Searching the neighbour in the negative y-axis
			searching_key.key[0] = vertex_key.key[0];
			searching_key.key[1] = vertex_key.key[1] - r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_negative_y)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_negative_y = true;
			}

			// Searching the neighbour in the positive xy-axis
			searching_key.key[0] = vertex_key.key[0] + r;
			searching_key.key[1] = vertex_key.key[1] + r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_positive_xy)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_positive_xy = true;
			}

			// Searching the neighbour in the negative xy-axis
			searching_key.key[0] = vertex_key.key[0] - r;
			searching_key.key[1] = vertex_key.key[1] - r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_negative_xy)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_negative_xy = true;
			}

			// Searching the neighbour in the positive yx-axis
			searching_key.key[0] = vertex_key.key[0] - r;
			searching_key.key[1] = vertex_key.key[1] + r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_positive_yx)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_positive_yx = true;
			}

			// Searching the neighbour in the negative yx-axis
			searching_key.key[0] = vertex_key.key[0] + r;
			searching_key.key[1] = vertex_key.key[1] - r;
			neighbor_vertex = environment_->getGridModel().gridMapKeyToVertex(searching_key);
			if ((terrain_costmap.find(neighbor_vertex)->first == neighbor_vertex) && (!is_found_neighbor_negative_yx)) {
				neighbors.push_back(neighbor_vertex);
				is_found_neighbor_negative_yx = true;
			}
		}
	} else
		printf(RED "Couldn't search the neighbours because there isn't terrain information \n" COLOR_RESET);
}


void GridBasedBodyAdjacency::computeBodyCost(double& cost, Vertex vertex, double orientation)
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
	cost = body_cost;
}


bool GridBasedBodyAdjacency::isStanceAdjacency()
{
	return is_stance_adjacency_;
}

} //@namespace environment
} //@namespace dwl
