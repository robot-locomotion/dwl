#include <planning/CostMap.h>
#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

CostMap::CostMap() : is_first_update_(true)
{
	name_ = "CostMap";
	is_cost_map_ = true;

	//TODO
	environment::SearchArea area;
	area.grid_resolution = 0.04;
	area.max_x = 1.5 + 0.1;
	area.min_x = 1.5 - 0.1;
	area.max_y = 0.0 + 0.1;
	area.min_y = 0.0 - 0.1;
	stance_areas_.push_back(area);
/*	area.max_x = 1 + 0.25;
	area.min_x = 1 - 0.25;
	area.max_y = 1 + 0.25;
	area.min_y = 1 - 0.25;
	stance_areas_.push_back(area);*/
}


CostMap::~CostMap()
{

}


void CostMap::setCostMap(std::vector<dwl::environment::Cell> reward_map)
{
/*	dwl::environment::Cell c;
	std::vector<dwl::environment::Cell> reward_map;
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -1.5;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1001;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -2;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 999;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -3;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 1001;
	c.reward = -4;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 999;
	c.reward = -5;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 998;
	c.reward = -6;
	std::cout << "vertex = " << gridmap_.gridMapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);*/






	// Storing the cost-map data according the vertex id
	unsigned int vertex_id;
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		costmap_[vertex_id] = - reward_map[i].reward;
	}

	//  Converting the reward map message to a terrain adjacency map (cost-map), which is required for graph-searching algorithms
	double cost = 0;
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		cost = - reward_map[i].reward;

		addCostToAdjacencyVertex(terrain_cost_map_, vertex_id, cost);

/*		// Searching the closed neighbors around 3-neighboring area
		bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
		bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
		for (int r = 1; r <= 3; r++) {
			environment::Key searching_key;
			Vertex neighbour_vertex;

			// Searching the neighbour in the positive x-axis
			searching_key.key[0] = vertex_key.key[0] + r;
			searching_key.key[1] = vertex_key.key[1];
			neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
			if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_x)) {
				terrain_cost_map_[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +x = " << vertex_id << " with cost = " << cost << std::endl;
				is_found_neighbour_positive_x = true;
			}

			// Searching the neighbour in the negative x-axis
			searching_key.key[0] = vertex_key.key[0] - r;
			searching_key.key[1] = vertex_key.key[1];
			neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
			if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_x)) {
				terrain_cost_map_[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -x = " << vertex_id << " with cost = " << cost << std::endl;
				is_found_neighbour_negative_x = true;
			}

			// Searching the neighbour in the positive y-axis
			searching_key.key[0] = vertex_key.key[0];
			searching_key.key[1] = vertex_key.key[1] + r;
			neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
			if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_y)) {
				terrain_cost_map_[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +y = " << vertex_id << " with cost = " << cost << std::endl;
				is_found_neighbour_positive_y = true;
			}

			// Searching the neighbour in the negative y-axis
			searching_key.key[0] = vertex_key.key[0];
			searching_key.key[1] = vertex_key.key[1] - r;
			neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
			if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_y)) {
				terrain_cost_map_[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
				is_found_neighbour_negative_y = true;
			}
		}*/
	}







	std::cout << "size = " << terrain_cost_map_.size() << std::endl;
}


void CostMap::get(AdjacencyMap& adjacency_map, Eigen::Vector3d robot_state, bool terrain_cost)
{
	if (terrain_cost) {
		adjacency_map = terrain_cost_map_;

	} else {
		// Computing the body cost-map
		computeBodyCostMap(robot_state);
		adjacency_map = body_cost_map_;

		// Deleting the old terrain and body cost-map and reward vector
		AdjacencyMap empty_cost_map, empty_body_cost_map;
		std::map<Vertex, Weight> empty_reward_vector;

		terrain_cost_map_.swap(empty_cost_map);
		body_cost_map_.swap(empty_body_cost_map);
		costmap_.swap(empty_reward_vector);
	}
}


void CostMap::computeBodyCostMap(Eigen::Vector3d robot_state) //TODO
{
	// Computing a body adjacency map (body cost-map)
	double yaw = robot_state(2);
	for (AdjacencyMap::iterator vertex_iter = terrain_cost_map_.begin();
				vertex_iter != terrain_cost_map_.end();
				vertex_iter++)
	{
		Vertex vertex_id = vertex_iter->first;
		Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex_id);

		double body_cost = 0;
		for (int n = 0; n < stance_areas_.size(); n++) {
			// Computing the boundary of stance area
			Eigen::Vector2d boundary_min, boundary_max;
			boundary_min(0) = stance_areas_[n].min_x + vertex_position(0);
			boundary_min(1) = stance_areas_[n].min_y + vertex_position(1);
			boundary_max(0) = stance_areas_[n].max_x + vertex_position(0);
			boundary_max(1) = stance_areas_[n].max_y + vertex_position(1);

			std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > vertex_queue;
			double stance_cost = 0;
			for (double y = boundary_min(1); y < boundary_max(1); y += stance_areas_[n].grid_resolution) {
				for (double x = boundary_min(0); x < boundary_max(0); x += stance_areas_[n].grid_resolution) {
					// Computing the rotated coordinate of the point inside the search area
					Eigen::Vector2d point_position;
					point_position(0) = (x - vertex_position(0)) * cos(yaw) - (y - vertex_position(1)) * sin(yaw) + vertex_position(0);//robot_state
					point_position(1) = (x - vertex_position(0)) * sin(yaw) + (y - vertex_position(1)) * cos(yaw) + vertex_position(1);

					Vertex point = gridmap_.coordToVertex(point_position);

					// Insert the element in an organized vertex queue, according to the maximun value
					vertex_queue.insert(std::pair<Weight, Vertex>(costmap_.find(point)->second, point));
				}
			}

			// Averaging the 5-best rewards
			int number_top_reward = 5;
			if (vertex_queue.size() < number_top_reward)
				number_top_reward = vertex_queue.size();

			for (int i = 0; i < number_top_reward; i++) {
				stance_cost += vertex_queue.begin()->first;
				vertex_queue.erase(vertex_queue.begin());
			}
			stance_cost /= number_top_reward;
			std::cout << "Stance cost = " << stance_cost << std::endl;

			body_cost += stance_cost;
		}

		body_cost /= stance_areas_.size();

		//addCostToAdjacencyVertex(body_cost_map_, vertex_id, body_cost);
	}
}


void CostMap::addCostToAdjacencyVertex(AdjacencyMap& adjacency_map, Vertex vertex_id, double cost)
{
	environment::Key vertex_key = gridmap_.vertexToGridMapKey(vertex_id);

	// Searching the closed neighbors around 3-neighboring area
	bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
	bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
	for (int r = 1; r <= 3; r++) {
		environment::Key searching_key;
		Vertex neighbour_vertex;

		// Searching the neighbour in the positive x-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_x)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +x = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_x = true;
		}

		// Searching the neighbour in the negative x-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_x)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -x = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_x = true;
		}

		// Searching the neighbour in the positive y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_y)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_y = true;
		}

		// Searching the neighbour in the negative y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((costmap_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_y)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_y = true;
		}
	}
}

} //@namespace planning

} //@namespace dwl






/*	unsigned int vertex_id, edge_id;
	double cost;
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridMapKeyToVertex(reward_map[i].cell_key.grid_id);
		unsigned short int vertex_x = reward_map[i].cell_key.grid_id.key[0];
		unsigned short int vertex_y = reward_map[i].cell_key.grid_id.key[1];

		// Searching the closed neighbors around 3-neighboring area
		bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
		bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
		for (int r = 1; r <= 3; r++) { //TODO
			for (int j = 0; j < reward_map.size(); j++) {
				unsigned short int edge_x = reward_map[j].cell_key.grid_id.key[0];
				unsigned short int edge_y = reward_map[j].cell_key.grid_id.key[1];

				// Getting the values of the edge
				edge_id = gridmap_.gridMapKeyToVertex(reward_map[j].cell_key.grid_id);
				cost = - reward_map[j].reward;

				// Searching the neighbour in the positive x-axis
				if ((vertex_x + r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_positive_x)) {
					terrain_cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					reward_vector_[vertex_id] = cost;
					std::cout << "Vertex = " << vertex_id << " | Neighbour +x = " << edge_id << " with cost = " << cost << std::endl;
					is_found_neighbour_positive_x = true;
				}

				// Searching the neighbour in the negative x-axis
				if ((vertex_x - r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_negative_x)) {
					terrain_cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					reward_vector_[vertex_id] = cost;
					std::cout << "Vertex = " << vertex_id << " | Neighbour -x = " << edge_id << " with cost = " << cost << std::endl;
					is_found_neighbour_negative_x = true;
				}

				// Searching the neighbour in the positive y-axis
				if ((vertex_y + r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_positive_y)) {
					terrain_cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					reward_vector_[vertex_id] = cost;
					std::cout << "Vertex = " << vertex_id << " | Neighbour +y = " << edge_id << " with cost = " << cost << std::endl;
					is_found_neighbour_positive_y = true;
				}

				// Searching the neighbour in the negative y-axis
				if ((vertex_y - r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_negative_y)) {
					terrain_cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					reward_vector_[vertex_id] = cost;
					std::cout << "Vertex = " << vertex_id << " | Neighbour -y = " << edge_id << " with cost = " << cost << std::endl;
					is_found_neighbour_negative_y = true;
				}
			}
		}
	}*/
