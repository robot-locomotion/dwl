#include <environment/GridBasedBodyAdjacency.h>


namespace dwl
{

namespace environment
{

GridBasedBodyAdjacency::GridBasedBodyAdjacency() : is_stance_adjacency_(true), number_top_reward_(5), uncertainty_factor_(1.15)
{
	name_ = "grid-based body";

	SearchArea area;
	double x_foot[] = {0.4269, 0.4269, -0.4269, -0.4269};
	double y_foot[] = {0.3886, -0.3886, 0.3886, -0.3886};
	area.grid_resolution = 0.04;

	// Defining the search areas for the stance position of HyQ
	for (int i = 0; i < 4; i++) {
		area.max_x = x_foot[i] + 0.1;
		area.min_x = x_foot[i] - 0.1;
		area.max_y = y_foot[i] + 0.1;
		area.min_y = y_foot[i] - 0.1;
		stance_areas_.push_back(area);
	}
}

GridBasedBodyAdjacency::~GridBasedBodyAdjacency()
{

}

void GridBasedBodyAdjacency::computeAdjacencyMap(AdjacencyMap& adjacency_map, Eigen::Vector3d position)
{
	if (is_there_terrain_information_) {
		for (CostMap::iterator vertex_iter = terrain_cost_map_.begin();
				vertex_iter != terrain_cost_map_.end();
				vertex_iter++)
		{
			Vertex vertex = vertex_iter->first;

			if (!isStanceAdjacency()) {
				double terrain_cost = vertex_iter->second;

				//addCostToAdjacentVertexs(adjacency_map, vertex, terrain_cost); //TODO Extent for lattice representation. To think about how, i.e. in the same class or using inherince
				std::vector<Vertex> neighbours;
				searchNeighbours(neighbours, vertex);
				for (int i = 0; i < neighbours.size(); i++)
					adjacency_map[neighbours[i]].push_back(Edge(vertex, terrain_cost));
			} else {
				//Computing the body cost
				Eigen::Vector2d vertex_position = gridmap_.vertexToCoord(vertex);
				double yaw = position(2);

				double body_cost = 0;
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
							point_position(0) = (x - vertex_position(0)) * cos(yaw) - (y - vertex_position(1)) * sin(yaw) + vertex_position(0);//robot_state
							point_position(1) = (x - vertex_position(0)) * sin(yaw) + (y - vertex_position(1)) * cos(yaw) + vertex_position(1);

							Vertex point = gridmap_.coordToVertex(point_position);

							// Inserts the element in an organized vertex queue, according to the maximun value
							if (terrain_cost_map_.find(point)->first == point)
								stance_cost_queue.insert(std::pair<Weight, Vertex>(terrain_cost_map_.find(point)->second, point));
						}
					}

					// Averaging the 5-best rewards
					if (stance_cost_queue.size() < number_top_reward_)
						number_top_reward_ = stance_cost_queue.size();

					if (number_top_reward_ == 0) {
						stance_cost += uncertainty_factor_ * average_cost_;
					} else {
						for (int i = 0; i < number_top_reward_; i++) {
							stance_cost += stance_cost_queue.begin()->first;
							stance_cost_queue.erase(stance_cost_queue.begin());
						}

						stance_cost /= number_top_reward_;
					}
					//std::cout << "Stance cost = " << stance_cost << std::endl;

					body_cost += stance_cost;
				}

				body_cost /= stance_areas_.size();
				//std::cout << "body cost = " << body_cost << std::endl;

				//addCostToAdjacentVertexs(adjacency_map, vertex, body_cost); //TODO Extent for lattice representation. To think about how, i.e. in the same class or using inherince
				std::vector<Vertex> neighbours;
				searchNeighbours(neighbours, vertex);
				for (int i = 0; i < neighbours.size(); i++)
					adjacency_map[neighbours[i]].push_back(Edge(vertex, body_cost));
			}
		}
	} else
		printf(RED "Couldn't compute the adjacencyt map because there isn't terrain information \n" COLOR_RESET);
}


void GridBasedBodyAdjacency::searchNeighbours(std::vector<Vertex>& neighbours, Vertex vertex_id)
{
	Key vertex_key = gridmap_.vertexToGridMapKey(vertex_id);

	// Searching the closed neighbors around 3-neighboring area
	bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
	bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
	bool is_found_neighbour_positive_xy = false, is_found_neighbour_negative_xy = false;
	bool is_found_neighbour_positive_yx = false, is_found_neighbour_negative_yx = false;
	for (int r = 1; r <= 3; r++) {
		Key searching_key;
		Vertex neighbour_vertex;

		// Searching the neighbour in the positive x-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_x)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_positive_x = true;
		}

		// Searching the neighbour in the negative x-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_x)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_negative_x = true;
		}

		// Searching the neighbour in the positive y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_y)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_positive_y = true;
		}

		// Searching the neighbour in the negative y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_y)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_negative_y = true;
		}

		// Searching the neighbour in the positive xy-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_xy)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_positive_xy = true;
		}

		// Searching the neighbour in the negative xy-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_xy)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_negative_xy = true;
		}

		// Searching the neighbour in the positive yx-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_yx)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_positive_yx = true;
		}

		// Searching the neighbour in the negative yx-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_yx)) {
			neighbours.push_back(neighbour_vertex);
			is_found_neighbour_negative_yx = true;
		}
	}
}

void GridBasedBodyAdjacency::addCostToAdjacentVertexs(AdjacencyMap& adjacency_map, Vertex vertex_id, double cost) //TODO Grid-based adjacency representation
{
	Key vertex_key = gridmap_.vertexToGridMapKey(vertex_id);

	// Searching the closed neighbors around 3-neighboring area
	bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
	bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
	bool is_found_neighbour_positive_xy = false, is_found_neighbour_negative_xy = false;
	bool is_found_neighbour_positive_yx = false, is_found_neighbour_negative_yx = false;
	for (int r = 1; r <= 3; r++) {
		Key searching_key;
		Vertex neighbour_vertex;

		// Searching the neighbour in the positive x-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_x)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +x = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_x = true;
		}

		// Searching the neighbour in the negative x-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1];
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_x)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -x = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_x = true;
		}

		// Searching the neighbour in the positive y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_y)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour +y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_y = true;
		}

		// Searching the neighbour in the negative y-axis
		searching_key.key[0] = vertex_key.key[0];
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_y)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_y = true;
		}

		// Searching the neighbour in the positive xy-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_xy)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_xy = true;
		}

		// Searching the neighbour in the negative xy-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_xy)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_xy = true;
		}

		// Searching the neighbour in the positive yx-axis
		searching_key.key[0] = vertex_key.key[0] - r;
		searching_key.key[1] = vertex_key.key[1] + r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_positive_yx)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_positive_yx = true;
		}

		// Searching the neighbour in the negative yx-axis
		searching_key.key[0] = vertex_key.key[0] + r;
		searching_key.key[1] = vertex_key.key[1] - r;
		neighbour_vertex = gridmap_.gridMapKeyToVertex(searching_key);
		if ((terrain_cost_map_.find(neighbour_vertex)->first == neighbour_vertex) && (!is_found_neighbour_negative_yx)) {
			adjacency_map[neighbour_vertex].push_back(Edge(vertex_id, cost));
//				std::cout << "Vertex = " << neighbour_vertex << " | Neighbour -y = " << vertex_id << " with cost = " << cost << std::endl;
			is_found_neighbour_negative_yx = true;
		}
	}
}


void GridBasedBodyAdjacency::getSuccessors(std::list<Edge>& successors, Vertex vertex)
{
	std::vector<Vertex> neighbours;
	searchNeighbours(neighbours, vertex);
	for (int i = 0; i < neighbours.size(); i++) {
		double cost = terrain_cost_map_.find(neighbours[i])->second;
		successors.push_back(Edge(vertex, cost));
	}
}


bool GridBasedBodyAdjacency::isStanceAdjacency()
{
	return is_stance_adjacency_;
}


} //@namespace environment

} //@namespace dwl
