#include <planning/CostMap.h>
#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

CostMap::CostMap()
{
	name_ = "cost map";
	is_cost_map_ = true;
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
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1001;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -2;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 999;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -3;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 1001;
	c.reward = -4;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 999;
	c.reward = -5;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 998;
	c.reward = -6;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	reward_map.push_back(c);*/






	unsigned long int vertex_id, edge_id;
	double cost;
	for (int i = 0; i < reward_map.size(); i++) {
		vertex_id = gridmap_.gridmapKeyToVertex(reward_map[i].cell_key.grid_id);
		unsigned short int vertex_x = reward_map[i].cell_key.grid_id.key[0];
		unsigned short int vertex_y = reward_map[i].cell_key.grid_id.key[1];

		// Searching the closed neighbors around 5-neighboring area
		bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
		bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
		for (int r = 1; r <= 3; r++) {
			for (int j = 0; j < reward_map.size(); j++) {
				unsigned short int edge_x = reward_map[j].cell_key.grid_id.key[0];
				unsigned short int edge_y = reward_map[j].cell_key.grid_id.key[1];

				// Getting the values of the edge
				edge_id = gridmap_.gridmapKeyToVertex(reward_map[j].cell_key.grid_id);
				cost = - reward_map[j].reward;

				// Searching the neighbour in the positive x-axis
				if ((vertex_x + r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_positive_x)) {
//					std::cout << "v=" << vertex_id << " | e=" << edge_id << " x+ | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_positive_x = true;
				}

				// Searching the neighbour in the negative x-axis
				if ((vertex_x - r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_negative_x)) {
//					std::cout << "v=" << vertex_id << " | e=" << edge_id << " x- | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_negative_x = true;
				}

				// Searching the neighbour in the positive y-axis
				if ((vertex_y + r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_positive_y)) {
//					std::cout << "v=" << vertex_id << " | e=" << edge_id << " y+ | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_positive_y = true;
				}

				// Searching the neighbour in the negative y-axis
				if ((vertex_y - r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_negative_y)) {
//					std::cout << "v=" << vertex_id << " | e=" << edge_id << " y- | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_negative_y = true;
				}
			}
		}
	}
}


void CostMap::get(AdjacencyMap& state)
{
	state = cost_map_;
}


} //@namespace planning

} //@namespace dwl
