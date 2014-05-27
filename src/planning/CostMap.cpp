#include <planning/CostMap.h>
#include <planning/Solver.h>
#include <environment/RewardMap.h>


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


void CostMap::setCostMap()
{
	dwl::environment::Cell c;
	std::vector<dwl::environment::Cell> cell;
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -1.5;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);
	c.cell_key.grid_id.key[0] = 1001;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -2;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);
	c.cell_key.grid_id.key[0] = 999;
	c.cell_key.grid_id.key[1] = 1000;
	c.reward = -3;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 1001;
	c.reward = -4;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 999;
	c.reward = -5;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);
	c.cell_key.grid_id.key[0] = 1000;
	c.cell_key.grid_id.key[1] = 998;
	c.reward = -6;
	std::cout << "vertex = " << gridmap_.gridmapKeyToVertex(c.cell_key.grid_id) << std::endl;
	cell.push_back(c);






	unsigned long int vertex_id, edge_id;
	double cost;
	for (int i = 0; i < cell.size(); i++) {
		vertex_id = gridmap_.gridmapKeyToVertex(cell[i].cell_key.grid_id);
		unsigned short int vertex_x = cell[i].cell_key.grid_id.key[0];
		unsigned short int vertex_y = cell[i].cell_key.grid_id.key[1];

		// Searching the closed neighbors around 5-neighboring area
		bool is_found_neighbour_positive_x = false, is_found_neighbour_negative_x = false;
		bool is_found_neighbour_positive_y = false, is_found_neighbour_negative_y = false;
		for (int r = 1; r <= 3; r++) {
			for (int j = 0; j < cell.size(); j++) {
				unsigned short int edge_x = cell[j].cell_key.grid_id.key[0];
				unsigned short int edge_y = cell[j].cell_key.grid_id.key[1];

				// Getting the values of the edge
				edge_id = gridmap_.gridmapKeyToVertex(cell[j].cell_key.grid_id);
				cost = - cell[j].reward;



				// Searching the neighbour in the positive x-axis
				if ((vertex_x + r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_positive_x)) {
					std::cout << "v=" << vertex_id << " | e=" << edge_id << " x+ | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_positive_x = true;
				}

				// Searching the neighbour in the negative x-axis
				if ((vertex_x - r == edge_x) && (vertex_y == edge_y) && (!is_found_neighbour_negative_x)) {
					std::cout << "v=" << vertex_id << " | e=" << edge_id << " x- | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_negative_x = true;
				}

				// Searching the neighbour in the positive y-axis
				if ((vertex_y + r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_positive_y)) {
					std::cout << "v=" << vertex_id << " | e=" << edge_id << " y+ | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_positive_y = true;
				}

				// Searching the neighbour in the negative y-axis
				if ((vertex_y - r == edge_y) && (vertex_x == edge_x) && (!is_found_neighbour_negative_y)) {
					std::cout << "v=" << vertex_id << " | e=" << edge_id << " y- | r=" << r << std::endl;
					cost_map_[vertex_id].push_back(Edge(edge_id, cost));
					is_found_neighbour_negative_y = true;
				}
			}
		}
	}






/*
	cost_map_[-1].push_back(Edge(1,  79.83));
	cost_map_[-1].push_back(Edge(5,  81.15));
	cost_map_[1].push_back(Edge(-1,  79.75));
	cost_map_[1].push_back(Edge(2,  39.42));
	cost_map_[1].push_back(Edge(3, 103.00));
	cost_map_[2].push_back(Edge(1,  38.65));
	cost_map_[3].push_back(Edge(1, 102.53));
	cost_map_[3].push_back(Edge(5,  61.44));
	cost_map_[3].push_back(Edge(6,  96.79));
	cost_map_[4].push_back(Edge(5, 133.04));
	cost_map_[5].push_back(Edge(-1,  81.77));
	cost_map_[5].push_back(Edge(3,  62.05));
	cost_map_[5].push_back(Edge(4, 134.47));
	cost_map_[5].push_back(Edge(6,  91.63));
	cost_map_[6].push_back(Edge(3,  97.24));*/
}


void CostMap::get(AdjacencyMap& state)
{
	state = cost_map_;
}


} //@namespace planning

} //@namespace dwl
