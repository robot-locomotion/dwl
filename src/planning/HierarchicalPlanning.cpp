#include <planning/HierarchicalPlanning.h>

#include <environment/PlaneGrid.h>
#include <environment/RewardMap.h>


namespace dwl
{

namespace planning
{


HierarchicalPlanning::HierarchicalPlanning()
{
	name_ = "hierarchical";
}


bool HierarchicalPlanning::init(BodyPose start, BodyPose goal)
{
	printf("Initialized the HierarchicalPlanning algortihm\n");

	//TODO

	solver_->init();

	return true;
}


bool HierarchicalPlanning::compute()
{
	printf("Computing the HierarchicalPlanning algorithm\n");

	SolverInterface solver;



	environment::PlaneGrid gridmap(0.04, 0.02);
	dwl::environment::CellKey cell_key;

	cell_key.grid_id.key[0] = 1000;
	cell_key.grid_id.key[1] = 1000;
	unsigned long int source_id = gridmap.gridmapKeyToVertex(cell_key.grid_id);

	cell_key.grid_id.key[0] = 1000;
	cell_key.grid_id.key[1] = 998;
	unsigned long int target_id = gridmap.gridmapKeyToVertex(cell_key.grid_id);
	solver.searcher.source = source_id;
	solver.searcher.target = target_id;

	solver_->compute(solver);

	std::list<Vertex> path = solver_->getShortestPath(target_id);
	std::cout << "Total cost to " << target_id << " from " << source_id << ": " << solver_->getMinimumCost() << std::endl;

	//std::cout << "Total cost to " << vertex_names[6] << ": " << solver_->getMinimumCost() << std::endl;
	std::list<Vertex>::iterator path_iter = path.begin();
	std::cout << "Path: ";
	for( ; path_iter != path.end(); path_iter++) {
		std::cout << *path_iter << " "; //vertex_names[*path_iter] << " ";
	}
	std::cout << std::endl;


	/*
	std::vector<std::string> vertex_names;

	vertex_names.push_back("Harrisburg");   // 0
	vertex_names.push_back("Baltimore");    // 1
	vertex_names.push_back("Washington");   // 2
	vertex_names.push_back("Philadelphia"); // 3
	vertex_names.push_back("Binghamton");   // 4
	vertex_names.push_back("Allentown");    // 5
	vertex_names.push_back("New York");     // 6*/

	return true;
}


} //@namespace planning

} //@namespace dwl
