#include <planning/Solver.h>


namespace dwl
{

namespace planning
{


Solver::Solver() : environment_(NULL), is_graph_searching_algorithm_(false), total_cost_(0), is_settep_adjacency_model_(false)
{

}


Solver::~Solver()
{

}


void Solver::setTerrainInformation(std::vector<Cell> reward_map)
{
	if (is_settep_adjacency_model_) {
		environment_->setEnvironmentInformation(reward_map);
	} else
		printf(RED "Could not set the terriain information because it is required to defined an adjacency model \n" COLOR_RESET);
}


void Solver::setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model)
{
	printf(BLUE "Setting the %s adjacency model \n" COLOR_RESET, adjacency_model->getName().c_str());
	is_settep_adjacency_model_ = true;
	environment_ = adjacency_model;
}


std::list<Vertex> Solver::getShortestPath(Vertex target)
{
	std::list<Vertex> path;
	if (is_graph_searching_algorithm_) {
		PreviousVertex::iterator prev;
		Vertex vertex = target;
		path.push_front(vertex);
		while((prev = previous_.find(vertex)) != previous_.end()) {
			vertex = prev->second;
			path.push_front(vertex);
		}
	} else {
		printf(YELLOW "Could not get the shortest path because the %s is not a graph-searchin algorithm\n" COLOR_RESET, name_.c_str());
	}

	return path;
}


double Solver::getMinimumCost()
{
	return total_cost_;
}


std::string dwl::planning::Solver::getName()
{
	return name_;
}


} //@namespace planning

} //@namespace dwl
