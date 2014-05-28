#include <planning/Solver.h>


namespace dwl
{

namespace planning
{


Solver::Solver() : is_graph_searching_algorithm_(false), total_cost_(0)
{

}


Solver::~Solver()
{

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
