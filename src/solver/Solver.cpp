#include <solver/Solver.h>


namespace dwl
{

namespace solver
{

Solver::Solver() : robot_(NULL), environment_(NULL), adjacency_(NULL), is_graph_searching_algorithm_(false),
		is_optimization_algorithm_(false),total_cost_(std::numeric_limits<double>::max()), time_started_(clock()),
		is_set_adjacency_model_(false)
{

}


Solver::~Solver()
{
	delete adjacency_;
}


void Solver::reset(robot::Robot* robot, environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot and environment information in the %s solver\n" COLOR_RESET,
			getName().c_str());
	robot_ = robot;
	environment_ = environment;

	if (!(is_graph_searching_algorithm_) && (!is_set_adjacency_model_)) {
		printf(YELLOW "Warning: Could not be set the robot and environment information in the adjacency model \n"
				COLOR_RESET);
		return;
	}

	adjacency_->reset(robot, environment);
}


void Solver::setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model)
{
	printf(BLUE "Setting the %s adjacency model in the %s solver\n" COLOR_RESET,
			adjacency_model->getName().c_str(), getName().c_str());
	adjacency_ = adjacency_model;
	is_set_adjacency_model_ = true;
}


std::list<Vertex> Solver::getShortestPath(Vertex source, Vertex target)
{
	std::list<Vertex> path;
	if (is_graph_searching_algorithm_) {
		PreviousVertex::iterator prev;
		Vertex vertex = target;
		path.push_front(vertex);
		while ((prev = policy_.find(vertex)) != policy_.end()) {
			vertex = prev->second;
			path.push_front(vertex);
			if (vertex == source)
				break;
		}
	} else {
		printf(YELLOW "Could not get the shortest path because the %s is not a graph-searching algorithm\n"
				COLOR_RESET, name_.c_str());
	}

	return path;
}


double Solver::getMinimumCost()
{
	return total_cost_;
}


std::string Solver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
