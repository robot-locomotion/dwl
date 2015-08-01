#include <solver/Solver.h>


namespace dwl
{

namespace solver
{

SearchTreeSolver::SearchTreeSolver() : adjacency_(NULL),
		total_cost_(std::numeric_limits<double>::max()), time_started_(clock()),
		is_set_model_(false), is_set_adjacency_model_(false)
{

}


SearchTreeSolver::~SearchTreeSolver()
{
	delete adjacency_;
}


void SearchTreeSolver::reset(robot::Robot* robot,
				   environment::EnvironmentInformation* environment)
{
	printf(BLUE "Setting the robot and environment information in the %s solver\n" COLOR_RESET,
			getName().c_str());

	if (!is_set_adjacency_model_) {
		printf(YELLOW "WARNING: Could not be set the robot and environment information in the"
				"adjacency model \n" COLOR_RESET);
		return;
	}

	adjacency_->reset(robot, environment);
}


void SearchTreeSolver::setAdjacencyModel(environment::AdjacencyEnvironment* adjacency_model)
{
	printf(BLUE "Setting the %s adjacency model in the %s solver\n" COLOR_RESET,
			adjacency_model->getName().c_str(), getName().c_str());
	adjacency_ = adjacency_model;
	is_set_adjacency_model_ = true;
}


std::list<Vertex> SearchTreeSolver::getShortestPath(Vertex source, Vertex target)
{
	std::list<Vertex> path;
	PreviousVertex::iterator prev;
	Vertex vertex = target;
	path.push_front(vertex);
	while ((prev = policy_.find(vertex)) != policy_.end()) {
		vertex = prev->second;
		path.push_front(vertex);
		if (vertex == source)
			break;
	}

	return path;
}


double SearchTreeSolver::getMinimumCost()
{
	return total_cost_;
}


std::string SearchTreeSolver::getName()
{
	return name_;
}

} //@namespace solver
} //@namespace dwl
