#include <solver/AStar.h>


namespace dwl
{

namespace solver
{

AStar::AStar() : expansions_(0)
{
	name_ = "A-star";
}


AStar::~AStar()
{
	policy_.clear();
}


bool AStar::init()
{
	return true;
}


bool AStar::compute(Vertex source,
					Vertex target,
					double computation_time)
{
	if (!is_set_adjacency_model_) {
		printf(RED "Could not computed the shortest path because "
				"it is required to defined an adjacency model\n" COLOR_RESET);
		return false;
	}

	// Computing the shortest path
	findShortestPath(source, target);

	return true;
}


void AStar::findShortestPath(Vertex source,
							 Vertex target)
{
	// Defining the f_cost and g_cost
	CostMap f_cost;
	CostMap g_cost;

	// Setting the initial time
	time_started_ = clock();

	// Number of expansions
	expansions_ = 0;

	// Defining the set of nodes already evaluated (openset), the set of
	// tentative nodes to be evaluated (openset), and ordered openset queue
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > openset_queue;
	std::map<Vertex, bool> openset;
	std::map<Vertex, bool> closedset;

	// Cost from start along best known path.
	g_cost[source] = 0;

	// Estimated total cost from start to goal
	f_cost[source] = g_cost[source] + adjacency_->heuristicCost(source, target);

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost[source], source));
	openset[source] = true;
	double min_f_cost = std::numeric_limits<double>::max();
	while ((!openset.empty()) && (g_cost[target] < min_f_cost)) {
		Vertex current = openset_queue.begin()->second;

		// Checking if it is getted the target
		if (adjacency_->isReachedGoal(target, current)) {
			policy_[target] = current;
			g_cost[target] = g_cost[current];
			break;
		}

		// Deleting the current vertex to the openset list
		openset_queue.erase(openset_queue.begin());
		openset.erase(current);

		// Adding the current vertex to the closedset
		closedset[current] = true;

		// Visit each edge exiting in the current vertex
		std::list<Edge> successors;
		adjacency_->getSuccessors(successors, current);
		for (std::list<Edge>::iterator edge_iter = successors.begin();
						edge_iter != successors.end();
						edge_iter++)
		{
			Vertex neighbor = edge_iter->target;
			Weight weight = edge_iter->weight;

			if (closedset.count(neighbor) > 0)
				continue;

			Weight tentative_g_cost = g_cost[current] + weight;
			if ((openset.count(neighbor) == 0) || (tentative_g_cost < g_cost[neighbor])) {
				policy_[neighbor] = current;
				g_cost[neighbor] = tentative_g_cost;
				f_cost[neighbor] = g_cost[neighbor] + adjacency_->heuristicCost(neighbor, target);

				if (openset.count(neighbor) == 0) {
					openset[neighbor] = true;
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost[neighbor], neighbor));
				}
			}
		}
		expansions_++;

		// Computing the minimum f cost
		Vertex current_vertex = openset_queue.begin()->second;
		double current_f_cost = openset_queue.begin()->first;
		min_f_cost = current_f_cost + adjacency_->heuristicCost(current_vertex, target);
	}

	total_cost_ = g_cost[target];
}

} //@namespace solver
} //@namespace dwl

