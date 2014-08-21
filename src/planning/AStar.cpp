#include <planning/AStar.h>


namespace dwl
{

namespace planning
{

AStar::AStar() : compute_whole_adjacency_map_(false)
{
	name_ = "A-star";
	is_graph_searching_algorithm_ = true;
}


AStar::~AStar()
{

}


bool AStar::init()
{
	return true;
}


bool AStar::compute(Vertex source, Vertex target, double computation_time)
{
	if (is_set_adjacency_model_) {
		// Computing the shortest path
		CostMap min_cost;
		PreviousVertex previous;
		if (compute_whole_adjacency_map_) {
			// Computing adjacency map
			AdjacencyMap adjacency_map;
			adjacency_->computeAdjacencyMap(adjacency_map, source, target);

			// Computing the path according to A-star algorithm
			findShortestPath(min_cost, previous, source, target, adjacency_map);
		} else {
			findShortestPath(min_cost, previous, source, target);
		}

		previous_ = previous;
		total_cost_ = min_cost[target];
		std::cout << "Total cost = " << total_cost_ << std::endl;
	} else {
		printf(RED "Could not computed the shortest path because it is required to defined an adjacency model\n" COLOR_RESET);
		return false;
	}

	return true;
}


void AStar::findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target, AdjacencyMap adjacency_map)
{
	CostMap f_cost;

	// Defining the set of nodes already evaluated (openset), the set of tentative nodes to be evaluated (openset), and ordered openset queue
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > openset_queue;
	std::map<Vertex, bool> openset;
	std::map<Vertex, bool> closedset;

	// Cost from start along best known path.
	g_cost[source] = 0;

	// Estimated total cost from start to goal
	f_cost[source] = g_cost[source] + adjacency_->heuristicCostEstimate(source, target);

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost[source], source));
	openset[source] = true;
	while (!openset.empty()) {
		Vertex current = openset_queue.begin()->second;

		// Checking if it is getted the target
		if (current == target) {
			// Reconstructing path
			break;
		}

		// Deleting the current vertex to the openset list
		openset_queue.erase(openset_queue.begin());
		openset.erase(source);

		// Adding the current vertex to the closedset
		closedset[current] = true;

		// Visit each edge exiting in the current vertex
		for (std::list<Edge>::iterator edge_iter = adjacency_map[current].begin();
				edge_iter != adjacency_map[current].end();
				edge_iter++)
		{
			Vertex neighbor = edge_iter->target;
			Weight weight = edge_iter->weight;
			if (closedset.find(neighbor)->second == true)
				continue;

			Weight tentative_g_cost = g_cost[current] + weight;
			if ((openset.find(neighbor)->second == false) || (tentative_g_cost < g_cost[neighbor])) {
				previous[neighbor] = current;
				g_cost[neighbor] = tentative_g_cost;
				f_cost[neighbor] = g_cost[neighbor] + adjacency_->heuristicCostEstimate(neighbor, target);

				if (openset.find(neighbor)->second == false) {
					openset[neighbor] = true;
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost[neighbor], neighbor));
				}
			}
		}
	}
}


void AStar::findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target)
{
	CostMap f_cost;

	// Definiting the set of nodes already evaluated (openset), the set of tentative nodes to be evaluated (openset), and ordered openset queue
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > openset_queue;
	std::map<Vertex, bool> openset;
	std::map<Vertex, bool> closedset;

	// Cost from start along best known path.
	g_cost[source] = 0;

	// Estimated total cost from start to goal
	f_cost[source] = g_cost[source] + adjacency_->heuristicCostEstimate(source, target);

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost[source], source));
	openset[source] = true;
	int expansions = 0;
	while (!openset.empty()) {
		Vertex current = openset_queue.begin()->second;

		// Checking if it is getted the target
		if (adjacency_->isReachedGoal(target, current)) {
			previous[target] = current;
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
			if (closedset.find(neighbor)->second == true)
				continue;

			Weight tentative_g_cost = g_cost[current] + weight;
			if ((openset.find(neighbor)->second == false) || (tentative_g_cost < g_cost[neighbor])) {
				previous[neighbor] = current;
				g_cost[neighbor] = tentative_g_cost;
				f_cost[neighbor] = g_cost[neighbor] + adjacency_->heuristicCostEstimate(neighbor, target);

				if (openset.find(neighbor)->second == false) {
					openset[neighbor] = true;
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost[neighbor], neighbor));
				}
			}
		}
		expansions++;
	}
	std::cout << "Expansions = " << expansions << std::endl;
}

} //@namespace planning
} //@namespace dwl

