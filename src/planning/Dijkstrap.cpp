#include <planning/Dijkstrap.h>


namespace dwl
{

namespace planning
{

Dijkstrap::Dijkstrap() : expansions_(0)
{
	name_ = "Dijkstrap";
	is_graph_searching_algorithm_ = true;
}


Dijkstrap::~Dijkstrap()
{
	policy_.clear();
}


bool Dijkstrap::init()
{
	printf("Initialized the Dijkstrap algortihm\n");
	return true;
}


bool Dijkstrap::compute(Vertex source, Vertex target, double computation_time)
{
	if (!is_set_adjacency_model_) {
		printf(RED "Could not computed the shortest path because it is required to defined an adjacency model\n" COLOR_RESET);
		return false;
	} else if (adjacency_->isLatticeRepresentation()) {
		printf(RED "Could not computed the shortest path because it is lattice representation\n" COLOR_RESET);
		return false;
	}

	// Computing adjacency map
	AdjacencyMap adjacency_map;
	adjacency_->computeAdjacencyMap(adjacency_map, source, target);

	// Computing the shortest path
	findShortestPath(source, target, adjacency_map);

	return true;
}


void Dijkstrap::findShortestPath(Vertex source, Vertex target, AdjacencyMap adjacency_map)
{
	// Defining the minimum cost
	CostMap min_cost;

	for (AdjacencyMap::iterator vertex_iter = adjacency_map.begin();
		vertex_iter != adjacency_map.end();
		vertex_iter++)
	{
		Vertex v = vertex_iter->first;
		min_cost[v] = std::numeric_limits<double>::infinity();
	}

	min_cost[source] = 0;
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > vertex_queue;
	for (AdjacencyMap::iterator vertex_iter = adjacency_map.begin();
		vertex_iter != adjacency_map.end();
		vertex_iter++)
	{
		Vertex v = vertex_iter->first;
		vertex_queue.insert(std::pair<Weight, Vertex>(min_cost[v], v));
	}

	while (!vertex_queue.empty()) {
		Vertex current = vertex_queue.begin()->second;
		vertex_queue.erase(vertex_queue.begin());

		// Checking if it is getted the target
		if (adjacency_->isReachedGoal(target, current)) {
			policy_[target] = current;
			min_cost[target] = min_cost[current];
			break;
		}

		// Visit each edge exiting u
		for (std::list<Edge>::iterator edge_iter = adjacency_map[current].begin();
			edge_iter != adjacency_map[current].end();
			edge_iter++)
		{
			Vertex neighbor = edge_iter->target;
			Weight weight = edge_iter->weight;
			Weight distance_through_current = min_cost[current] + weight;
			if (distance_through_current < min_cost[neighbor]) {
				vertex_queue.erase(std::pair<Weight, Vertex>(min_cost[neighbor], neighbor));

				min_cost[neighbor] = distance_through_current;
				policy_[neighbor] = current;
				vertex_queue.insert(std::pair<Weight, Vertex>(min_cost[neighbor], neighbor));
			}
		}
	}
}

} //@namespace planning
} //@namespace dwl
