#include <planning/Dijkstrap.h>


namespace dwl
{

namespace planning
{

Dijkstrap::Dijkstrap()
{
	name_ = "Dijkstrap";
	is_graph_searching_algorithm_ = true;
}


Dijkstrap::~Dijkstrap()
{

}


bool Dijkstrap::init()
{
	printf("Initialized the Dijkstrap algortihm\n");
	return true;
}


bool Dijkstrap::compute(Vertex source, Vertex target, double orientation)
{
	if (is_settep_adjacency_model_) {
		// Computing adjacency map
		AdjacencyMap adjacency_map;
		adjacency_->computeAdjacencyMap(adjacency_map, source, target, orientation);

		// Computing the path according to Dijkstrap algorithm
		CostMap min_cost;
		PreviousVertex previous;
		findShortestPath(min_cost, previous, source, adjacency_map);
		previous_ = previous;
		total_cost_ = min_cost[target];
	} else {
		printf(RED "Could not compute the shortest path because it is required to defined an adjacency model \n" COLOR_RESET);
		return false;
	}

	return true;
}


void Dijkstrap::findShortestPath(CostMap& min_cost, PreviousVertex& previous, Vertex source, AdjacencyMap adjacency_map)
{
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
		Vertex u = vertex_queue.begin()->second;
		vertex_queue.erase(vertex_queue.begin());

		// Visit each edge exiting u
		for (std::list<Edge>::iterator edge_iter = adjacency_map[u].begin();
			edge_iter != adjacency_map[u].end();
			edge_iter++)
		{
			Vertex v = edge_iter->target;
			Weight weight = edge_iter->weight;
			Weight distance_through_u = min_cost[u] + weight;
			if (distance_through_u < min_cost[v]) {
				vertex_queue.erase(std::pair<Weight, Vertex>(min_cost[v], v));

				min_cost[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.insert(std::pair<Weight, Vertex>(min_cost[v], v));
			}
		}
	}
}


} //@namespace planning

} //@namespace dwl
