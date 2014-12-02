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
	if (is_set_adjacency_model_) {
		printf(RED "Could not computed the shortest path because "
				"it is required to defined an adjacency model\n" COLOR_RESET);
		return false;
	}

	// Computing the shortest path
	CostMap min_cost;
	PreviousVertex previous;

//	if (compute_whole_adjacency_map_) {
		// Computing adjacency map
		AdjacencyMap adjacency_map;
//		adjacency_->computeAdjacencyMap(adjacency_map, source, target);

		// Computing the path according to A-star algorithm
//		findShortestPath(min_cost, previous, source, target, adjacency_map);
//	} else {
		findShortestPath(min_cost, previous, source, target);
//	}

	policy_ = previous;
	total_cost_ = min_cost[target];

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

void Dijkstrap::findShortestPath(CostMap& g_cost, PreviousVertex& previous, Vertex source, Vertex target) //TODO
{
	CostMap f_cost;

	// Setting the initial time
	time_started_ = clock();

	// Number of expansions
	expansions_ = 0;

	// Definiting the set of nodes already evaluated (openset), the set of
	// tentative nodes to be evaluated (openset), and ordered openset queue
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > openset_queue;
	std::map<Vertex, bool> openset;
	std::map<Vertex, bool> closedset;

	// Cost from start along best known path.
	g_cost[source] = 0;

	// Estimated total cost from start to goal
	f_cost[source] = g_cost[source];

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

			if (closedset.count(neighbor) > 0)
				continue;

			Weight tentative_g_cost = g_cost[current] + weight;
			if ((openset.count(neighbor) == 0) || (tentative_g_cost < g_cost[neighbor])) {
				previous[neighbor] = current;
				g_cost[neighbor] = tentative_g_cost;
				f_cost[neighbor] = g_cost[neighbor];

				if (openset.count(neighbor) == 0) {
					openset[neighbor] = true;
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost[neighbor], neighbor));
				}
			}
		}
		expansions_++;
	}
}

} //@namespace planning

} //@namespace dwl
