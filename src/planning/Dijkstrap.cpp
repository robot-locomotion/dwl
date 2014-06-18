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


bool Dijkstrap::compute(SolverInterface solver_interface)
{
	GraphSearching solver = solver_interface.searcher;

	if (is_settep_adjacency_model_) {
		// Computing adjacency map
		AdjacencyMap adjacency_map;
		environment_->computeAdjacencyMap(adjacency_map, solver.position);

		// Check if the start and goal position belong to the adjacency map, in negative case, these are added to the adjacency map
		environment_->checkStartAndGoalVertex(adjacency_map, solver.source, solver.target);

		// Computing the path according to Dijkstrap algorithm
		CostMap min_cost;
		PreviousVertex previous;
		findShortestPath(solver.source, adjacency_map, min_cost, previous);
		previous_ = previous;
		total_cost_ = min_cost[solver.target];
	} else {
		printf(RED "Could not compute the shortest path because it is required to defined an adjacency model \n" COLOR_RESET);
		return false;
	}

	return true;
}


void Dijkstrap::findShortestPath(Vertex source, AdjacencyMap adjacency_map, CostMap& min_cost, PreviousVertex& previous)
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





/*
const int num_nodes = 5;
enum nodes { A, B, C, D, E };
char name[] = "ABCDE";
edge edge_array[] = { edge(A, C), edge(B, B), edge(B, D), edge(B, E),
    edge(C, B), edge(C, D), edge(D, E), edge(E, A), edge(E, B)
};
int weights[] = { 1, 2, 1, 2, 7, 3, 1, 1, 1 };
int num_arcs = sizeof(edge_array) / sizeof(edge);

// Defining the dijkstrap graph
graph dijkstrap_graph(edge_array, edge_array + num_arcs, weights, num_nodes);

// Defining the weight map
boost::property_map<graph, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, dijkstrap_graph);

std::vector<vertex_descriptor> parent_vertex(num_vertices(dijkstrap_graph));
std::vector<int> cost_from_source(num_vertices(dijkstrap_graph));
vertex_descriptor source_vertex = vertex(A, dijkstrap_graph);

dijkstra_shortest_paths(dijkstrap_graph, source_vertex,
						predecessor_map(boost::make_iterator_property_map(parent_vertex.begin(), get(boost::vertex_index, dijkstrap_graph))).
						distance_map(boost::make_iterator_property_map(cost_from_source.begin(), get(boost::vertex_index, dijkstrap_graph))));


std::cout << "Distance(" << name[4] << ") = " << cost_from_source[4] << ",  Parent(" << name[4] << ") = " << name[parent_vertex[4]] << std::endl;*/
