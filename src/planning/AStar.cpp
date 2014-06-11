#include <planning/AStar.h>


namespace dwl
{

namespace planning
{


AStar::AStar()
{
	name_ = "A-start";
	is_graph_searching_algorithm_ = true;
}


AStar::~AStar()
{

}


bool AStar::init()
{
	return true;
}


bool AStar::compute(SolverInterface solver_interface)
{
	GraphSearching solver = solver_interface.searcher;
	VertexCost min_cost;
	PreviousVertex previous;

	// Computing the path according to Dijkstrap algorithm
	findShortestPath(solver.source, solver.target, solver.adjacency_map, min_cost, previous);
	previous_ = previous;
	total_cost_ = min_cost[solver.target];

	return true;
}


void AStar::findShortestPath(Vertex source, Vertex target, AdjacencyMap adjacency_map, VertexCost& g_cost, PreviousVertex& previous)
{
	VertexCost f_cost;

	// Definiting the set of nodes already evaluated (openset), the set of tentative nodes to be evaluated (openset), and ordered openset queue
	std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > openset_queue;
	std::map<Vertex, bool> openset;
	std::map<Vertex, bool> closedset;

	// Cost from start along best known path.
	g_cost[source] = 0;

	// Estimated total cost from start to goal
	f_cost[source] = g_cost[source] + heuristicCostEstimate(source, target); //TODO

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost[source], source));
	openset[source] = true;

	while (!openset.empty()) {
		Vertex current = openset_queue.begin()->second;

		// Checking if it is getted the target
		if (current == target) {
			// reconstruct path //TODO
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
				f_cost[neighbor] = g_cost[neighbor] + heuristicCostEstimate(neighbor, target);

				if (openset.find(neighbor)->second == false) {
					openset[neighbor] = true;
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost[neighbor], neighbor));
				}
			}
		}
	}
}


double AStar::heuristicCostEstimate(Vertex source, Vertex target)
{
	environment::PlaneGrid gridmap(0.04, 0.04);
	Eigen::Vector2d source_position = gridmap.vertexToCoord(source);
	Eigen::Vector2d target_position = gridmap.vertexToCoord(target);

	double distance = (target_position - source_position).squaredNorm();
	double dy = target_position(1) - source_position(1);
	//double dx = target_position(0) - source_position(0);
	//double heading = abs(atan(dy / dx));


	return 0.5 * (0.8 * distance + 0.2 * abs(dy));
}




} //@namespace planning

} //@namespace dwl
