#include <solver/AnytimeRepairingAStar.h>
#include <time.h>


namespace dwl
{

namespace solver
{

AnytimeRepairingAStar::AnytimeRepairingAStar(double initial_inflation) :
		initial_inflation_(initial_inflation), satisfied_inflation_(1.0),
		min_f_cost_(std::numeric_limits<double>::max()), expansions_(0)
{
	name_ = "Anytime Repairing A*";
}


AnytimeRepairingAStar::~AnytimeRepairingAStar()
{
	policy_.clear();
	g_cost_.clear();
}


bool AnytimeRepairingAStar::init()
{
	return true;
}


bool AnytimeRepairingAStar::compute(Vertex source,
									Vertex target,
									double computation_time)
{
	if (!is_set_adjacency_model_) {
		printf(RED "Could not computed the shortest path because "
				"it is required to defined an adjacency model\n" COLOR_RESET);
		return false;
	}

	// Defining the set of nodes already evaluated (openset), the set of tentative nodes
	// to be evaluated (openset), and ordered openset queue
	SetQueue openset_queue;
	Set openset, visitedset;

	// Setting the initial time
	time_started_ = clock();

	satisfied_inflation_ = initial_inflation_;
	double current_inflation = initial_inflation_;

	// Number of expansions
	expansions_ = 0;

	// Getting the allocated time for computing a solution
	double allocated_time_secs = computation_time * (double) CLOCKS_PER_SEC;

	// Setting the g cost of the start and goal state
	g_cost_[source] = 0;
	g_cost_[target] = std::numeric_limits<double>::max();

	// Estimated total cost from start to goal
	double f_cost = g_cost_[source] +
			satisfied_inflation_ * adjacency_->heuristicCost(source, target);

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost, source));
	openset[source] = true;

	// Computing the minimum f cost
	Vertex current_vertex = openset_queue.begin()->second;
	double current_f_cost = openset_queue.begin()->first;
	min_f_cost_ = current_f_cost +
			satisfied_inflation_ * adjacency_->heuristicCost(current_vertex, target);

	while ((satisfied_inflation_ > 1) && ((clock() - time_started_) < allocated_time_secs)) {
		// Computing a path with reuse of states values
		if (improvePath(openset_queue, visitedset, target, computation_time)) {
			// Decreasing the current inflation gain
			double next_inflation = min_f_cost_ / g_cost_[target];
			if (current_inflation > next_inflation)
				current_inflation = next_inflation;

			if (current_inflation < 1)
				current_inflation = 1;
			satisfied_inflation_ = current_inflation;

			total_cost_ = g_cost_[target];
		}
	}

	return true;
}


bool AnytimeRepairingAStar::improvePath(SetQueue& openset_queue,
										Set& visitedset,
										Vertex target,
										double computation_time)
{
	// Setting an empty closed set and inconsistent set
	Set closedset;
	SetQueue inconsistentset_queue;

	double allocated_time_secs = computation_time * (double) CLOCKS_PER_SEC;
	while ((!openset_queue.empty()) && ((clock() - time_started_) < allocated_time_secs)
			&& (g_cost_[target] > min_f_cost_)) {
		Vertex current = openset_queue.begin()->second;

		// Deleting the current vertex to the openset list
		openset_queue.erase(openset_queue.begin());

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

			if (visitedset.count(neighbor) == 0) {
				g_cost_[neighbor] = std::numeric_limits<double>::max();
				visitedset[neighbor] = true;
			}

			Weight tentative_g_cost = g_cost_[current] + weight;
			if (tentative_g_cost < g_cost_[neighbor]) {
				policy_[neighbor] = current;
				g_cost_[neighbor] = tentative_g_cost;
				double f_cost = g_cost_[neighbor] +
						satisfied_inflation_ * adjacency_->heuristicCost(neighbor, target);
				if (closedset.count(neighbor) == 0) {
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost, neighbor));
				} else {
					inconsistentset_queue.insert(std::pair<Weight, Vertex>(f_cost, neighbor));
				}
			}
		}

		expansions_++;
	}

	// Setting open set with all over consistent states
	for (SetQueue::iterator incons_iter = inconsistentset_queue.begin();
			incons_iter != inconsistentset_queue.end(); incons_iter++)
	{
		double f_cost = incons_iter->first;
		Vertex vertex = incons_iter->second;
		openset_queue.insert(std::pair<Weight, Vertex>(f_cost, vertex));
	}

	// Computing the minimum f cost
	Vertex current_vertex = openset_queue.begin()->second;
	double current_f_cost = openset_queue.begin()->first;
	min_f_cost_ = current_f_cost +
			satisfied_inflation_ * adjacency_->heuristicCost(current_vertex, target);

	return true;
}

} //@namespace solver
} //@namespace dwl
