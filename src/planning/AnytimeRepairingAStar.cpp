#include <planning/AnytimeRepairingAStar.h>
#include <time.h>


namespace dwl
{

namespace planning
{

AnytimeRepairingAStar::AnytimeRepairingAStar() : initial_inflation_(50.0), satisfied_inflation_(1.0), expansions_(0), decrease_inflation_rate_(0.01)
{
	name_ = "Anytime Repairing A*";
	is_graph_searching_algorithm_ = true;
}


AnytimeRepairingAStar::~AnytimeRepairingAStar()
{

}


bool AnytimeRepairingAStar::init()
{
	return true;
}


bool AnytimeRepairingAStar::compute(Vertex source, Vertex target, double computation_time)
{
	//bool

	// Definiting the set of nodes already evaluated (openset), the set of tentative nodes to be evaluated (openset), and ordered openset queue
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
	f_cost_[source] = g_cost_[source] + satisfied_inflation_ * adjacency_->heuristicCostEstimate(source, target);

	// Adding the start vertex to the openset
	openset_queue.insert(std::pair<Weight, Vertex>(f_cost_[source], source));
	openset[source] = true;

	while ((satisfied_inflation_ > 1) && ((clock() - time_started_) < allocated_time_secs)) {

		// Computing a path with reuse of states values
		if (improvePath(openset_queue, visitedset, target, computation_time)) {
			satisfied_inflation_ = current_inflation;
		}

		// Decreasing the current inflation gain
		if (fabs(satisfied_inflation_ - current_inflation) < 0.0000001) {
			current_inflation *= (1 - decrease_inflation_rate_);
			if (current_inflation < 1)
				current_inflation = 1;
		}

		//std::cout << "Time of computation = " << (clock() - time_started_) / CLOCKS_PER_SEC << std::endl;
		//std::cout << "epsilon = " << satisfied_inflation_ << std::endl;
	}

	std::cout << "Expansions = " << expansions_ << std::endl;
	return true;
}


bool AnytimeRepairingAStar::improvePath(SetQueue& openset_queue, Set& visitedset, Vertex target, double computation_time)
{
	// Setting an empty closed set and inconsistent set
	Set closedset;
	SetQueue inconsistentset_queue;

	bool is_found_path = false;
	double allocated_time_secs = computation_time * (double) CLOCKS_PER_SEC;
	while ((!openset_queue.empty()) && ((clock() - time_started_) < allocated_time_secs) && (g_cost_[target] > openset_queue.begin()->first)) { // Note that f_cost[target] = g_cost[target]
		Vertex current = openset_queue.begin()->second;

		// Checking if it is getted the target
		if (adjacency_->isReachedGoal(target, current)) {
			previous_[target] = current;
			g_cost_[target] = g_cost_[current];
			is_found_path = true;
			continue;
		}

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

			if (visitedset.find(neighbor)->second == false) {
				g_cost_[neighbor] = std::numeric_limits<double>::max();
				visitedset[neighbor] = true;
			}

			Weight tentative_g_cost = g_cost_[current] + weight;
			if (tentative_g_cost < g_cost_[neighbor]) {
				previous_[neighbor] = current;
				g_cost_[neighbor] = tentative_g_cost;
				f_cost_[neighbor] = g_cost_[neighbor] + satisfied_inflation_ * adjacency_->heuristicCostEstimate(neighbor, target);

				if (closedset.find(neighbor)->second == false) {
					openset_queue.insert(std::pair<Weight, Vertex>(f_cost_[neighbor], neighbor));
				} else {
					inconsistentset_queue.insert(std::pair<Weight, Vertex>(f_cost_[neighbor], neighbor));
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
		double vertex = incons_iter->second;
		openset_queue.insert(std::pair<Weight, Vertex>(f_cost, vertex));
	}

	return is_found_path;
}

} //@namespace planning
} //@namespace dwl
