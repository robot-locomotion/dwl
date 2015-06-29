#ifndef DWL_AnytimeRepairingAStar_H
#define DWL_AnytimeRepairingAStar_H

#include <solver/Solver.h>


namespace dwl
{

namespace solver
{

class AnytimeRepairingAStar : public Solver
{
	public:
		/** @brief Constructor function */
		AnytimeRepairingAStar(double initial_inflation = 3.0);

		/** @brief Destructor function */
		~AnytimeRepairingAStar();

		/**
		 * @brief Initializes the ARA* algorithm
		 * @return True if ARA* algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using ARA* algorithm
		 * @param Vertex Source vertex
		 * @param Vertex Target vertex
		 * @param double Allowed time for computing a solution (in seconds)
		 * @return True if it was computed a solution
		 */
		bool compute(Vertex source, Vertex target, double computation_time = std::numeric_limits<double>::max());

		/** @brief Defines a ordered queue according to the less weight */
		typedef std::set< std::pair<Weight, Vertex>, pair_first_less<Weight, Vertex> > SetQueue;

		/** @brief Defines a set of known vertex */
		typedef std::map<Vertex, bool> Set;


	private:
		/**
		 * @brief Improves the path according to inflation gain
		 * @param SetQueue& openset_queue Openset queue
		 * @param Set& visitedset Visited set
		 * @param Vertex target Target vertex
		 * @return True if the path was improved
		 */
		bool improvePath(SetQueue& openset_queue, Set& visitedset, Vertex target,	double computation_time);

		/** @brief Initial inflation */
		double initial_inflation_;

		/** @brief Satisfied inflation */
		double satisfied_inflation_;

		/** @brief G cost */
		CostMap g_cost_;

		/** @brief Minimum f cost */
		double min_f_cost_;

		/** @brief number of expansions */
		int expansions_;
};

} //@namespace solver
} //@namespace dwl

#endif
