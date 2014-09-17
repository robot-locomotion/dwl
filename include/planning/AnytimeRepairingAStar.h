#ifndef DWL_AnytimeRepairingAStar_H
#define DWL_AnytimeRepairingAStar_H

#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

class AnytimeRepairingAStar : public Solver
{
	public:
		/** @brief Constructor function */
		AnytimeRepairingAStar();

		/** @brief Destructor function */
		~AnytimeRepairingAStar();

		/**
		 * @brief Initializes the ARA* algorithm
		 * @return bool Return true if ARA* algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Computes a shortest-path using ARA* algorithm
		 * @param dwl::Vertex source Source vertex
		 * @param dwl::Vertex target Target vertex
		 * @param double computation_time Allowed time for computing a solution (in seconds)
		 * @return bool Return true if it was computed a solution
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
		 * @param
		 */
		bool improvePath(SetQueue& openset_queue, Set& visitedset, Vertex target, double computation_time);

		/** @brief Initial inflation */
		double initial_inflation_;

		/** @brief Satisfied inflation */
		double satisfied_inflation_;

		/** @brief Decrease inflation rate in percentage */
		double decrease_inflation_rate_;

		/** @brief G cost */
		CostMap g_cost_;

		/** @brief Minimum f cost */
		double min_f_cost_;

		/** @brief number of expansions */
		int expansions_;

};

} //@namespace planning
} //@namespace dwl

#endif
