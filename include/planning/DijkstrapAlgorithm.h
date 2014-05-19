#ifndef DWL_DijktraAlgorithm_H
#define DWL_DijktraAlgorithm_H

#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

/**
 * @class DijkstrapAlgorithm
 * @brief Class for solving a shortest-search problem using the Dijkstrap algorithm
 */
class DijkstrapAlgorithm : public dwl::planning::Solver
{
	public:
		/** @brief Constructor function */
		DijkstrapAlgorithm();

		/** @brief Destructor function */
		~DijkstrapAlgorithm();

		/**
		 * @brief Initialize the Dijkstrap algorithm
		 * @return bool Return true if Dijkstrap algorithm was initialized
		 */
		bool init();

		/**
		 * @brief Compute the shortest-path according to Dijkstrap algorithm
		 */
		bool compute(Eigen::MatrixXd& solution);
};

} //@namespace planning

} //@namespace dwl

#endif
