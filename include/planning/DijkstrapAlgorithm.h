#ifndef DWL_DijktraAlgorithm_H
#define DWL_DijktraAlgorithm_H

#include <planning/Solver.h>


namespace dwl
{

namespace planning
{

class DijkstrapAlgorithm : public dwl::planning::Solver
{
	public:
		/** @brief Constructor function */
		DijkstrapAlgorithm();

		/** @brief Destructor function */
		~DijkstrapAlgorithm();

		bool init();
		bool compute(Eigen::MatrixXd& solution);
};

} //@namespace planning

} //@namespace dwl

#endif
