#ifndef DWL__SOLVER__QUADPROGPP_QP__H
#define DWL__SOLVER__QUADPROGPP_QP__H

#include <solver/OptimizationSolver.h>
#include <solver/QuadProg++.h>
#include <time.h>


namespace dwl
{

namespace solver
{

class QuadProgQP : public OptimizationSolver
{
	public:
		/** @brief Constructor function */
		QuadProgQP();

		/** @brief Destructor function */
		~QuadProgQP();

		/**
		 * @brief Initialization of the NLP solver using Ipopt
		 * @return True if was initialized
		 */
		bool init();

		/**
		 * @brief Computes a solution of an NLP given a defined computation time
		 * @param double Allocated computation time in seconds
		 * @return True if it was computed a solution
		 */
		bool compute(double allocated_time_secs = std::numeric_limits<double>::max());
};

} //@namespace solver
} //@namespace dwl

#endif
