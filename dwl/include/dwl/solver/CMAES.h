#ifndef DWL__SOLVER__CMAES__H
#define DWL__SOLVER__CMAES__H

#include <dwl/solver/OptimizationSolver.h>
#include <cmaes.h>


namespace dwl
{

namespace solver
{

class CMAES : public OptimizationSolver
{
	public:
		CMAES();
		~CMAES();

		bool init();
		bool compute(double allocated_time_secs);
};

} //@namespace solver
} //@namespace dwl


#endif
