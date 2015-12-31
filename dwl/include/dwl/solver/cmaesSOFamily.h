#ifndef DWL__SOLVER__CMAESSOFAMILY__H
#define DWL__SOLVER__CMAESSOFAMILY__H

#include <dwl/solver/OptimizationSolver.h>
#include <cmaes.h>


namespace dwl
{

namespace solver
{

enum CMAESAlgorithms {CMAES, IPOP, BIPOP, ACMAES, AIPOP, ABIPOP, SEPCMAES,
	SEPIPOP, SEPBIPOP, SEPACMAES, SEPAIPOP, SEPABIPOP};

class cmaesSOFamily : public OptimizationSolver
{
	public:
		cmaesSOFamily();
		~cmaesSOFamily();

		void setAlgorithm(enum CMAESAlgorithms alg);
		bool init();
		bool compute(double allocated_time_secs);
};

} //@namespace solver
} //@namespace dwl


#endif
