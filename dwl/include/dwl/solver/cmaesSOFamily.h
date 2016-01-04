#ifndef DWL__SOLVER__CMAESSOFAMILY__H
#define DWL__SOLVER__CMAESSOFAMILY__H

#include <dwl/solver/OptimizationSolver.h>
#pragma GCC system_header // This pragma turns off the warning messages in this file
#pragma message "Turning off the warning messages of libcmaes"
#include_next <cmaes.h>


namespace dwl
{

namespace solver
{

enum CMAESAlgorithms {CMAES, IPOP, BIPOP, ACMAES, AIPOP, ABIPOP, SEPCMAES,
	SEPIPOP, SEPBIPOP, SEPACMAES, SEPAIPOP, SEPABIPOP, VDCMA, VDIPOPCMA,
	VDBIPOPCMA};

class cmaesSOFamily : public OptimizationSolver
{
	public:
		cmaesSOFamily();
		~cmaesSOFamily();

		void setAlgorithm(enum CMAESAlgorithms alg);
		bool init();
		bool compute(double allocated_time_secs);


	private:
		double fitnessFunction(const double* x,
							   const int& n);
		void allAsSoftConstraints();
		libcmaes::CMAParameters<>* cmaes_params_;
};

} //@namespace solver
} //@namespace dwl


#endif
