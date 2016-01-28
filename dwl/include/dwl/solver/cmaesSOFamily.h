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

enum CMAESFamily {CMAES, IPOP, BIPOP, ACMAES, AIPOP, ABIPOP, SEPCMAES,
	SEPIPOP, SEPBIPOP, SEPACMAES, SEPAIPOP, SEPABIPOP, VDCMA, VDIPOPCMA,
	VDBIPOPCMA};

template<class TScaling=libcmaes::linScalingStrategy>
class cmaesSOFamily : public OptimizationSolver
{
	public:
		/** @brief Constructor function */
		cmaesSOFamily();

		/** @brief Destructor function */
		~cmaesSOFamily();

		/**
		 * @brief Set the ipopt configuration parameters from a yaml file
		 * @param std::string Filename
		 */
		void setFromConfigFile(std::string filename);

		/** @brief Sets the desired CMA-ES family to use */
		void setFamily(enum CMAESFamily alg);

		/** @brief Sets the allowed number of iterations */
		void setAllowedNumberofIterations(int max_iter);

		/** @brief Sets the allowed number of functions evaluations */
		void setAllowedNumberOfFunctionEvalutions(int max_fevals);

		/** @brief Sets the initial distribution */
		void setInitialDistribution(double sigma);

		/**
		 * @brief Sets the elitism. These are the type of elitism:
		 * 	0: no elitism
		 * 	1: elitism: reinjects the best-ever seen solution
		 * 	2: initial elitism: reinject x0 as long as it is not improved upon
		 * 	3: initial elitism on restart: restart if best encountered solution is not the
		 * 	   the final solution and reinjects the best solution until the population
		 * 	   has better fitness, in its majority
		 */
		void setElitism(int elitism);

		/**
		 * @brief Initialization of the NLP solver using Ipopt
		 * @return True if was initialized
		 */
		bool init();

		/**
		 * @brief Computes a solution of an SOP given a defined computation time
		 * @param double Allocated computation time in seconds
		 * @return True if it was computed a solution
		 */
		bool compute(double allocated_time_secs);


	private:
		/**
		 * @brief Wraps the fitness (objective) function
		 * @param const double* State array
		 * @param const int& Dimension of the state array
		 */
		double fitnessFunction(const double* x,
							   const int& n);

		/** @brief Fitness function wrapper */
		libcmaes::FitFunc fitness_;

		/** @brief Pointer to the CMA-ES configuration parameters */
		libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,TScaling>>* cmaes_params_;

		/** @brief Warm point for the initialization of the optimization */
		Eigen::VectorXd warm_point_;

		/** @brief Initial distribution */
		double sigma_;
};

} //@namespace solver
} //@namespace dwl

#include <dwl/solver/impl/cmaesSOFamily.hcc>

#endif
