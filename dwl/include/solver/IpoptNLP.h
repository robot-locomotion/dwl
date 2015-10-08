#ifndef DWL__SOLVER__IPOPT_NLP__H
#define DWL__SOLVER__IPOPT_NLP__H

#include <solver/OptimizationSolver.h>
#include <solver/IpoptWrapper.h>
#include <IpIpoptApplication.hpp>
#include <time.h>


namespace dwl
{

namespace solver
{

class IpoptNLP : public OptimizationSolver
{
	public:
		/** @brief Constructor function */
		IpoptNLP();

		/** @brief Destructor function */
		~IpoptNLP();

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


	private:
		/** @brief Ipopt wrapper */
		solver::IpoptWrapper ipopt_;

		/** @brief Smart pointer to the NLP */
		Ipopt::SmartPtr<Ipopt::TNLP> nlp_ptr_;

		/** @brief Smart pointer to an Ipopt application */
		Ipopt::SmartPtr<Ipopt::IpoptApplication> app_;
};

} //@namespace solver
} //@namespace dwl

#endif
