#ifndef DWL__SOLVER__IPOPT_NLP__H
#define DWL__SOLVER__IPOPT_NLP__H

#include <solver/Solver.h>
#include <solver/IpoptWrapper.h>
#include <IpIpoptApplication.hpp>


namespace dwl
{

namespace solver
{

class IpoptNLP : public Solver
{
	public:
		/** @brief Constructor function */
		IpoptNLP();

		/** @brief Destructor function */
		~IpoptNLP();

		/**
		 * @brief Iinitialization of the NLP solver using Ipopt
		 * @return True if was initialized
		 */
		bool init();

		/**
		 * @brief Computes a solution of an NLP given a defined computation time
		 * @oaram double Computation time
		 * @return True if it was computed a solution
		 */
		bool compute(double computation_time = std::numeric_limits<double>::max());

		/**
		 * @brief Gets the Ipopt wrapper
		 * @return The Ipopt wrapper
		 */
		solver::IpoptWrapper& getIpopt();


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
