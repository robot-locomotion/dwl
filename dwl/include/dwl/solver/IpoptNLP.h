#ifndef DWL__SOLVER__IPOPT_NLP__H
#define DWL__SOLVER__IPOPT_NLP__H

#include <dwl/solver/OptimizationSolver.h>
#include <dwl/solver/IpoptWrapper.h>
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
		 * @brief Set the ipopt configuration parameters from a yaml file
		 * @param std::string Filename
		 */
		void setFromConfigFile(std::string filename);

		/**
		 * @brief Sets the printing level uses during the Ipopt computation
		 * @param int Printing level
		 */
		void setPrintLevel(int print_level);

		/**
		 * @brief Sets the printing frequency iteration uses during the Ipopt
		 * computation
		 * @param int Printing frequency iteration
		 */
		void setPrintFrequencyIteration(int print_freq_iter);

		/**
		 * @brief Sets the name and abilities the output file generation
		 * @param std::string Filename for output file
		 */
		void setFilename(std::string filename);

		/**
		 * @brief Sets the printing level for the report
		 * @param int Printing level
		 */
		void setFilePrintLevel(int file_print_level);

		/**
		 * @brief Sets the convergence tolerance
		 * @param double Convergence tolerance
		 */
		void setConvergenceTolerance(double tolerance);

		/**
		 * @brief Sets the maximum allocated number of iterations
		 * @param int Maximum number of iterations
		 */
		void setMaxIteration(int max_iter);

		/**
		 * @brief Sets the dual infeasibility tolerance
		 * @param double Tolerance value
		 */
		void setDualInfeasibilityTolerance(double tolerance);

		/**
		 * @brief Sets the constraint violation tolerance
		 * @param double Tolerance value
		 */
		void setConstraintViolationTolerance(double tolerance);

		/**
		 * @brief Sets the complementary violation tolerance
		 * @param double Tolerance value
		 */
		void setComplementaryTolerance(double tolerance);

		/**
		 * @brief Sets the acceptable convergence tolerance
		 * @param double Tolerance value
		 */
		void setAcceptableConvergenceTolerance(double tolerance);

		/**
		 * @brief Sets the acceptable number of iterations
		 * @param int Acceptable number of iterations
		 */
		void setAcceptableIterations(int iterations);

		/**
		 * @brief Sets the barrier method
		 * @param std::string Barrier method
		 */
		void setMuStrategy(std::string mu_strategy);

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

		/** @brief Label that indicates if it's initialized the solver */
		bool initialized_;

		/** @brief Printing level uses during the Ipopt computation */
		int print_level_;

		/** @brief Printing frequency uses during the Ipopt computation */
		int print_freq_iter_;

		/** @brief Indicates if the user request for a report */
		bool outfile_;

		/** @brief Filename that record the Ipopt computation report */
		std::string filename_;

		/** @brief Printing level for the Ipopt report */
		int file_print_level_;

		/** @brief Convergence tolerance */
		double convergence_tol_;

		/** @brief Maximum number of allocated iterations */
		int max_iter_;

		/** @brief Dual infeasibility tolerance */
		double dual_inf_tol_;

		/** @brief Constraint violation tolerance */
		double constr_viol_tol_;

		/** @brief Complementary violation tolerance */
		double compl_viol_tol_;

		/** @brief Acceptable tolerance */
		double acceptable_tol_;

		/** @brief Acceptable number of iterations before triggering termination */
		int acceptable_iter_;

		/** @brief Barrier method */
		std::string mu_strategy_;
};

} //@namespace solver
} //@namespace dwl

#endif
