#ifndef DWL__SOLVER__OPTIMIZATION_SOLVER__H
#define DWL__SOLVER__OPTIMIZATION_SOLVER__H

#include <model/OptimizationModel.h>
#include <utils/utils.h>


namespace dwl
{

namespace solver
{

/**
 * @class OptimizationSolver
 * @brief Abstract class for implementing different optimization solver such as Nonlinear
 * Optimization Programming (NLP), Quadratic Programming (QP), Sequential Quadratic Programming
 * (SQP), etc.
 */
class OptimizationSolver
{
	public:
		/** @brief Constructor function */
		OptimizationSolver();

		/** @brief Destructor function */
		virtual ~OptimizationSolver();

		/**
		 * @brief Abstract method for initialization of the solver
		 * @return True if was initialized
		 */
		virtual bool init() = 0;

		/**
		 * @brief Abstract method for computing a solution of an optimization problem
		 * @return True if it was computed a solution
		 */
		virtual bool compute(double computation_time = NO_BOUND) = 0;

		/**
		 * @brief Gets the optimization model
		 * @return the reference object of the optimization model
		 */
		model::OptimizationModel& getOptimizationModel();

		/**
		 * @brief Gets the whole-body trajectory computed by the optimizer
		 * @return const std::vector<LocomotionState> Whole-body trajectory
		 */
		const std::vector<LocomotionState>& getWholeBodyTrajectory();

		/**
		 * @brief Gets the name of the solver
		 * @return The name of the solver
		 */
		std::string getName();


	protected:
		/** @brief Name of the solver */
		std::string name_;

		/** @brief Optimization model */
		model::OptimizationModel model_;

		/** @brief Computed whole-body trajectory */
		std::vector<LocomotionState> locomotion_trajectory_;
};

} //@namespace solver
} //@namespace dwl


#endif
