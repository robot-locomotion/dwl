#ifndef DWL__OCP__PREVIEW_OPTIMIZATION__H
#define DWL__OCP__PREVIEW_OPTIMIZATION__H

#include <dwl/model/OptimizationModel.h>
#include <dwl/simulation/PreviewLocomotion.h>

namespace dwl
{

namespace ocp
{

/**
 * @class PreviewOptimization
 * @brief A preview optimization problem requires information of "modeling" constraint and "goals"
 * cost functions. A modeling constraints establishes the desired locomotion preview such as
 * stability constraints (i.e. CoP/ZMP inside the support region). These constraints could
 * implemented as soft or hard constraints. On the other hand, a goal cost evaluates the performance
 * of a specific behavior preview such as: stance duration, step distance or smoothing of CoM.
 */
class PreviewOptimization : public model::OptimizationModel
{
	public:
		/** @brief Constructor function */
		PreviewOptimization();

		/** @brief Destructor function */
		~PreviewOptimization();

		/** @brief Initializes the optimization model, i.e. the dimensions of the optimization
		 * vectors */
		void init(bool only_soft_constraints);

		/**
		 * @brief Sets the initial sequence of preview control parameters
		 * @param const simulation::PreviewControl& Initial sequence of preview
		 * control parameters
		 */
		void setStartingPreviewControl(const simulation::PreviewControl& control);

		/**
		 * @brief Gets the starting point of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full initial point
		 */
		void getStartingPoint(Eigen::Ref<Eigen::VectorXd> full_initial_point);

		/**
		 * @brief Evaluates the bounds of the problem
		 * @param Eigen::Ref<Eigen::VectorXd> Full state lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full state upper bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint lower bound
		 * @param Eigen::Ref<Eigen::VectorXd> Full constraint upper bound
		 */
		void evaluateBounds(Eigen::Ref<Eigen::VectorXd> full_state_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_state_upper_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_lower_bound,
							Eigen::Ref<Eigen::VectorXd> full_constraint_upper_bound);
		/**
		 * @brief Evaluates the constraint function given a current decision state
		 * @param Eigen::Ref<Eigen::VectorXd> Constraint vector
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateConstraints(Eigen::Ref<Eigen::VectorXd> full_constraint,
								 const Eigen::Ref<const Eigen::VectorXd>& decision_var);

		/**
		 * @brief Evaluates the cost function given a current decision state
		 * @param double& Cost value
		 * @param const Eigen::Ref<const Eigen:VectorXd>& Decision vector
		 */
		void evaluateCosts(double& cost,
						   const Eigen::Ref<const Eigen::VectorXd>& decision_var);

		/**
		 * @brief Evaluates the solution from an optimizer
		 * @param const Eigen::Ref<const Eigen::VectorXd>& Solution vector
		 * @return WholeBodyTrajectory& Returns the whole-body trajectory solution
		 */
		WholeBodyTrajectory& evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution);


	private:
		double stepTimeCost(const simulation::PreviewControl& preview_control);
		double comAccelerationCost(const simulation::PreviewControl& preview_control);
		simulation::PreviewLocomotion preview_;
};

} //@namespace ocp
} //@namespace dwl


#endif
