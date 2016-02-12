#ifndef DWL__OCP__PREVIEW_OPTIMIZATION__H
#define DWL__OCP__PREVIEW_OPTIMIZATION__H

#include <dwl/model/OptimizationModel.h>
#include <dwl/ocp/SupportPolygonConstraint.h>
#include <dwl/ocp/PointConstraint.h>
#include <dwl/simulation/PreviewLocomotion.h>


namespace dwl
{

namespace ocp
{

struct PreviewVariables
{
	double duration;
	Eigen::Vector2d cop_shift;
	double length_shift;
	Eigen::Vector2d foothold_shift;
	double head_acc;
};

struct PreviewBounds
{
	PreviewVariables lower;
	PreviewVariables upper;
};

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
		 * @brief Sets the actual whole-body state
		 * @param const WholeBodyState& Actual whole-body state
		 */
		void setActualWholeBodyState(const WholeBodyState& state);

		/**
		 * @brief Sets the initial sequence of preview control parameters
		 * @param const simulation::PreviewControl& Initial sequence of preview
		 * control parameters
		 */
		void setStartingPreviewControl(const simulation::PreviewControl& control);

		/**
		 * @brief Sets the bounds of the preview optimization
		 * @param PreviewBound Lower bound
		 * @param PreviewBound Upper bound
		 */
		void setBounds(PreviewBounds bounds);

		/**
		 * @brief Sets the step cost weights
		 * @param double Step duration weight
		 * @param double Step distance weight
		 */
		void setStepWeights(double time_weight,
							double distance_weight);

		/**
		 * @brief Sets the CoM acceleration weights
		 * @param double Weight in the x-axis acceleration
		 * @param double Weight in the y-axis acceleration
		 * @param double Weight in the z-axis acceleration
		 */
		void setComAccelerationWeight(double x_weight,
									  double y_weight,
									  double z_weight);

		/**
		 * @brief Sets the CoP stability weight
		 * @param double CoP stability weight
		 */
		void setCopStabilityWeight(double weight);

		/**
		 * @brief Sets the preview model weight
		 * @param doublw Preview model weight
		 */
		void setPreviewModelWeight(double weight);

		/**
		 * @brief Sets the reference step properties
		 * @param double Step duration
		 * @param double Step distance
		 */
		void setDesiredStep(double duration,
							double distance);

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

		/** @brief Returns the preview system pointer */
		simulation::PreviewLocomotion* getPreviewSystem();

		/** @brief Returns the reduced-body trajectory */
		ReducedBodyTrajectory& getReducedTrajectory();


	private:
		double stepCost(const simulation::PreviewTrajectory& preview_trans,
						const simulation::PreviewState& actual_state,
						const simulation::PreviewControl& preview_control);
		double comEnergyCost(const simulation::PreviewState& actual_state,
							 const simulation::PreviewControl& preview_control);
		double copStabilitySoftConstraint(const simulation::PreviewTrajectory& preview_trans,
										  const simulation::PreviewState& actual_state,
										  const simulation::PreviewControl& preview_control);
		double previewModelSoftConstraint(const simulation::PreviewTrajectory& preview_trans,
										  const simulation::PreviewControl& preview_control);
		double targetLegLengthSoftConstraint(const simulation::PreviewTrajectory& preview_trans,
											 const simulation::PreviewControl& preview_control);

		/** @brief Returns the control dimension of the preview schedule */
		unsigned int getControlDimension();

		/**
		 * @brief Gets the control dimension of the preview schedule
		 * @param const unsigned int& Phase index
		 * @return Returns the control dimension of the preview schedule
		 */
		unsigned int getParamsDimension(const unsigned int& phase);

		/**
		 * @brief Converts the generalized control vector to preview control
		 * @param simulation::PreviewControl& Preview control
		 * @param const Eigen::VectorXd& Generalized control vector
		 */
		void toPreviewControl(simulation::PreviewControl& preview_control,
							  const Eigen::VectorXd& generalized_control);

		/**
		 * @brief Converts the preview control to generalized control vector
		 * @param Eigen::VectorXd& Generalized control vector
		 * @param const simulation::PreviewControl& Preview control
		 */
		void fromPreviewControl(Eigen::VectorXd& generalized_control,
								const simulation::PreviewControl& preview_control);

		ocp::SupportPolygonConstraint polygon_constraint_;
		ocp::PointConstraint preview_constraint_;
		simulation::PreviewLocomotion preview_;
		simulation::PreviewState actual_state_;

		PreviewBounds bounds_;

		ReducedBodyTrajectory reduced_traj_;
		simulation::PreviewTrajectory preview_transitions_;

		double desired_step_duration_;
		double desired_step_distance_;
		double step_time_weight_;
		double step_dist_weight_;
		Eigen::Vector3d acc_int_weight_;

		rbd::BodySelector feet_;
		unsigned int num_feet_;

		/** @brief Preview schedule */
		simulation::PreviewSchedule schedule_;

		/** @brief Number of phases of the schedule */
		unsigned int phases_;

		/** @brief Indicates it was set the schedule */
		bool set_schedule_;

		/** @brief Indicates it was set the bounds */
		bool is_bound_;
};

} //@namespace ocp
} //@namespace dwl


#endif
