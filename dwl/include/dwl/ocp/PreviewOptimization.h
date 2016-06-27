#ifndef DWL__OCP__PREVIEW_OPTIMIZATION__H
#define DWL__OCP__PREVIEW_OPTIMIZATION__H

#include <dwl/model/OptimizationModel.h>
#include <dwl/ocp/SupportPolygonConstraint.h>
#include <dwl/ocp/PointConstraint.h>
#include <dwl/simulation/PreviewLocomotion.h>
#include <dwl/utils/CollectData.h>


namespace dwl
{

namespace ocp
{

struct PreviewVariables
{
	double duration;
	Eigen::Vector2d cop_shift;
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
 * @brief A preview optimization problem requires information of "modeling"
 * constraint and "goals" cost functions. A modeling constraints establishes
 * the desired locomotion preview such as stability constraints (i.e. CoP/ZMP
 * inside the support region). These constraints could implemented as soft or
 * hard constraints. On the other hand, a goal cost evaluates the performance
 * of a specific behavior preview such as: stance duration, step distance or
 * smoothing of CoM.
 */
class PreviewOptimization : public model::OptimizationModel
{
	public:
		/** @brief Constructor function */
		PreviewOptimization();

		/** @brief Destructor function */
		~PreviewOptimization();

		/** @brief Initializes the optimization model, i.e. the dimensions
		 * of the optimization vectors */
		void init(bool only_soft_constraints);

		/**
		 * @brief Sets the actual whole-body state
		 * @param const WholeBodyState& Actual whole-body state
		 */
		void setActualWholeBodyState(const WholeBodyState& state);

		/**
		 * @brief Sets the actual preview state
		 * @param const simulation::PreviewState& Actual preview state
		 */
		void setActualPreviewState(const simulation::PreviewState& state);

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
		 * @brief Sets the CoP stability constraint properties
		 * @param double CoP stability margin
		 * @param const SoftConstraintProperties& properties
		 */
		void setCopStabilityConstraint(double margin,
									   const SoftConstraintProperties& properties = SoftConstraintProperties());

		/**
		 * @brief Sets the preview model constraint properties
		 * @param double Minimum pendulum length
		 * @param double Maximum pendulum length
		 * @param double Maximum pitch angle
		 * @param double Maximum roll angle
		 * @param const SoftConstraintProperties& properties
		 */
		void setPreviewModelConstraint(double min_length,
									   double max_length,
									   double max_pitch,
									   double max_roll,
									   const SoftConstraintProperties& properties = SoftConstraintProperties());

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
		 */
		void setComAccelerationWeight(double x_weight,
									  double y_weight);

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

		/** @brief Gets the optimized preview control */
		simulation::PreviewControl& getFullPreviewControl();
		simulation::PreviewControl& getAppliedPreviewControl();

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

		/** @brief Returns the preview trajectory */
		simulation::PreviewTrajectory& getPreviewTrajectory();

		/** @brief Returns the reduced-body trajectory */
		ReducedBodyTrajectory& getReducedTrajectory();

		/**
		 * @brief Saves a state and preview control pairs
		 * @param simulation::PreviewState& Preview state
		 * @param simulation::PreviewControl& Preview control sequence
		 * @param std::string Filename
		 */
		void saveControl(simulation::PreviewState& state,
						 simulation::PreviewControl& control,
						 std::string filename);

		/**
		 * @brief Saves the solution of the preview optimization
		 * @param std::string Filename
		 */
		void saveSolution(std::string filename);


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
		double terrainCost(const simulation::PreviewTrajectory& preview_trans,
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
		 * @brief Orders the preview control given the actual state/phase
		 * @param simulation::PreviewControl& Preview control
		 * @param const simulation::PreviewControl& Nominal preview control (from
		 * decision order)
		 */
		void orderPreviewControl(simulation::PreviewControl& control,
								 const simulation::PreviewControl& nom_control);
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

		/** @brief Optimized preview control sequence */
		simulation::PreviewControl full_pc_;
		simulation::PreviewControl applied_pc_;

		/** @brief Starting preview control sequence */
		simulation::PreviewControl warm_control_;
		bool warm_point_;

		/** @brief Constraints of the preview optimization */
		ocp::SupportPolygonConstraint polygon_constraint_;
		ocp::PointConstraint preview_constraint_;
		double support_margin_;

		/** @brief Preview locomotion model */
		simulation::PreviewLocomotion preview_;

		/** @brief Preview schedule */
		simulation::PreviewSchedule schedule_;
		std::vector<unsigned int> phase_id_;

		/** @brief Actual preview state */
		simulation::PreviewState actual_state_;

		/** @brief Bounds of the preview optimization */
		PreviewBounds bounds_;

		/** @brief Preview trajectory and transitions */
		simulation::PreviewTrajectory preview_trajectory_;
		ReducedBodyTrajectory reduced_traj_;
		simulation::PreviewTrajectory preview_transitions_;

		/** @brief Desired states */
		double desired_step_duration_;
		double desired_step_distance_;
		double desired_yaw_;

		/** @brief Weights of the cost functions */
		double step_time_weight_;
		double step_dist_weight_;
		Eigen::Vector3d acc_int_weight_;
		double terrain_weight_;

		/** @brief Feet information */
		rbd::BodySelector feet_;
		unsigned int num_feet_;

		/** @brief Number of phases of the schedule */
		unsigned int phases_;

		/** @brief Number of steps in the schedule */
		unsigned int num_steps_;

		/** @brief Number of stance in the schedule */
		unsigned int num_stances_;

		/** @brief Number of applied control params */
		unsigned int num_controls_;

		/** @brief Indicates it was initialized the schedule */
		bool init_schedule_;

		/** @brief Indicates it was set the schedule */
		bool set_schedule_;

		/** @brief Indicates it was set the bounds */
		bool is_bound_;

		/** @brief For collecting data */
		utils::CollectData cdata_;
		bool collect_data_;
};

} //@namespace ocp
} //@namespace dwl


#endif
