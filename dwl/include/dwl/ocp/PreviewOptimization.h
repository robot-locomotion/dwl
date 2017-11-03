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

struct TerrainModel
{
	TerrainModel() : weight(0.), margin(std::numeric_limits<double>::max()),
			offset(0.) {}
	TerrainModel(double _weight,
				 double _margin,
				 double _offset) : weight(_weight), margin(_margin),
						 offset(_offset) {}

	double weight;
	double margin;
	double offset;
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
		 * @brief Sets the actual reduced-body state
		 * @param const ReducedBodyState& Actual reduced-body state
		 */
		void setActualReducedBodyState(const ReducedBodyState& state);

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
		 * @brief Sets the velocity commands cost weights
		 * @param double Velocity weight
		 */
		void setVelocityWeights(double velocity_weight);

		/**
		 * @brief Sets the CoT weight
		 * @param double Weight of the CoT
		 */
		void setCostOfTransportWeight(double weight);

		/**
		 * @brief Sets the terrain cost weight
		 * @param TerrainModel Terrain model
		 */
		void setTerrainModel(const TerrainModel& model);

		/**
		 * @brief Sets the reference velocity commands
		 * @param double Desired velocity in X
		 * @param double Desired velocity in Y
		 */
		void setVelocityCommand(double velocity_x,
								double velocity_y);

		/**
		 * @brief Gets the starting point of the problem
		 * @param double* Initial values for the decision variables, $x$
		 * @param int Number of the decision variables
		 */
		void getStartingPoint(double* decision, int decision_dim);

		/** @brief Gets the optimized preview control */
		simulation::PreviewControl& getFullPreviewControl();
		simulation::PreviewControl& getAppliedPreviewControl();

		/** @brief Returns the applied velocity command */
		simulation::VelocityCommand& getVelocityCommand();

		/**
		 * @brief Evaluates the bounds of the problem
		 * @param double* Lower bounds $x^L$ for $x$
		 * @param double* Upper bounds $x^L$ for $x$
		 * @param int Number of decision variables (dimension of $x$)
		 * @param double* Lower bounds of the constraints $g^L$ for $x$
		 * @param double* Upper bounds of the constraints $g^L$ for $x$
		 * @param int Number of constraints (dimension of $g(x)$)
		 */
		void evaluateBounds(double* decision_lbound, int decision_dim1,
							double* decision_ubound, int decision_dim2,
							double* constraint_lbound, int constraint_dim1,
							double* constraint_ubound, int constraint_dim2);
		/**
		 * @brief Evaluates the constraint function given a current decision state
		 * @param double* Array of constraint function values, $g(x)$
		 * @param int Number of constraint variables (dimension of $g(x)$)
		 * @param const double* Array of the decision variables, $x$, at which the constraint functions,
		 * $g(x)$, are evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		void evaluateConstraints(double* constraint, int constraint_dim,
						 	 	 const double* decision, int decision_dim);

		/**
		 * @brief Evaluates the cost function given a current decision state
		 * @param double& Value of the objective function ($f(x)$).
		 * @param const double* Array of the decision variables, $x$, at which the cost functions,
		 * $f(x)$, is evaluated
		 * @param int Number of decision variables (dimension of $x$)
		 */
		void evaluateCosts(double& cost,
						   const double* decision, int decision_dim);

		/**
		 * @brief Evaluates the solution from an optimizer
		 * @param const Eigen::Ref<const Eigen::VectorXd>& Solution vector
		 * @return WholeBodyTrajectory& Returns the whole-body trajectory solution
		 */
		WholeBodyTrajectory& evaluateSolution(const Eigen::Ref<const Eigen::VectorXd>& solution);

		/** @brief Returns the preview system pointer */
		simulation::PreviewLocomotion* getPreviewSystem();

		/** @brief Returns the whole-body trajectory */
		WholeBodyTrajectory& getWholeBodyTrajectory();

		/** @brief Returns the reduced-body trajectory */
		ReducedBodyTrajectory& getReducedBodyTrajectory();

		/** @brief Returns the reduced-body sequence */
		ReducedBodyTrajectory& getReducedBodySequence();

		/**
		 * @brief Saves a state and preview control pairs
		 * @param const PreviewData& Preview data
		 * @param std::string Filename
		 */
		void saveControl(const simulation::PreviewData& data,
						 std::string filename);

		/**
		 * @brief Saves the solution of the preview optimization
		 * @param std::string Filename
		 */
		void saveSolution(std::string filename);


	private:
		double velocityCost(const ReducedBodyState& terminal_state,
							const ReducedBodyState& actual_state,
							const simulation::PreviewControl& preview_control);
		double costOfTransport(const ReducedBodyState& terminal_state,
							   const ReducedBodyState& actual_state);
		double copStabilitySoftConstraint(const ReducedBodyTrajectory& phase_trans,
										  const ReducedBodyState& actual_state,
										  const simulation::PreviewControl& preview_control);
		double previewModelSoftConstraint(const ReducedBodyTrajectory& phase_trans,
										  const simulation::PreviewControl& preview_control);
		double terrainCost(const ReducedBodyTrajectory& phase_trans,
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

		/** @brief Actual reduced-body state */
		ReducedBodyState actual_state_;

		/** @brief Bounds of the preview optimization */
		PreviewBounds bounds_;

		/** @brief Reduced-body trajectory and transitions */
		ReducedBodyTrajectory reduced_trajectory_;
		ReducedBodyTrajectory reduced_sequence_;
		ReducedBodyTrajectory phase_transitions_;

		/** @brief Desired states */
		simulation::VelocityCommand actual_command_;
		double desired_yaw_B_;

		/** @brief Cost and constraint models */
		Eigen::Vector2d command_weight_;
		double cot_weight_;
		TerrainModel terrain_model_;

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

		/** @brief Whole-body solution */
		WholeBodyTrajectory motion_solution_;
};

} //@namespace ocp
} //@namespace dwl


#endif
