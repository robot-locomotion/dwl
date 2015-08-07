#ifndef DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H
#define DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H

#include <ros/ros.h>

// Locomotion headers
#include <locomotion/WholeBodyTrajectoryOptimization.h>
#include <model/ConstrainedDynamicalSystem.h>
#include <model/StateTrackingEnergyCost.h>
#include <model/ControlEnergyCost.h>
#include <solver/IpoptNLP.h>

// Messages headers
#include <dwl_planners/WholeBodyTrajectory.h>


namespace dwl_planners
{

class ConstrainedWholeBodyPlanner
{
	public:
		/** @brief Constructor function */
		ConstrainedWholeBodyPlanner(ros::NodeHandle node = ros::NodeHandle("~"));

		/** @brief Destructor function */
		~ConstrainedWholeBodyPlanner();

		/** @brief Initializes the constrained whole-body planner */
		void init();

		/** @brief Computes the whole-body trajectory */
		bool compute();

		/** @brief Publishes the planned whole-body trajectory */
		void publishWholeBodyTrajectory();


	private:
		/**
		 * @brief Writes the whole-body state message from a locomotion state
		 * @param dwl_planners::WholeBodyState& Whole-body state message
		 * @param const dwl::LocomotionState& Locomotion state
		 */
		void writeWholeBodyStateMessage(dwl_planners::WholeBodyState& msg,
										const dwl::LocomotionState& state);

		/** @brief Ros node handle */
		ros::NodeHandle node_;

		/** @brief Privated Ros node handle */
		ros::NodeHandle privated_node_;

		/** Motion plan publisher */
		ros::Publisher motion_plan_pub_;

		/** @brief Whole-body trajectory optimization */
		dwl::locomotion::WholeBodyTrajectoryOptimization planning_;

		/** @brief Current whole-body state */
		dwl::LocomotionState current_state_;

		/** @brief Desired whole-body state */
		dwl::LocomotionState desired_state_;

		/** @brief Interpolation time */
		double interpolation_time_;

		/** @brief Allowed computation time */
		double computation_time_;

		/** @brief Whole-body trajectory message */
		dwl_planners::WholeBodyTrajectory robot_trajectory_msg_;
};

} //@namespace dwl_planners

#endif
