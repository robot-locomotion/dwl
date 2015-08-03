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
		ConstrainedWholeBodyPlanner();

		/** @brief Destructor function */
		~ConstrainedWholeBodyPlanner();

		/** @brief Initializes the constrained whole-body planner */
		void init();

		/** @brief Computes the whole-body trajectory */
		bool compute();

		/** @brief Publishes the planned whole-body trajectory */
		void publishWholeBodyTrajectory();


	private:
		/** @brief Ros node handle */
		ros::NodeHandle node_;

		/** Motion plan publisher */
		ros::Publisher motion_plan_pub_;

		/** @brief Whole-body trajectory optimization */
		dwl::locomotion::WholeBodyTrajectoryOptimization planning_;

		/** @brief Floating-base system information */
		dwl::rbd::FloatingBaseSystem floating_base_system_;

		/** @brief Current whole-body state */
		dwl::LocomotionState current_state_;

		/** @brief Desired whole-body state */
		dwl::LocomotionState desired_state_;
		dwl::LocomotionState lower_bound_;
		dwl::LocomotionState upper_bound_;

		/** @brief Whole-body trajectory message */
		dwl_planners::WholeBodyTrajectory robot_trajectory_msg_;
};

} //@namespace dwl_planners

#endif
