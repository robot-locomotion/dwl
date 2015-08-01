#ifndef DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H
#define DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H

#include <ros/ros.h>

// Locomotion headers
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
		ConstrainedWholeBodyPlanner();
		~ConstrainedWholeBodyPlanner();

		void init();
		bool compute();

		void publishWholeBodyTrajectory();


	private:
		ros::NodeHandle node_;
		ros::Publisher motion_plan_pub_;

		dwl::solver::OptimizationSolver* solver_;
		dwl::solver::IpoptNLP* ipopt_solver_;
		dwl::model::ConstrainedDynamicalSystem* constrained_system_;
		dwl::rbd::FloatingBaseSystem floating_base_system_;

		dwl::LocomotionState current_state_;
		dwl::LocomotionState lower_bound_;
		dwl::LocomotionState upper_bound_;


		dwl_planners::WholeBodyTrajectory robot_trajectory_msg_;
};

} //@namespace dwl_planners

#endif
