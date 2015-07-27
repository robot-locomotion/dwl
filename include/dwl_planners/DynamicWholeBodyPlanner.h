#ifndef DWL_PLANNERS__DYNAMIC_WHOLE_BODY_PLANNER__H
#define DWL_PLANNERS__DYNAMIC_WHOLE_BODY_PLANNER__H

#include <ros/ros.h>
#include <locomotion/PlanningOfMotionSequence.h>
#include <robot/Robot.h>
#include <solver/IpoptNLP.h>


namespace dwl_planners
{

class DynamicWholeBodyPlanner
{
	public:
		DynamicWholeBodyPlanner();
		~DynamicWholeBodyPlanner();

		void init();
		bool compute();

	private:
		ros::NodeHandle node_;

		/** @brief Planning of motion sequences pointer */
		dwl::locomotion::PlanningOfMotionSequence* planning_ptr_;
		dwl::robot::Robot robot_;
		dwl::solver::Solver* solver_;
		dwl::environment::EnvironmentInformation* environment_;
};

} //@namespace dwl_planners

#endif
