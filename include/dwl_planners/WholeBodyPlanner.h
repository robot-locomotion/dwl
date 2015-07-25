#ifndef DWL_Planners_WholeBodyPlanner_H
#define DWL_Planners_WholeBodyPlanner_H

#include <ros/ros.h>
#include <locomotion/PlanningOfMotionSequence.h>
#include <robot/Robot.h>
#include <solver/IpoptNLP.h>


namespace dwl_planners
{

class WholeBodyPlanner
{
	public:
		WholeBodyPlanner();
		~WholeBodyPlanner();

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
