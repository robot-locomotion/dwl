#ifndef DWL_Planners_HierarchicalPlanner_H
#define DWL_Planners_HierarchicalPlanner_H

#include <ros/ros.h>
#include <planning/WholeBodyLocomotion.h>
#include <planning/HierarchicalPlanning.h>
#include <planning/DijkstrapAlgorithm.h>
#include <planning/CostMap.h>
#include <reward_map_server/RewardMap.h>


namespace dwl_planners
{

class HierarchicalPlanners
{
	public:
		/**
		 * @brief Constructor function
		 * @param ros::NodeHandle node ROS node handle
		 */
		HierarchicalPlanners(ros::NodeHandle node);

		/** @brief Destructor function */
		~HierarchicalPlanners();

		/**
		 * @brief Initializes the hierarchical planner
		 */
		void init();

		/**
		 * @brief Callbacks method when the reward map message arrives
		 * @param const reward_map_server::RewardMapConstPtr& msg Reward map message
		 */
		void rewardMapCallback(const reward_map_server::RewardMapConstPtr& msg);


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Reward map subscriber */
		ros::Subscriber reward_sub_;

		/** @brief Locomotion algorithm */
		dwl::WholeBodyLocomotion locomotor_;

		/** @brief Planning of motion sequences pointer */
		dwl::planning::PlanningOfMotionSequences* planning_ptr_;

		/** @brief Solver pointer */
		dwl::planning::Solver* solver_ptr_;

		/** @brief Cost-map pointer */
		dwl::planning::Cost* cost_map_ptr_;

};


} //@namespace dwl_planners


#endif
