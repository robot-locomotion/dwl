#ifndef DWL_Planners_HierarchicalPlanner_H
#define DWL_Planners_HierarchicalPlanner_H

#include <ros/ros.h>

#include <planning/WholeBodyLocomotion.h>
#include <planning/HierarchicalPlanning.h>
#include <planning/Dijkstrap.h>
#include <planning/AStar.h>
#include <environment/EnvironmentInformation.h>
#include <environment/AdjacencyEnvironment.h>
#include <environment/GridBasedBodyAdjacency.h>
#include <environment/LatticeBasedBodyAdjacency.h>

#include <reward_map_server/RewardMap.h>
#include <nav_msgs/Path.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>



namespace dwl_planners
{

/**
 * @class HierarchicalPlanners
 * @brief Class for solving legged motion planning by approaching as a hierarchical planner
 */
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

		/**
		 * @brief Publishs the computed body path
		 */
		void publishBodyPath();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Reward map subscriber */
		message_filters::Subscriber<reward_map_server::RewardMap>* reward_sub_;

		/** @brief TF and reward map subscriber */
		tf::MessageFilter<reward_map_server::RewardMap>* tf_reward_sub_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

		/** @brief Approximated body path publisher */
		ros::Publisher body_path_pub_;

		/** @brief Locomotion algorithm */
		dwl::WholeBodyLocomotion locomotor_;

		/** @brief Planning of motion sequences pointer */
		dwl::planning::PlanningOfMotionSequences* planning_ptr_;

		dwl::environment::EnvironmentInformation environment_ptr_;

		//dwl::environment::EnvironmentInformation* environment_ptr_;

		/** @brief Solver pointer */
		dwl::planning::Solver* solver_ptr_;

		/** @brief Approximated body path message */
		nav_msgs::Path body_path_msg_;

		/** @brief Approximated body path */
		std::vector<dwl::Pose> body_path_;

		/** @brief Robot pose */
		dwl::Pose robot_pose_;

};


} //@namespace dwl_planners


#endif
