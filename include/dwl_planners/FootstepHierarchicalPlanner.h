#ifndef DWL_PLANNERS__FOOTSTEP_HIERARCHICAL_PLANNER__H
#define DWL_PLANNERS__FOOTSTEP_HIERARCHICAL_PLANNER__H

#include <ros/ros.h>

// Locomotion headers
#include <locomotion/HierarchicalPlanning.h>
#include <locomotion/MotionPlanning.h>
#include <locomotion/SearchBasedBodyMotionPlanning.h>
#include <locomotion/ContactPlanning.h>
#include <locomotion/GreedyFootstepPlanning.h>
#include <solver/Dijkstrap.h>
#include <solver/AStar.h>
#include <solver/AnytimeRepairingAStar.h>

// Robot and Environment headers
#include <robot/Robot.h>
#include <environment/EnvironmentInformation.h>
#include <environment/AdjacencyEnvironment.h>
#include <environment/GridBasedBodyAdjacency.h>
#include <environment/LatticeBasedBodyAdjacency.h>
#include <environment/PotentialLegCollisionFeature.h>
#include <environment/PotentialBodyOrientationFeature.h>
#include <environment/SupportTriangleFeature.h>
#include <environment/LegCollisionFeature.h>
#include <environment/BodyOrientationFeature.h>
#include <environment/KinematicFeasibilityFeature.h>
#include <environment/StancePostureFeature.h>

// Messages headers
#include <dwl_planners/ContactSequence.h>
#include <dwl_planners/ContactRegion.h>
#include <terrain_server/RewardMap.h>
#include <terrain_server/ObstacleMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// TF headers
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


namespace dwl_planners
{

/**
 * @class FootstepHierarchicalPlanners
 * @brief This class implement a hierarchical approach for footstep planning over rough terrain
 */
class FootstepHierarchicalPlanners
{
	public:
		/**
		 * @brief Constructor function
		 * @param ros::NodeHandle node ROS node handle
		 */
		FootstepHierarchicalPlanners(ros::NodeHandle node = ros::NodeHandle("~"));

		/** @brief Destructor function */
		~FootstepHierarchicalPlanners();

		/** @brief Initializes the hierarchical planner */
		void init();

		/** Initializes the robot properties */
		void initRobot();

		/** Initializes the body planner */
		void initBodyPlanner();

		/** Initialized the contact planner */
		void initContactPlanner();

		/** @brief Computes the hierarchical plan */
		bool compute();

		/**
		 * @brief Callback method when the reward map message arrives
		 * @param const terrain_server::RewardMapConstPtr& msg Reward map message
		 */
		void rewardMapCallback(const terrain_server::RewardMapConstPtr& msg);

		/**
		 * @brief Callback method when the obstacle map message arrives
		 * @param const terrain_server::ObstacleMapConstPtr& msg Obstacle map message
		 */
		void obstacleMapCallback(const terrain_server::ObstacleMapConstPtr& msg);

		/**
		 * @brief Callback method for reseting the body goal state
		 * @param const geometry_msgs::PoseStampedConstPtr& msg Body goal message
		 */
		void resetGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

		/** @brief Publishes the computed body path */
		void publishBodyPath();

		/** @brief Publishes the contact sequence */
		void publishContactSequence();

		/** @brief Publishes the contact regions */
		void publishContactRegions();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

		/** @brief Reward map subscriber */
		ros::Subscriber reward_sub_;

		/** @brief Obstacle map subscriber */
		ros::Subscriber obstacle_sub_;

		/** @brief Body goal subscriber */
		ros::Subscriber body_goal_sub_;

		/** @brief Approximated body path publisher */
		ros::Publisher body_path_pub_;

		/** @brief Contact sequence publisher */
		ros::Publisher contact_sequence_pub_;

		/** @brief Contact sequence publisher for visualization */
		ros::Publisher contact_sequence_rviz_pub_;

		ros::Publisher contact_regions_pub_;

		/** @brief Thread mutex of the reward information */
		pthread_mutex_t reward_lock_;

		/** @brief Thread mutex of the obstacle information */
		pthread_mutex_t obstacle_lock_;

		/** @brief Thread mutex of the planner */
		pthread_mutex_t planner_lock_;

		/** @brief Planning of motion sequences pointer */
		dwl::locomotion::HierarchicalPlanning planning_;

		/** @brief Body planner */
		dwl::locomotion::MotionPlanning* body_planner_ptr_;

		/** @brief Contact planner */
		dwl::locomotion::ContactPlanning* footstep_planner_ptr_;

		/** @brief Solver pointer */
		dwl::solver::Solver* body_path_solver_ptr_;

		/** @brief Adjacency environment pointer */
		dwl::environment::AdjacencyEnvironment* adjacency_ptr_;

		/** @brief Environment information */
		dwl::environment::EnvironmentInformation environment_;

		/** @brief Robot properties */
		dwl::robot::Robot robot_;

		/** @brief Current robot pose */
		dwl::Pose current_pose_;

		/** @brief Approximated body path message */
		nav_msgs::Path body_path_msg_;

		/** @brief Contact sequence ROS message */
		dwl_planners::ContactSequence contact_sequence_msg_;

		/** @brief Contact sequence for visualization in RVIZ */
		visualization_msgs::Marker contact_sequence_rviz_msg_;

		dwl_planners::ContactRegion contact_regions_msg_;

		/** @brief Approximated body path */
		std::vector<dwl::Pose> body_path_;

		/** @brief Contact search regions */
		std::vector<dwl::ContactSearchRegion> contact_search_region_;

		/** Contact sequence */
		std::vector<dwl::Contact> contact_sequence_;

		/** @brief Base frame */
		std::string base_frame_;

		/** @brief World frame */
		std::string world_frame_;
};

} //@namespace dwl_planners

#endif
