#ifndef ObstacleMapServer_H
#define ObstacleMapServer_H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Utils.h>

#include <environment/ObstacleMap.h>

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <reward_map_server/ObstacleMap.h>
//#include <reward_map_server/RewardCell.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

//#include <utils/utils.h>
#include <utils/Orientation.h>


class ObstacleMapServer
{
	public:
		/** @brief Constructor function */
		ObstacleMapServer();

		/** @brief Destructor function */
		~ObstacleMapServer();

		/**
		 *  @brief Callback function when it arrives a octomap message
		 *  @param const octomap_msgs::Octomap::ConstPtr& msg Octomap message
		 */
		void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

		/** @brief Publishes a reward map */
		void publishObstacleMap();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Pointer to the ObstacleMap class */
		dwl::environment::ObstacleMap obstacle_map_;

		/** @brief Obstacle map publisher */
		ros::Publisher obstacle_pub_;

		/** @brief Octomap subcriber */
		message_filters::Subscriber<octomap_msgs::Octomap>* octomap_sub_;

		/** @brief TF and octomap subscriber */
		tf::MessageFilter<octomap_msgs::Octomap>* tf_octomap_sub_;

		/** @brief Obstacle map message */
		reward_map_server::ObstacleMap obstacle_map_msg_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

};

#endif
