#ifndef DWL_RewardMapServer_H
#define DWL_RewardMapServer_H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Utils.h>

#include <environment/RewardOctoMap.h>
#include <environment/SlopeFeature.h>
#include <environment/HeightDeviationFeature.h>
#include <environment/CurvatureFeature.h>

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <reward_map_server/RewardMap.h>
#include <reward_map_server/RewardCell.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <utils/utils.h>
//#include <pthread.h>



class RewardMapServer
{
	public:
		/** @brief Constructor function */
		RewardMapServer();

		/** @brief Destructor function */
		~RewardMapServer();

		void addFeacture() {}
		void compute() {}

		/**
		 * @brief Callback function when it arrives a octomap message
		 */
		void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

		/**
		 * @brief Publishs a reward map
		 */
		void publishRewardMap();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Pointer to the reward map class */
		dwl::environment::RewardMap* reward_map_;

		/** @brief Reward map publisher */
		ros::Publisher reward_pub_;

		/** @brief Octomap subcriber */
		message_filters::Subscriber<octomap_msgs::Octomap>* octomap_sub_;

		/** @brief TF and octomap subscriber */
		tf::MessageFilter<octomap_msgs::Octomap>* tf_octomap_sub_;

		/** @brief Reward map message */
		reward_map_server::RewardMap reward_map_msg_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

};

#endif
