#ifndef TERRAIN_SERVER__OBSTACLE_MAP_SERVER___H
#define TERRAIN_SERVER__OBSTACLE_MAP_SERVER___H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Utils.h>

#include <environment/ObstacleMap.h>
#include <utils/Orientation.h>

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <terrain_server/ObstacleMap.h>
#include <terrain_server/RewardCell.h>
#include <std_srvs/Empty.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


namespace terrain_server
{

/**
 * @class ObstacleMapServer
 * @brief Class for building obstacle map of the environment
 */
class ObstacleMapServer
{
	public:
		/** @brief Constructor function */
		ObstacleMapServer();

		/** @brief Destructor function */
		~ObstacleMapServer();

		/** @brief Initialization of the reward map server */
		bool init();

		/**
		 *  @brief Callback function when it arrives a octomap message
		 *  @param const octomap_msgs::Octomap::ConstPtr& msg Octomap message
		 */
		void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

		/** @brief Publishes a reward map */
		void publishObstacleMap();

		/** @brief Resets the obstacle map */
		bool reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);


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

		/** @brief Reset service */
		ros::ServiceServer reset_srv_;

		/** @brief Obstacle map message */
		terrain_server::ObstacleMap obstacle_map_msg_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

		/** @brief Base frame */
		std::string base_frame_;

		/** @brief World frame */
		std::string world_frame_;

		/** @brief Indicates if it was computed new information of the reward map */
		bool new_information_;
};

} //@namespace terrain_server

#endif
