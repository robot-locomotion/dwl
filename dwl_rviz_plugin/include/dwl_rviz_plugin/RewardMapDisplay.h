#ifndef RVIZ_RewardMapDisplay_H
#define RVIZ_RewardMapDisplay_H

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <terrain_server/RewardMap.h>

#include <rviz/display.h>
#include "rviz/ogre_helpers/point_cloud.h"


namespace rviz
{

	class RosTopicProperty;
	class IntProperty;
	class EnumProperty;

} //@namespace rviz


namespace dwl_rviz_plugin
{

/**
 * @class RewardMapDisplay
 * @brief Rviz plugin for visualization of reward map of the environment
 */
class RewardMapDisplay : public rviz::Display
{
	Q_OBJECT
	public:
		/** @brief Constructor function */
		RewardMapDisplay();

		/** @brief Destructor function */
		virtual ~RewardMapDisplay();

		/**
		 * @brief Updates the informatio to display
		 * @param float wall_dt Wall delta time
		 * @param float ros_dt Ros delta time
		 */
		virtual void update(float wall_dt, float ros_dt);

		/** @brief Resets the information to display */
		virtual void reset();


	protected:
		/** @brief Method for initialization of the plugin */
		virtual void onInitialize();

		/** @brief Enables the display */
		virtual void onEnable();

		/** @brief Disable the display */
		virtual void onDisable();

		/** @brief Subscribes to the topic */
		void subscribe();

		/** @brief Unsubscribes to the topic */
		void unsubscribe();

		/** @brief Proccesing of the incoming message */
		void incomingMessageCallback(const terrain_server::RewardMapConstPtr& msg);

		/**
		 * @brief Sets the color of the reward cell
		 * @param double reward_value Reward value of the cell
		 * @param double min_reward Minimun reward value of the map
		 * @param double max_reward Maximun reward value of the map
		 * @param double color_factor Color factor
		 * @param rviz::PointCloud::Point& point Point with color information
		 */
		void setColor(double reward_value, double min_reward, double max_reward, double color_factor, rviz::PointCloud::Point& point);

		/** Clears the display data */
		void clear();

		/** @brief Vector of points */
		typedef std::vector<rviz::PointCloud::Point> VPoint;

		/** @brief Subscriber to the ObstacleMap messages */
		boost::shared_ptr<message_filters::Subscriber<terrain_server::RewardMap> > sub_;

		/** @brief Mutex of thread */
		boost::mutex mutex_;

		/** @brief Ogre-rviz point clouds */
		rviz::PointCloud* cloud_;

		/** @brief Plugin properties */
		rviz::IntProperty* queue_size_property_;

		/** @brief Obstacle map topic properties */
		rviz::RosTopicProperty* rewardmap_topic_property_;

		/** @brief Color property */
		rviz::EnumProperty* voxel_color_property_;

		/** @brief Max tree areas */
		int max_tree_areas_;

		/** @brief New points */
		VPoint new_points_;

		/** @brief Point buffer */
		VPoint point_buf_;

		/** @brief Indicates if the new points was received */
		bool new_points_received_;

		/** @brief Queue size */
		u_int32_t queue_size_;

		/** @brief Number of received messages */
		uint32_t messages_received_;

		/** @brief Color factor */
		double color_factor_;

		/** @brief Grid size */
		double grid_size_;

		/** @brief Height size */
		double height_size_;

		private Q_SLOTS:
			/** @brief Updates queue size */
			void updateQueueSize();

			/** @brief Updates the topic name */
			void updateTopic();

			/** @brief Color mode */
			void updateColorMode();
};

} //@namespace reward_map_rviz_plugin


#endif //RVIZ_RewardMapDisplay_H
