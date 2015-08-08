#ifndef RVIZ_ObstacleMapDisplay_H
#define RVIZ_ObstacleMapDisplay_H

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <terrain_server/ObstacleMap.h>

#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>


namespace rviz
{

	class RosTopicProperty;
	class IntProperty;
	class EnumProperty;

} //@namespace rviz


namespace dwl_rviz_plugin
{

/**
 * @class ObstacleMapDisplay
 * @brief Rviz plugin for visualization of obstacle map of the environment
 */
class ObstacleMapDisplay : public rviz::Display
{
	Q_OBJECT
	public:
		/** @brief Constructor function */
		ObstacleMapDisplay();

		/** @brief Destructor function */
		virtual ~ObstacleMapDisplay();

		/**
		 * @brief Updates the information to display
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
		void incomingMessageCallback(const terrain_server::ObstacleMapConstPtr& msg);

		/** Clears the display data */
		void clear();

		/** @brief Vector of points */
		typedef std::vector<rviz::PointCloud::Point> VPoint;

		/** @brief Subscriber to the ObstacleMap messages */
		boost::shared_ptr<message_filters::Subscriber<terrain_server::ObstacleMap> > sub_;

		/** @brief Mutex of thread */
		boost::mutex mutex_;

		/** @brief Ogre-rviz point clouds */
		rviz::PointCloud* cloud_;

		/** @brief Plugin properties */
		rviz::IntProperty* queue_size_property_;

		/** @brief Obstacle map topic properties */
		rviz::RosTopicProperty* obstaclemap_topic_property_;

		/** @brief Plugin color properties */
		rviz::ColorProperty* color_property_;

		/** @brief Plugin alpha property */
		rviz::FloatProperty* alpha_property_;

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

		/** @brief Grid size */
		double grid_size_;

		/** @brief Height size */
		double height_size_;

		/** @brief Color value */
		Ogre::ColourValue color_;

		/** @brief Alpha value */
		float alpha_;

		private Q_SLOTS:
			/** @brief Updates queue size */
			void updateQueueSize();

			/** @brief Updates the topic name */
			void updateTopic();

			/** @brief Updates the color */
			void updateColor();
};

} //@namespace reward_map_rviz_plugin


#endif
