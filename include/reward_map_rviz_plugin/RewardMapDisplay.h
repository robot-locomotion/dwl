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


namespace reward_map_rviz_plugin
{

class RewardMapDisplay : public rviz::Display
{
	Q_OBJECT
	public:
		RewardMapDisplay();
		virtual ~RewardMapDisplay();

		// Overrides from Display
		virtual void update(float wall_dt, float ros_dt);
		virtual void reset();


	protected:
		// overrides from Display
		virtual void onInitialize();
		virtual void onEnable();
		virtual void onDisable();

		void subscribe();
		void unsubscribe();

		void incomingMessageCallback(const terrain_server::RewardMapConstPtr& msg);

		void setColor(double reward_value, double min_reward, double max_reward, double color_factor, rviz::PointCloud::Point& point);

		void clear();

		typedef std::vector<rviz::PointCloud::Point> VPoint;

		boost::shared_ptr<message_filters::Subscriber<terrain_server::RewardMap> > sub_;
		boost::mutex mutex_;

		int max_tree_areas_;

		// point buffer
		VPoint new_points_;
		VPoint point_buf_;
		bool new_points_received_;

		// Ogre-rviz point clouds
		rviz::PointCloud* cloud_;
		//double box_size_;
		//double box_size_buf_;

		// Plugin properties
		rviz::IntProperty* queue_size_property_;
		rviz::RosTopicProperty* rewardmap_topic_property_;


		u_int32_t queue_size_;
		uint32_t messages_received_;
		double color_factor_;
		double grid_size_;
		double height_size_;

		private Q_SLOTS:
			void updateQueueSize();
			void updateTopic();
};

} //@namespace reward_map_rviz_plugin


#endif //RVIZ_RewardMapDisplay_H
