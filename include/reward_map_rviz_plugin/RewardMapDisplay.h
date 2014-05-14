#ifndef RVIZ_RewardMapDisplay_H
#define RVIZ_RewardMapDisplay_H

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <reward_map_server/RewardMap.h>

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

		void incomingMessageCallback(const reward_map_server::RewardMapConstPtr& msg);

		void setColor(double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point& point);

		void clear();

		typedef std::vector<rviz::PointCloud::Point> VPoint;
		typedef std::vector<VPoint> VVPoint;

		boost::shared_ptr<message_filters::Subscriber<reward_map_server::RewardMap> > sub_;
		boost::mutex mutex_;

		int max_tree_areas_;

		// point buffer
		VVPoint new_points_;
		VVPoint point_buf_;
		//VPoint point_buf_;
		bool new_points_received_;

		// Ogre-rviz point clouds
		std::vector<rviz::PointCloud*> cloud_;
		std::vector<double> box_size_;
		std::vector<double> box_size_buf_;

		// Plugin properties
		rviz::IntProperty* queue_size_property_;
		rviz::RosTopicProperty* rewardmap_topic_property_;
		//rviz::EnumProperty* octree_render_property_;
		//rviz::EnumProperty* octree_coloring_property_;
		//rviz::IntProperty* tree_depth_property_;

		u_int32_t queue_size_;
		std::size_t octree_depth_;
		uint32_t messages_received_;
		double color_factor_;
		size_t areas_count_;

		private Q_SLOTS:
			void updateQueueSize();
			void updateTopic();
			//void updateTreeDepth();
			//void updateOctreeRenderMode();
			//void updateOctreeColorMode();
};

} //@namespace reward_map_rviz_plugin


#endif //RVIZ_RewardMapDisplay_H
