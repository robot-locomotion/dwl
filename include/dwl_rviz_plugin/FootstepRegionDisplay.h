#ifndef RVIZ_FootstepRegionDisplay_H
#define RVIZ_FootstepRegionDisplay_H

#include "rviz/display.h"

#include <utils/Orientation.h>
#include <dwl_planners/ContactRegion.h>
#include <nav_msgs/MapMetaData.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <boost/shared_ptr.hpp>


namespace Ogre
{
	class ManualObject;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class PointCloud;
class RosTopicProperty;
}

namespace dwl_rviz_plugin
{
/**
* \class FootstepRegionDisplay
* \brief Displays a dwl_planners::ContactRegion message
*/
class FootstepRegionDisplay : public rviz::Display
{
	Q_OBJECT
	public:
		FootstepRegionDisplay();
		virtual ~FootstepRegionDisplay();
		virtual void onInitialize();
		// Overrides from Display
		virtual void fixedFrameChanged();
		virtual void reset();
	protected:
		// overrides from Display
		virtual void onEnable();
		virtual void onDisable();

	private Q_SLOTS:
		void updateAlpha();
		void updateTopic();

	private:
		void subscribe();
		void unsubscribe();
		void clear();
		void incomingMessage(const dwl_planners::ContactRegion::ConstPtr& msg);
		rviz::PointCloud* cloud_;
		message_filters::Subscriber<dwl_planners::ContactRegion> sub_;
		tf::MessageFilter<dwl_planners::ContactRegion>* tf_filter_;
		rviz::RosTopicProperty* topic_property_;
		rviz::FloatProperty* alpha_property_;
		uint32_t messages_received_;
		uint64_t last_frame_count_;
};

} // namespace rviz

#endif
