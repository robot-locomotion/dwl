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
		/** @brief Constructor function */
		FootstepRegionDisplay();

		/** @brief Destructor function */
		virtual ~FootstepRegionDisplay();

		/** @brief Method for initialization of the plugin */
		virtual void onInitialize();

		/** @brief Sets target frame */
		virtual void fixedFrameChanged();

		/** @brief Resets display */
		virtual void reset();


	protected:
		/** @brief Enables the display */
		virtual void onEnable();

		/** @brief Disable the display */
		virtual void onDisable();


	private Q_SLOTS:
		/** @brief Updates the alpha parameter */
		void updateAlpha();

		/** @brief Updates the topic to subscribe */
		void updateTopic();


	private:
		/** @brief Subscribes to the topic */
		void subscribe();

		/** @brief Unsubscribe to the topic */
		void unsubscribe();

		/** Clears the display data */
		void clear();

		/** @brief Proccesing of the incoming message */
		void incomingMessage(const dwl_planners::ContactRegion::ConstPtr& msg);

		/** @brief Point cloud pointer */
		rviz::PointCloud* cloud_;

		/** @brief Subscriber to the ObstacleMap messages */
		message_filters::Subscriber<dwl_planners::ContactRegion> sub_;

		/** @brief Tf filter that synchronizes the messages */
		tf::MessageFilter<dwl_planners::ContactRegion>* tf_filter_;

		/** @brief Topic properties */
		rviz::RosTopicProperty* topic_property_;

		/** @brief Alpha value */
		rviz::FloatProperty* alpha_property_;

		/** @brief Number of messages received */
		uint32_t messages_received_;

		/** @brief Number of frame received */
		uint64_t last_frame_count_;
};

} // namespace dwl_rviz_plugin

#endif
