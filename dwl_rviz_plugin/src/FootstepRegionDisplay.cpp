#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"
#include <dwl_rviz_plugin/FootstepRegionDisplay.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

FootstepRegionDisplay::FootstepRegionDisplay() : Display(), messages_received_(0),
		last_frame_count_(uint64_t(-1))
{
	alpha_property_ = new FloatProperty( "Alpha", 1.0,
			"Amount of transparency to apply to the cells.", this, SLOT( updateAlpha() ));

	alpha_property_->setMin( 0 );
	alpha_property_->setMax( 1 );

	topic_property_ = new RosTopicProperty( "Topic", "",
			QString::fromStdString( ros::message_traits::datatype<dwl_planners::ContactRegion>() ),
			"dwl_planners::ContactRegion topic to subscribe to.",
			this, SLOT( updateTopic() ));
}


void FootstepRegionDisplay::onInitialize()
{
	tf_filter_ = new tf::MessageFilter<dwl_planners::ContactRegion>(*context_->getTFClient(),
			fixed_frame_.toStdString(),	10, update_nh_ );

	static int count = 0;
	std::stringstream ss;
	ss << "PolyLine" << count++;
	cloud_ = new PointCloud();
	cloud_->setRenderMode(PointCloud::RM_TILES);
	cloud_->setCommonDirection(Ogre::Vector3::UNIT_Z);
	cloud_->setCommonUpVector(Ogre::Vector3::UNIT_Y);

	scene_node_->attachObject(cloud_);

	updateAlpha();

	tf_filter_->connectInput(sub_);
	tf_filter_->registerCallback(boost::bind(&FootstepRegionDisplay::incomingMessage, this, _1));

	context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}


FootstepRegionDisplay::~FootstepRegionDisplay()
{
	if ( initialized() ) {
		unsubscribe();
		clear();
		scene_node_->detachObject(cloud_);
		delete cloud_;
		delete tf_filter_;
	}
}


void FootstepRegionDisplay::clear()
{
	cloud_->clear();
	messages_received_ = 0;
	setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}


void FootstepRegionDisplay::updateTopic()
{
	unsubscribe();
	subscribe();
	context_->queueRender();
}


void FootstepRegionDisplay::updateAlpha()
{
	cloud_->setAlpha( alpha_property_->getFloat() );
	context_->queueRender();
}


void FootstepRegionDisplay::subscribe()
{
	if (!isEnabled())
		return;

	try {
		sub_.subscribe(update_nh_, topic_property_->getTopicStd(), 10);
		setStatus(StatusProperty::Ok, "Topic", "OK");
	} catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
	}
}


void FootstepRegionDisplay::unsubscribe()
{
	sub_.unsubscribe();
}


void FootstepRegionDisplay::onEnable()
{
	subscribe();
}


void FootstepRegionDisplay::onDisable()
{
	unsubscribe();
	clear();
}


void FootstepRegionDisplay::fixedFrameChanged()
{
	clear();
	tf_filter_->setTargetFrame(fixed_frame_.toStdString());
}


bool validateFloats(const dwl_planners::ContactRegion& msg)
{
	bool valid = true;
//	valid = valid && rviz::validateFloats(msg.regionscell_width);
//	valid = valid && rviz::validateFloats(msg.cell_height);
//	valid = valid && rviz::validateFloats(msg.regions);
	return valid;
}


void FootstepRegionDisplay::incomingMessage(const dwl_planners::ContactRegion::ConstPtr& msg)
{
	if (!msg)
		return;

	++messages_received_;
	if (context_->getFrameCount() == last_frame_count_)
		return;

	last_frame_count_ = context_->getFrameCount();
	if (!validateFloats(*msg)) {
		setStatus(StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
		return;
	}
	setStatus(StatusProperty::Ok, "Topic", QString::number(messages_received_) + " messages received");
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
	}

	scene_node_->setPosition(position);
	scene_node_->setOrientation(orientation);

	cloud_->clear();

	if (msg->regions[0].size.y == 0)
		setStatus(StatusProperty::Error, "Topic", "Cell width is zero, cells will be invisible.");
	else if (msg->regions[0].size.x == 0)
		setStatus(StatusProperty::Error, "Topic", "Cell height is zero, cells will be invisible.");

	cloud_->setDimensions(msg->regions[0].size.x, msg->regions[0].size.y, msg->regions[0].size.z);

	// Computing the yaw
//	double roll, pitch, yaw;
//	double q_x = msg->regions[0].pose.orientation.x;
//	double q_y = msg->regions[0].pose.orientation.y;
//	double q_z = msg->regions[0].pose.orientation.z;
//	double q_w = msg->regions[0].pose.orientation.w;
//	dwl::Orientation converter(q_x, q_y, q_z, q_w);
//	converter.getRPY(roll, pitch, yaw);
//	Ogre::Vector3 dir;
//	yaw = (90 + 25) * 3.1416 / 180;
//	dir.x = cos(yaw);
//	dir.y = sin(yaw);
//	dir.z = 0;
//	cloud_->setCommonUpVector(dir);

	uint32_t num_points = msg->regions.size();
	typedef std::vector< PointCloud::Point > V_Point;
	V_Point points;
	points.resize( num_points );
	for (uint32_t i = 0; i < num_points; i++) {
		PointCloud::Point& current_point = points[i];
		current_point.position.x = msg->regions[i].pose.position.x;
		current_point.position.y = msg->regions[i].pose.position.y;
		current_point.position.z = msg->regions[i].pose.position.z;

		Ogre::ColourValue color_int(msg->regions[i].color.r, msg->regions[i].color.g, msg->regions[i].color.b, 0.5*msg->regions[i].color.a); //= qtToOgre(color_property_->getColor());
		current_point.color = color_int;
	}

	if (!points.empty()) {
		cloud_->addPoints(&points.front(), points.size());
	}
}


void FootstepRegionDisplay::reset()
{
	Display::reset();
	clear();
}

} // namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::FootstepRegionDisplay, rviz::Display)
