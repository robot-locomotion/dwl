#include <QObject>

#include <dwl_rviz_plugin/ObstacleMapDisplay.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>

#include <environment/SpaceDiscretization.h>

#include <sstream>


using namespace rviz;

namespace dwl_rviz_plugin
{

ObstacleMapDisplay::ObstacleMapDisplay() : rviz::Display(), messages_received_(0), grid_size_(std::numeric_limits<double>::max())
{
	obstaclemap_topic_property_ = new RosTopicProperty( "Topic",
														 "",
														 QString::fromStdString(ros::message_traits::datatype<terrain_server::ObstacleMap>()),
														 "terrain_server::ObstacleMap topic to subscribe to reward map",
														 this, SLOT( updateTopic() ));

	queue_size_property_ = new IntProperty( "Queue Size",
	                                         queue_size_,
	                                         "Advanced: set the size of the incoming message queue. Increasing this "
	                                         "is useful if your incoming TF data is delayed significantly from your"
	                                         " image data, but it can greatly increase memory usage if the messages are big.",
	                                         this, SLOT( updateQueueSize() ));
	queue_size_property_->setMin(1);

	color_property_ = new ColorProperty( "Color", QColor( 0, 0, 0 ),
										  "The color of the obstacle map.",
										  this, SLOT( updateColor() ));
	alpha_property_ = new FloatProperty( "Alpha", 0.5f,
										  "The amount of transparency to apply to the obstacle map.",
										  this, SLOT( updateColor() ));
	alpha_property_->setMin( 0.0f );
	alpha_property_->setMax( 1.0f );

}


ObstacleMapDisplay::~ObstacleMapDisplay()
{
	unsubscribe();

	delete cloud_;

	if (scene_node_)
		scene_node_->detachAllObjects();
}


void ObstacleMapDisplay::update(float wall_dt, float ros_dt)
{
	if (new_points_received_) {
		boost::mutex::scoped_lock lock(mutex_);
		cloud_->clear();
		cloud_->setDimensions(grid_size_, grid_size_, height_size_);
		cloud_->addPoints(&new_points_.front(), new_points_.size());

		new_points_.clear();

		new_points_received_ = false;
	}
}


void ObstacleMapDisplay::reset()
{
	clear();
	messages_received_ = 0;
	setStatus(StatusProperty::Ok, "Messages", QString("0 reward map messages received"));
}


void ObstacleMapDisplay::onInitialize()
{
	boost::mutex::scoped_lock lock(mutex_);

	std::stringstream sname;
	sname << "PointCloud Nr.";// << i;
	cloud_ = new rviz::PointCloud();
	cloud_->setName(sname.str());
	cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
	scene_node_->attachObject((Ogre::MovableObject*) cloud_);

	alpha_ = alpha_property_->getFloat();
	color_ = color_property_->getOgreColor();
}


void ObstacleMapDisplay::onEnable()
{
	scene_node_->setVisible(true);
	subscribe();
}


void ObstacleMapDisplay::onDisable()
{
	scene_node_->setVisible(false);
	unsubscribe();

	clear();
}


void ObstacleMapDisplay::subscribe()
{
	if (!isEnabled())
		return;

	try {
		unsubscribe();

		const std::string& topicStr = obstaclemap_topic_property_->getStdString();

		if (!topicStr.empty()) {
			sub_.reset(new message_filters::Subscriber<terrain_server::ObstacleMap>());

			sub_->subscribe(threaded_nh_, topicStr, queue_size_);
			sub_->registerCallback(boost::bind(&ObstacleMapDisplay::incomingMessageCallback, this, _1));
		}
	}
	catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
	}
}


void ObstacleMapDisplay::unsubscribe()
{
	clear();

	try {
		// reset filters
		sub_.reset();
	} catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
	}
}


void ObstacleMapDisplay::incomingMessageCallback(const terrain_server::ObstacleMapConstPtr& msg)
{
	++messages_received_;
	setStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " reward map messages received");

	// Getting tf transform
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
		std::stringstream ss;
		ss << "Failed to transform from frame [" << msg->header.frame_id << "] to frame ["
		   << context_->getFrameManager()->getFixedFrame() << "]";
		this->setStatusStd(StatusProperty::Error, "Message", ss.str());

		return;
	}
	scene_node_->setOrientation(orientation);
	scene_node_->setPosition(position);

	// Clearing the old data of the buffers
	point_buf_.clear();

	// Computing the minimun key of the height
	double min_reward, max_reward = 0;
	unsigned int min_key_z = std::numeric_limits<unsigned int>::max();
	for (int i = 0; i < msg->cell.size(); i++) {
		if (min_key_z > msg->cell[i].key_z)
			min_key_z = msg->cell[i].key_z;
	}
	grid_size_ = msg->plane_size;
	height_size_ = msg->height_size;


	// Getting reward values and size of the pixel
	double cell_size = 0;
	dwl::environment::SpaceDiscretization space_discretization(grid_size_);
	space_discretization.setEnvironmentResolution(height_size_, false);
	for (int i = 0; i < msg->cell.size(); i++) {
		// Getting cartesian information of the reward map
		PointCloud::Point new_point;
		double x, y, z;
		space_discretization.keyToCoord(x, msg->cell[i].key_x, true);
		space_discretization.keyToCoord(y, msg->cell[i].key_y, true);

		unsigned int key_z = msg->cell[i].key_z;
		while (key_z >= min_key_z) {
			space_discretization.keyToCoord(z, key_z, false);
			Ogre::Vector3 position(x, y, z);
			new_point.position = position;
			new_point.setColor(color_.r, color_.g, color_.b, 0.5);
			key_z -= 1;

			point_buf_.push_back(new_point);
		}
	}

	// Recording the data from the buffers
	boost::mutex::scoped_lock lock(mutex_);

	new_points_.swap(point_buf_);

	new_points_received_ = true;
}


void ObstacleMapDisplay::clear()
{
	boost::mutex::scoped_lock lock(mutex_);

	cloud_->clear();
}


void ObstacleMapDisplay::updateQueueSize()
{
	queue_size_ = queue_size_property_->getInt();

	subscribe();
}


void ObstacleMapDisplay::updateTopic()
{
	unsubscribe();
	reset();
	subscribe();
	context_->queueRender();
}


void ObstacleMapDisplay::updateColor()
{
//	QColor color = color_property_->getColor();
	//color.setAlphaF( alpha_property_->getFloat() );
	alpha_ = alpha_property_->getFloat();
	color_ = color_property_->getOgreColor();
	//context_->queueRender();
}

} //@namespace dwl_rviz_plugin



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::ObstacleMapDisplay, rviz::Display)


