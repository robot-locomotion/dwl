#include <QObject>

#include <dwl_rviz_plugin/RewardMapDisplay.h>

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

enum VoxelColorMode{ FULL_COLOR, GREY };

RewardMapDisplay::RewardMapDisplay() : rviz::Display(), messages_received_(0), color_factor_(0.8),
		grid_size_(std::numeric_limits<double>::max())
{
	rewardmap_topic_property_ = new RosTopicProperty( "Topic",
	                                                  "",
	                                                  QString::fromStdString(ros::message_traits::datatype<terrain_server::RewardMap>()),
	                                                  "terrain_server::RewardMap topic to subscribe to reward map",
	                                                  this, SLOT( updateTopic() ));

	queue_size_property_ = new IntProperty( "Queue Size",
	                                         queue_size_,
	                                         "Advanced: set the size of the incoming message queue. Increasing this "
	                                         "is useful if your incoming TF data is delayed significantly from your"
	                                         " image data, but it can greatly increase memory usage if the messages are big.",
	                                         this, SLOT( updateQueueSize() ));

	queue_size_property_->setMin(1);

	voxel_color_property_ = new rviz::EnumProperty("Color",
													"Full Color",
													"Select voxel coloring.",
													this,
													SLOT( updateColorMode() ) );


	voxel_color_property_->addOption( "Full Color", FULL_COLOR );
	voxel_color_property_->addOption( "Grey", GREY );
}


RewardMapDisplay::~RewardMapDisplay()
{
	unsubscribe();

	delete cloud_;

	if (scene_node_)
		scene_node_->detachAllObjects();
}


void RewardMapDisplay::update(float wall_dt, float ros_dt)
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


void RewardMapDisplay::reset()
{
	clear();
	messages_received_ = 0;
	setStatus(StatusProperty::Ok, "Messages", QString("0 reward map messages received"));
}


void RewardMapDisplay::onInitialize()
{
	boost::mutex::scoped_lock lock(mutex_);

	std::stringstream sname;
	sname << "PointCloud Nr.";// << i;
	cloud_ = new rviz::PointCloud();
	cloud_->setName(sname.str());
	cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
	scene_node_->attachObject((Ogre::MovableObject*) cloud_);
}


void RewardMapDisplay::onEnable()
{
	scene_node_->setVisible(true);
	subscribe();
}


void RewardMapDisplay::onDisable()
{
	scene_node_->setVisible(false);
	unsubscribe();

	clear();
}


void RewardMapDisplay::subscribe()
{
	if (!isEnabled())
		return;

	try {
		unsubscribe();

		const std::string& topicStr = rewardmap_topic_property_->getStdString();

		if (!topicStr.empty()) {
			sub_.reset(new message_filters::Subscriber<terrain_server::RewardMap>());

			sub_->subscribe(threaded_nh_, topicStr, queue_size_);
			sub_->registerCallback(boost::bind(&RewardMapDisplay::incomingMessageCallback, this, _1));
		}
	}
	catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
	}
}


void RewardMapDisplay::unsubscribe()
{
	clear();

	try {
		// reset filters
		sub_.reset();
	}
	catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
	}
}


void RewardMapDisplay::incomingMessageCallback(const terrain_server::RewardMapConstPtr& msg)
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

	// Computing the maximum and minimum reward of the map, and minimum key of the height
	double min_reward = -1;
	double max_reward = 0;
	unsigned int min_key_z = std::numeric_limits<unsigned int>::max();
	for (int i = 0; i < msg->cell.size(); i++) {
		if (min_reward > msg->cell[i].reward)
			min_reward = msg->cell[i].reward;

		if (max_reward < msg->cell[i].reward)
			max_reward = msg->cell[i].reward;

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

			// Setting the color of the cell acording the reward value
			setColor(msg->cell[i].reward, min_reward, max_reward, color_factor_, new_point);
			key_z -= 1;

			point_buf_.push_back(new_point);
		}
	}

	// Recording the data from the buffers
	boost::mutex::scoped_lock lock(mutex_);

	new_points_.swap(point_buf_);
	//box_size_ = box_size_buf_;

	new_points_received_ = true;
}


void RewardMapDisplay::setColor(double reward_value, double min_reward, double max_reward, double color_factor, rviz::PointCloud::Point& point)
{
	VoxelColorMode color_mode = static_cast<VoxelColorMode>(voxel_color_property_->getOptionInt());

	switch (color_mode)
	{
		case FULL_COLOR:
		{
			int i;
			double m, n, f;

			double s = 1.0;
			double v = 1.0;

			double h = (1.0 - std::min(std::max((reward_value - min_reward) / (max_reward - min_reward), 0.0), 1.0)) * color_factor;

			h -= floor(h);
			h *= 4;
			i = floor(h);
			f = h - i;
			if (!(i & 1))
				f = 1 - f; // if i is even
			m = v * (1 - s);
			n = v * (1 - s * f);

			switch (i)
			{
				case 0:
					point.setColor(m, n, v);
					break;
				case 1:
					point.setColor(m, v, 1-n);
					break;
				case 2:
					point.setColor(n, v, m);
					break;
				case 3:
					point.setColor(v, 1-n, m);
					break;
				default:
					point.setColor(1, 0.5, 0.5);
					break;
			}
			break;
		} case GREY:
		{
			double v = (reward_value - min_reward) / (max_reward - min_reward);
			point.setColor(v, v, v);
			break;
		} default:
			break;
	}
}


void RewardMapDisplay::clear()
{
	boost::mutex::scoped_lock lock(mutex_);

	cloud_->clear();
}


void RewardMapDisplay::updateQueueSize()
{
	queue_size_ = queue_size_property_->getInt();

	subscribe();
}


void RewardMapDisplay::updateTopic()
{
	unsubscribe();
	reset();
	subscribe();
	context_->queueRender();
}

void RewardMapDisplay::updateColorMode()
{

}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::RewardMapDisplay, rviz::Display)


