#include <QObject>

#include <reward_map_rviz_plugin/RewardMapDisplay.h>

//#include <boost/bind.hpp>
//#include <boost/shared_ptr.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>

#include <environment/PlaneGrid.h>

#include <sstream>


using namespace rviz;

namespace reward_map_rviz_plugin
{


RewardMapDisplay::RewardMapDisplay() : rviz::Display(), messages_received_(0), max_tree_areas_(10), areas_count_(1), color_factor_(0.8)
{
	rewardmap_topic_property_ = new RosTopicProperty( "Topic",
	                                                  "",
	                                                  QString::fromStdString(ros::message_traits::datatype<reward_map_server::RewardMap>()),
	                                                  "reward_map_server::RewardMap topic to subscribe to reward map",
	                                                  this, SLOT( updateTopic() ));

	queue_size_property_ = new IntProperty( "Queue Size",
	                                         queue_size_,
	                                         "Advanced: set the size of the incoming message queue. Increasing this "
	                                         "is useful if your incoming TF data is delayed significantly from your"
	                                         " image data, but it can greatly increase memory usage if the messages are big.",
	                                         this, SLOT( updateQueueSize() ));

	queue_size_property_->setMin(1);

}


RewardMapDisplay::~RewardMapDisplay()
{
	unsubscribe();

	for (std::vector<rviz::PointCloud*>::iterator it = cloud_.begin(); it != cloud_.end(); ++it)
		delete *(it);

	if (scene_node_)
		scene_node_->detachAllObjects();
}


void RewardMapDisplay::update(float wall_dt, float ros_dt)
{
	if (new_points_received_) {
		boost::mutex::scoped_lock lock(mutex_);

		for (size_t i = 0; i < max_tree_areas_; ++i) {
			double size = box_size_[i];

			cloud_[i]->clear();
			cloud_[i]->setDimensions(size, size, 0.04);

			cloud_[i]->addPoints(&new_points_[i].front(), new_points_[i].size());
			new_points_[i].clear();
		}
		box_size_.clear();

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

	box_size_.resize(max_tree_areas_);
	box_size_buf_.resize(max_tree_areas_);
	cloud_.resize(max_tree_areas_);
	point_buf_.resize(max_tree_areas_);
	new_points_.resize(max_tree_areas_);

	for (std::size_t i = 0; i < max_tree_areas_; ++i) {
		std::stringstream sname;
		sname << "PointCloud Nr." << i;
		cloud_[i] = new rviz::PointCloud();
		cloud_[i]->setName(sname.str());
		cloud_[i]->setRenderMode(rviz::PointCloud::RM_BOXES);
		scene_node_->attachObject(cloud_[i]);
	}
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
			sub_.reset(new message_filters::Subscriber<reward_map_server::RewardMap>());

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


void RewardMapDisplay::incomingMessageCallback(const reward_map_server::RewardMapConstPtr& msg)
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
	for (std::size_t i = 0; i < max_tree_areas_; ++i) {
		point_buf_[i].clear();
	}
	box_size_buf_.clear();

	// Computing the maximum and minumum reward of the map
	double min_reward, max_reward = 0;
	for (int i = 0; i < msg->cell.size(); i++) {
		if (min_reward > msg->cell[i].reward)
			min_reward = msg->cell[i].reward;

		if (max_reward < msg->cell[i].reward)
			max_reward = msg->cell[i].reward;
	}


	// Getting reward values and size of the pixel
	double cell_size = 0;
	dwl::environment::PlaneGrid gridmap(0.04); //TODO
	areas_count_ = 1;
	for (int i = 0; i < msg->cell.size(); i++) {
		if (i == 0) {
			cell_size = msg->cell[0].cell_size;
			box_size_buf_.push_back(cell_size);
		} else if (cell_size != msg->cell[i].cell_size) {
			cell_size = msg->cell[i].cell_size;
			box_size_buf_.push_back(cell_size);

			++areas_count_;
		}

		// Getting cartesian information of the reward map
		PointCloud::Point new_point;
		new_point.position.x = gridmap.keyToCoord(msg->cell[i].key_x);
		new_point.position.y = gridmap.keyToCoord(msg->cell[i].key_y);
		new_point.position.z = gridmap.keyToCoord(msg->cell[i].key_z);

		// Setting the color of the cell acording the reward value
		setColor(msg->cell[i].reward, min_reward, max_reward, color_factor_, new_point);

		point_buf_[areas_count_ - 1].push_back(new_point);
	}

	// Recording the data from the buffers
	boost::mutex::scoped_lock lock(mutex_);
	for (size_t i = 0; i < areas_count_; ++i)
		new_points_[i].swap(point_buf_[i]);

	box_size_.swap(box_size_buf_);

	new_points_received_ = true;
}


void RewardMapDisplay::setColor(double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point& point)
{
	int i;
	double m, n, f;

	double s = 1.0;
	double v = 1.0;

	double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

	h -= floor(h);
	h *= 6;
	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i) {
    	case 6:
    	case 0:
    		point.setColor(v, n, m);
    		break;
    	case 1:
    		point.setColor(n, v, m);
    		break;
    	case 2:
    		point.setColor(m, v, n);
    		break;
    	case 3:
    		point.setColor(m, n, v);
    		break;
    	case 4:
    		point.setColor(n, m, v);
    		break;
    	case 5:
    		point.setColor(v, m, n);
    		break;
    	default:
    		point.setColor(1, 0.5, 0.5);
    		break;
	}
}


void RewardMapDisplay::clear()
{
	boost::mutex::scoped_lock lock(mutex_);

	// reset rviz pointcloud boxes
	for (size_t i = 0; i < cloud_.size(); ++i) {
		cloud_[i]->clear();
	}
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


} //@namespace rewardmap_rviz_plugin



#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(reward_map_rviz_plugin::RewardMapDisplay, rviz::Display)




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // reset rviz pointcloud classes
/*  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(tree_depth_property_->getInt(), octomap->getTreeDepth());
    for (octomap::OcTree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {

      if (octomap->isNodeOccupied(*it))
      {

        int render_mode_mask = octree_render_property_->getOptionInt();

        bool display_voxel = false;

        // the left part evaluates to 1 for free voxels and 2 for occupied voxels
        if (((int)octomap->isNodeOccupied(*it) + 1) & render_mode_mask)
        {
          // check if current voxel has neighbors on all sides -> no need to be displayed
          bool allNeighborsFound = true;

          octomap::OcTreeKey key;
          octomap::OcTreeKey nKey = it.getKey();

          for (key[2] = nKey[2] - 1; allNeighborsFound && key[2] <= nKey[2] + 1; ++key[2])
          {
            for (key[1] = nKey[1] - 1; allNeighborsFound && key[1] <= nKey[1] + 1; ++key[1])
            {
              for (key[0] = nKey[0] - 1; allNeighborsFound && key[0] <= nKey[0] + 1; ++key[0])
              {
                if (key != nKey)
                {
                  octomap::OcTreeNode* node = octomap->search(key);

                  // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                  if (!(node && (((int)octomap->isNodeOccupied(node)) + 1) & render_mode_mask))
                  {
                    // we do not have a neighbor => break!
                    allNeighborsFound = false;
                  }
                }
              }
            }
          }

          display_voxel |= !allNeighborsFound;
        }


        if (display_voxel)
        {
          PointCloud::Point newPoint;

          newPoint.position.x = it.getX();
          newPoint.position.y = it.getY();
          newPoint.position.z = it.getZ();

          float cell_probability;

          OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());

          switch (octree_color_mode)
          {
            case OCTOMAP_Z_AXIS_COLOR:
              setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
              break;
            case OCTOMAP_PROBABLILTY_COLOR:
              cell_probability = it->getOccupancy();
              newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
              break;
            default:
              break;
          }

          // push to point vectors
          unsigned int depth = it.getDepth();
          point_buf_[depth - 1].push_back(newPoint);

          ++pointCount;
        }
      }
    }
  }

  if (pointCount)
  {
    boost::mutex::scoped_lock lock(mutex_);

    new_points_received_ = true;

    for (size_t i = 0; i < max_octree_depth_; ++i)
      new_points_[i].swap(point_buf_[i]);

  }
  delete octomap;*/
