#include <reward_map_server/RewardMapServer.h>


RewardMapServer::RewardMapServer()
{
	reward_map_ = new dwl::environment::RewardOctoMap();

	// Adding the search areas
	// High resolution
	reward_map_->addSearchArea(-0.5, 2.5, -0.85, 0.85, -0.8, -0.2, 0.04);
	// Low resolution
	reward_map_->addSearchArea(2.5, 3.0, -0.85, 0.85, -0.8, -0.2, 0.08);
	reward_map_->addSearchArea(-0.75, -0.5, -0.85, 0.85, -0.8, -0.2, 0.08);
	reward_map_->addSearchArea(-0.75, 3.0, -1.25, -0.85, -0.8, -0.2, 0.08);
	reward_map_->addSearchArea(-0.75, 3.0, 0.85, 1.25, -0.8, -0.2, 0.08);
	reward_map_->setInterestRegion(1.5, 5.5);

	// Adding the features
	dwl::environment::Feature* slope_ptr = new dwl::environment::SlopeFeature();
	dwl::environment::Feature* height_dev_ptr = new dwl::environment::HeightDeviationFeature();
	dwl::environment::Feature* curvature_ptr = new dwl::environment::CurvatureFeature();
	reward_map_->addFeature(slope_ptr);
	reward_map_->addFeature(height_dev_ptr);
	reward_map_->addFeature(curvature_ptr);

	// Declaring the subscriber to octomap and tf messages
	octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap> (node_, "octomap_binary", 5);
	tf_octomap_sub_ = new tf::MessageFilter<octomap_msgs::Octomap> (*octomap_sub_, tf_listener_, "world", 5);
	tf_octomap_sub_->registerCallback(boost::bind(&RewardMapServer::octomapCallback, this, _1));

	// Declaring the publisher of reward map
	reward_pub_ = node_.advertise<reward_map_server::RewardMap>("reward_map", 1);

	reward_map_msg_.header.frame_id = "world"; //"base_footprint";
}


RewardMapServer::~RewardMapServer()
{
	delete reward_map_;
	//octomap_sub_.shutdown();

	if (tf_octomap_sub_){
		delete tf_octomap_sub_;
		tf_octomap_sub_ = NULL;
	}

	if (octomap_sub_){
		delete octomap_sub_;
		octomap_sub_ = NULL;
	}
}


void RewardMapServer::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	// Creating a octree
	octomap::OcTree* octomap = NULL;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	if (tree) {
		octomap = dynamic_cast<octomap::OcTree*>(tree);
	}

	if (!octomap) {
		ROS_WARN("Failed to create octree structure");
		return;
	}

	dwl::TerrainModel model;
	model.octomap = octomap;

	// Setting the resolution of the gridmap
	reward_map_->setResolution(octomap->getResolution(), false);

	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform("world", "base_footprint", msg->header.stamp, tf_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	// Getting the robot state (3D position and yaw angle)
	Eigen::Vector4d robot_position = Eigen::Vector4d::Zero();
	robot_position(0) = tf_transform.getOrigin()[0];
	robot_position(1) = tf_transform.getOrigin()[1];
	robot_position(2) = tf_transform.getOrigin()[2];

	// Computing the yaw angle
	tf::Quaternion q = tf_transform.getRotation();
	dwl::Orientation orientation(q.getX(), q.getY(), q.getZ(), q.getW());
	double roll, pitch, yaw;
	orientation.getRPY(roll, pitch, yaw);
	robot_position(3) = yaw;

	// Computing the reward map
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	reward_map_->compute(model, robot_position);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
}


void RewardMapServer::publishRewardMap()
{
	reward_map_msg_.header.stamp = ros::Time::now();

	std::map<dwl::Vertex, dwl::Cell> reward_gridmap;
	reward_gridmap = reward_map_->getRewardMap();

	reward_map_server::RewardCell cell;
	reward_map_msg_.cell_size = reward_map_->getResolution(true);
	reward_map_msg_.modeler_size = reward_map_->getResolution(false);

	// Converting the vertexs into a cell message
	for (std::map<dwl::Vertex, dwl::Cell>::iterator vertex_iter = reward_gridmap.begin();
			vertex_iter != reward_gridmap.end();
			vertex_iter++)
	{
		dwl::Vertex v = vertex_iter->first;
		dwl::Cell reward_cell = vertex_iter->second;

		cell.key_x = reward_cell.key.x;
		cell.key_y = reward_cell.key.y;
		cell.key_z = reward_cell.key.z;
		cell.reward = reward_cell.reward;
		reward_map_msg_.cell.push_back(cell);
	}

	// Publishing the reward map if there is at least one subscriber
	if (reward_pub_.getNumSubscribers() > 0)
		reward_pub_.publish(reward_map_msg_);

	// Deleting old information
	reward_map_msg_.cell.clear();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "reward_map_node");

	RewardMapServer octomap_modeler;
	ros::spinOnce();

	try {
		ros::Rate loop_rate(30);
		while(ros::ok()) {
			octomap_modeler.publishRewardMap();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("octomap_server exception: %s", e.what());
		return -1;
	}

	return 0;
}


