#include <terrain_server/ObstacleMapServer.h>


namespace terrain_server
{

ObstacleMapServer::ObstacleMapServer() : base_frame_("base_link"), world_frame_("odom"), new_information_(false)
{
	// Declaring the subscriber to octomap and tf messages
	octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap> (node_, "octomap_binary", 5);
	tf_octomap_sub_ = new tf::MessageFilter<octomap_msgs::Octomap> (*octomap_sub_, tf_listener_, world_frame_, 5);
	tf_octomap_sub_->registerCallback(boost::bind(&ObstacleMapServer::octomapCallback, this, _1));

	// Declaring the publisher of reward map
	obstacle_pub_ = node_.advertise<terrain_server::ObstacleMap>("obstacle_map", 1);

	obstacle_map_msg_.header.frame_id = world_frame_;

	reset_srv_ = node_.advertiseService("obstacle_map/reset", &ObstacleMapServer::reset, this);
}


bool ObstacleMapServer::init()
{
	// Getting the base and world frame
	node_.param("base_frame", base_frame_, base_frame_);
	node_.param("world_frame", world_frame_, world_frame_);

	// Getting the names of search areas
	XmlRpc::XmlRpcValue area_names;
	if (!node_.getParam("obstacle_map/search_areas", area_names)) {
		ROS_ERROR("No search areas given in the namespace: %s.", node_.getNamespace().c_str());
	} else {
		// Evaluating the fetching information of the search areas
		if (area_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
			ROS_ERROR("Malformed search area specification.");
			return false;
		}

		double min_x, max_x, min_y, max_y, min_z, max_z, resolution;
		for (int i = 0; i < area_names.size(); i++) {
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/min_x", min_x);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/max_x", max_x);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/min_y", min_y);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/max_y", max_y);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/min_z", min_z);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/max_z", max_z);
			node_.getParam("obstacle_map/" + (std::string) area_names[i] + "/resolution", resolution);

			// Adding the search areas
			obstacle_map_.addSearchArea(min_x, max_x, min_y, max_y, min_z, max_z, resolution);
		}
	}

	// Getting the interest region, i.e. the information outside this region will be deleted
	double radius_x, radius_y;
	node_.getParam("reward_map/interest_region/radius_x", radius_x);
	node_.getParam("reward_map/interest_region/radius_y", radius_y);
	obstacle_map_.setInterestRegion(radius_x, radius_y);

	return true;
}


ObstacleMapServer::~ObstacleMapServer()
{
	if (tf_octomap_sub_){
		delete tf_octomap_sub_;
		tf_octomap_sub_ = NULL;
	}

	if (octomap_sub_){
		delete octomap_sub_;
		octomap_sub_ = NULL;
	}
}


void ObstacleMapServer::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
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

	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform(world_frame_, base_frame_, msg->header.stamp, tf_transform);
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
	double yaw = dwl::math::getYaw(dwl::math::getRPY(Eigen::Quaterniond(q.getW(), q.getX(), q.getY(), q.getZ())));
	robot_position(3) = yaw;

	// Computing the reward map
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	obstacle_map_.compute(octomap, robot_position);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);

	new_information_ = true;
}


bool ObstacleMapServer::reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	obstacle_map_.reset();

	ROS_INFO("Reset obstacle map");

	return true;
}


void ObstacleMapServer::publishObstacleMap()
{
	if (new_information_) {
		// Publishing the reward map if there is at least one subscriber
		if (obstacle_pub_.getNumSubscribers() > 0) {
			obstacle_map_msg_.header.stamp = ros::Time::now();

			std::map<dwl::Vertex, dwl::Cell> obstacle_gridmap;
			obstacle_gridmap = obstacle_map_.getObstacleMap();

			terrain_server::Cell cell;
			obstacle_map_msg_.plane_size = obstacle_map_.getResolution(true);
			obstacle_map_msg_.height_size = obstacle_map_.getResolution(false);

			// Converting the vertexs into a cell message
			for (std::map<dwl::Vertex, dwl::Cell>::iterator vertex_iter = obstacle_gridmap.begin();
					vertex_iter != obstacle_gridmap.end();
					vertex_iter++)
			{
				dwl::Vertex v = vertex_iter->first;
				dwl::Cell obstacle_cell = vertex_iter->second;

				cell.key_x = obstacle_cell.key.x;
				cell.key_y = obstacle_cell.key.y;
				cell.key_z = obstacle_cell.key.z;
				obstacle_map_msg_.cell.push_back(cell);
			}


			obstacle_pub_.publish(obstacle_map_msg_);

			// Deleting old information
			obstacle_map_msg_.cell.clear();
			new_information_ = false;
		}
	}
}

} //@namespace terrain_server


int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_map_server");

	terrain_server::ObstacleMapServer obstacle_server;
	if (!obstacle_server.init())
			return -1;

	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);
		while(ros::ok()) {
			obstacle_server.publishObstacleMap();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("obstacle_map_server exception: %s", e.what());
		return -1;
	}

	return 0;
}
