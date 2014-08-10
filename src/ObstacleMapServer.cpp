#include <terrain_server/ObstacleMapServer.h>


namespace terrain_server
{

ObstacleMapServer::ObstacleMapServer() : base_frame_("base_link"), world_frame_("odom")
{
	// Getting the base and world frame
	node_.param("base_frame", base_frame_, base_frame_);
	node_.param("world_frame", world_frame_, world_frame_);

	// Adding the search areas
	// High resolution
	obstacle_map_.addSearchArea(-0.5, 2.5, -0.85, 0.85, -0.352, 0.2, 0.04);
	// Low resolution
	/*obstacle_map_->addSearchArea(2.5, 3.0, -0.85, 0.85, -0.8, -0.2, 0.08);
	obstacle_map_->addSearchArea(-0.75, -0.5, -0.85, 0.85, -0.8, -0.2, 0.08);
	obstacle_map_->addSearchArea(-0.75, 3.0, -1.25, -0.85, -0.8, -0.2, 0.08);
	obstacle_map_->addSearchArea(-0.75, 3.0, 0.85, 1.25, -0.8, -0.2, 0.08);
	obstacle_map_->setInterestRegion(1.5, 5.5);*/

	// Declaring the subscriber to octomap and tf messages
	octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap> (node_, "octomap_binary", 5);
	tf_octomap_sub_ = new tf::MessageFilter<octomap_msgs::Octomap> (*octomap_sub_, tf_listener_, "odom", 5);
	tf_octomap_sub_->registerCallback(boost::bind(&ObstacleMapServer::octomapCallback, this, _1));

	// Declaring the publisher of reward map
	obstacle_pub_ = node_.advertise<terrain_server::ObstacleMap>("obstacle_map", 1);

	obstacle_map_msg_.header.frame_id = "odom";
}


ObstacleMapServer::~ObstacleMapServer()
{
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


	// Setting the resolution of the gridmap
	obstacle_map_.setResolution(octomap->getResolution(), false);

	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform("odom", "base_link", msg->header.stamp, tf_transform);
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
	obstacle_map_.compute(octomap, robot_position);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);
}


void ObstacleMapServer::publishObstacleMap()
{
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

	// Publishing the reward map if there is at least one subscriber
	if (obstacle_pub_.getNumSubscribers() > 0)
		obstacle_pub_.publish(obstacle_map_msg_);

	// Deleting old information
	obstacle_map_msg_.cell.clear();
}

} //@namespace terrain_server


int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_map_node");

	terrain_server::ObstacleMapServer obstacle_server;
	ros::spinOnce();

	try {
		ros::Rate loop_rate(30);
		while(ros::ok()) {
			obstacle_server.publishObstacleMap();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("octomap_server exception: %s", e.what());
		return -1;
	}

	return 0;
}
