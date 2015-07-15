#include <terrain_server/RewardMapServer.h>


namespace terrain_server
{

RewardMapServer::RewardMapServer(ros::NodeHandle node) : private_node_(node), base_frame_("base_link"), world_frame_("odom") , new_information_(false)
{
	// Getting the base and world frame
	private_node_.param("base_frame", base_frame_, base_frame_);
	private_node_.param("world_frame", world_frame_, world_frame_);
	reward_map_msg_.header.frame_id = world_frame_;

	// Declaring the subscriber to octomap and tf messages
	octomap_sub_ = new message_filters::Subscriber<octomap_msgs::Octomap> (node_, "octomap_binary", 5);
	tf_octomap_sub_ = new tf::MessageFilter<octomap_msgs::Octomap> (*octomap_sub_, tf_listener_, world_frame_, 5);
	tf_octomap_sub_->registerCallback(boost::bind(&RewardMapServer::octomapCallback, this, _1));

	// Declaring the publisher of reward map
	reward_pub_ = node_.advertise<terrain_server::RewardMap>("reward_map", 1);

	reset_srv_ = node_.advertiseService("reset", &RewardMapServer::reset, this);
}


RewardMapServer::~RewardMapServer()
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


bool RewardMapServer::init()
{
	// Getting the names of search areas
	XmlRpc::XmlRpcValue area_names;
	if (!private_node_.getParam("search_areas", area_names)) {
		ROS_ERROR("No search areas given in the namespace: %s.", private_node_.getNamespace().c_str());
	} else {
		// Evaluating the fetching information of the search areas
		if (area_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
			ROS_ERROR("Malformed search area specification.");
			return false;
		}

		double min_x, max_x, min_y, max_y, min_z, max_z, resolution;
		for (int i = 0; i < area_names.size(); i++) {
			private_node_.getParam((std::string) area_names[i] + "/min_x", min_x);
			private_node_.getParam((std::string) area_names[i] + "/max_x", max_x);
			private_node_.getParam((std::string) area_names[i] + "/min_y", min_y);
			private_node_.getParam((std::string) area_names[i] + "/max_y", max_y);
			private_node_.getParam((std::string) area_names[i] + "/min_z", min_z);
			private_node_.getParam((std::string) area_names[i] + "/max_z", max_z);
			private_node_.getParam((std::string) area_names[i] + "/resolution", resolution);

			// Adding the search areas
			reward_map_.addSearchArea(min_x, max_x, min_y, max_y, min_z, max_z, resolution);
		}
	}

	// Getting the interest region, i.e. the information outside this region will be deleted
	double radius_x = 1, radius_y = 1;
	private_node_.getParam("interest_region/radius_x", radius_x);
	private_node_.getParam("interest_region/radius_y", radius_y);
	reward_map_.setInterestRegion(radius_x, radius_y);

	// Getting the feature information
	bool enable_slope, enable_height_dev, enable_curvature;
	double weight;
	double default_weight = 1;
	private_node_.getParam("features/slope/enable", enable_slope);
	private_node_.getParam("features/height_deviation/enable", enable_height_dev);
	private_node_.getParam("features/curvature/enable", enable_curvature);

	// Adding the slope feature if it's enable
	if (enable_slope) {
		// Setting the weight feature
		private_node_.param("features/slope/weight", weight, default_weight);
		dwl::environment::Feature* slope_ptr = new dwl::environment::SlopeFeature();
		slope_ptr->setWeight(weight);

		// Adding the feature
		reward_map_.addFeature(slope_ptr);
	}

	// Adding the height deviation feature if it's enable
	if (enable_height_dev) {
		// Setting the weight feature
		private_node_.param("features/height_deviation/weight", weight, default_weight);
		double flat_height_deviation, max_height_deviation, min_allowed_height;
		private_node_.param("features/height_deviation/flat_height_deviation", flat_height_deviation, 0.01);
		private_node_.param("features/height_deviation/max_height_deviation", max_height_deviation, 0.3);
		private_node_.param("features/height_deviation/min_allowed_height", min_allowed_height, -std::numeric_limits<double>::max());
		dwl::environment::Feature* height_dev_ptr = new dwl::environment::HeightDeviationFeature(flat_height_deviation,
				max_height_deviation, min_allowed_height);
		height_dev_ptr->setWeight(weight);

		// Setting the neighboring area
		double size, resolution;
		private_node_.param("features/height_deviation/neighboring_area/square_size", size, 0.1);
		private_node_.param("features/height_deviation/neighboring_area/resolution", resolution, 0.04);
		height_dev_ptr->setNeighboringArea(-size, size, -size, size, resolution);

		// Adding the feature
		reward_map_.addFeature(height_dev_ptr);
	}

	// Adding the curvature feature if it's enable
	if (enable_curvature) {
		// Setting the weight feature
		private_node_.param("features/curvature/weight", weight, default_weight);
		dwl::environment::Feature* curvature_ptr = new dwl::environment::CurvatureFeature();
		curvature_ptr->setWeight(weight);

		// Adding the feature
		reward_map_.addFeature(curvature_ptr);
	}

	return true;
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

	// Setting the resolution of the gridmap
	reward_map_.setResolution(octomap->getResolution(), false);

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
	reward_map_.compute(octomap, robot_position);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of reward map is %f seg.", duration);

	new_information_ = true;
}


bool RewardMapServer::reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
	reward_map_.reset();

	ROS_INFO("Reset reward map");

	return true;
}


void RewardMapServer::publishRewardMap()
{
	if (new_information_) {
		// Publishing the reward map if there is at least one subscriber
		if (reward_pub_.getNumSubscribers() > 0) {
			reward_map_msg_.header.stamp = ros::Time::now();

			std::map<dwl::Vertex, dwl::RewardCell> reward_gridmap;
			reward_gridmap = reward_map_.getRewardMap();

			terrain_server::RewardCell cell;
			reward_map_msg_.plane_size = reward_map_.getResolution(true);
			reward_map_msg_.height_size = reward_map_.getResolution(false);

			// Converting the vertexes into a cell message
			for (std::map<dwl::Vertex, dwl::RewardCell>::iterator vertex_iter = reward_gridmap.begin();
					vertex_iter != reward_gridmap.end();
					vertex_iter++)
			{
				dwl::Vertex v = vertex_iter->first;
				dwl::RewardCell reward_cell = vertex_iter->second;

				cell.key_x = reward_cell.key.x;
				cell.key_y = reward_cell.key.y;
				cell.key_z = reward_cell.key.z;
				cell.reward = reward_cell.reward;
				reward_map_msg_.cell.push_back(cell);
			}

			reward_pub_.publish(reward_map_msg_);

			// Deleting old information
			reward_map_msg_.cell.clear();
			new_information_ = false;
		}
	}
}

} //@namespace terrain_server



int main(int argc, char **argv)
{
	ros::init(argc, argv, "reward_map_server");

	terrain_server::RewardMapServer reward_server;
	if (!reward_server.init())
		return -1;

	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);
		while(ros::ok()) {
			reward_server.publishRewardMap();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("reward_map_server exception: %s", e.what());
		return -1;
	}

	return 0;
}
