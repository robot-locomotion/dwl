#include <dwl_planners/HierarchicalPlanner.h>


namespace dwl_planners
{

HierarchicalPlanners::HierarchicalPlanners(ros::NodeHandle node) : node_(node), planning_ptr_(NULL), solver_ptr_(NULL), cost_map_ptr_(NULL)
{
	reward_sub_ = new message_filters::Subscriber<reward_map_server::RewardMap> (node_, "reward_map", 5);
	tf_reward_sub_ = new tf::MessageFilter<reward_map_server::RewardMap> (*reward_sub_, tf_listener_, "world", 5);
	tf_reward_sub_->registerCallback(boost::bind(&HierarchicalPlanners::rewardMapCallback, this, _1));

	// Declaring the publisher of approximated body path
	body_path_pub_ = node_.advertise<nav_msgs::Path>("approximated_body_path", 1);

	body_path_msg_.header.frame_id = "world";
}


HierarchicalPlanners::~HierarchicalPlanners()
{

}


void HierarchicalPlanners::init()
{
	// Setting the subscription to the reward_map message
	//reward_sub_ = node_.subscribe("reward_map", 1, &HierarchicalPlanners::rewardMapCallback, this);

	//  Setup of the locomotion approach
	planning_ptr_ = new dwl::planning::HierarchicalPlanning();

	// Initialization of planning algorithm, which includes the initizalization and setup of solver algorithm
	solver_ptr_ = new dwl::planning::AStar();//new dwl::planning::Dijkstrap();
	planning_ptr_->reset(solver_ptr_);

	cost_map_ptr_ = new dwl::planning::CostMap();

	// Setting up the planner algorithm in the locomotion approach
	locomotor_.reset(planning_ptr_);
	locomotor_.addCost(cost_map_ptr_);

	locomotor_.init();
}


void HierarchicalPlanners::rewardMapCallback(const reward_map_server::RewardMapConstPtr& msg)
{
	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform("world", "base_footprint", msg->header.stamp, tf_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	// Getting the robot state (3D position and yaw angle)
	Eigen::Vector3d robot_position;
	robot_position(0) = tf_transform.getOrigin()[0];
	robot_position(1) = tf_transform.getOrigin()[1];
	robot_position(2) = tf_transform.getOrigin()[2];
	Eigen::Vector4d robot_orientation;
	robot_orientation(0) = tf_transform.getRotation().getX();
	robot_orientation(1) = tf_transform.getRotation().getY();
	robot_orientation(2) = tf_transform.getRotation().getZ();
	robot_orientation(3) = tf_transform.getRotation().getW();

	robot_pose_.position = robot_position;
	robot_pose_.orientation = robot_orientation;

	dwl::Pose start_pose, goal_pose;
	start_pose.position[0] = robot_position(0);
	start_pose.position[1] = robot_position(1);
	goal_pose.position[0] = 5.0;
	goal_pose.position[1] = 0.0;
	locomotor_.update(start_pose, goal_pose);



	std::vector<dwl::Cell> reward_map;
	locomotor_.setGridMapResolution(msg->cell_size);

	// Converting the messages to reward_map format
	dwl::Cell reward_cell;
	for (int i = 0; i < msg->cell.size(); i++) {
		// Filling the reward per every cell
		reward_cell.cell_key.grid_id.key[0] = msg->cell[i].key_x;
		reward_cell.cell_key.grid_id.key[1] = msg->cell[i].key_y;
		reward_cell.cell_key.height_id = msg->cell[i].key_z;
		reward_cell.reward = msg->cell[i].reward;

		// Adding the reward cell to the queue
		reward_map.push_back(reward_cell);
	}

	// Adding the cost map
	cost_map_ptr_->setCostMap(reward_map);

	// Computing the locomotion plan
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	locomotor_.compute(robot_pose_);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);


	locomotor_.getBodyPath().swap(body_path_);

/*	for (int i = 0; i < body_path_.size(); i++) {
		dwl::Pose path = body_path_[i];
	}*/
}


void HierarchicalPlanners::publishBodyPath()
{
	body_path_msg_.header.stamp = ros::Time::now();
	body_path_msg_.poses.resize(body_path_.size());

	for (int i = 0; i < body_path_.size(); i++) {
		body_path_msg_.poses[i].pose.position.x = body_path_[i].position(0);
		body_path_msg_.poses[i].pose.position.y = body_path_[i].position(1);
		body_path_msg_.poses[i].pose.position.z = robot_pose_.position(2);//;body_path_[i].position(2);
	}
	body_path_pub_.publish(body_path_msg_);

	std::vector<dwl::Pose> empty_body_path_;
	body_path_.swap(empty_body_path_);
}

} //@namespace dwl_planners



int main(int argc, char **argv)
{
	ros::init(argc, argv, "hierarchical_planner");

	ros::NodeHandle node;
	dwl_planners::HierarchicalPlanners planner(node);

	planner.init();
	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);
		while(ros::ok()) {
			planner.publishBodyPath();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("hierarchical_planner exception: %s", e.what());
		return -1;
	}

	return 0;
}
