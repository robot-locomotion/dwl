#include <dwl_planners/HierarchicalPlanner.h>


namespace dwl_planners
{

HierarchicalPlanners::HierarchicalPlanners(ros::NodeHandle node) : node_(node), planning_ptr_(NULL), solver_ptr_(NULL)
{
	reward_sub_ = new message_filters::Subscriber<reward_map_server::RewardMap> (node_, "reward_map", 5);
	tf_reward_sub_ = new tf::MessageFilter<reward_map_server::RewardMap> (*reward_sub_, tf_listener_, "world", 5);
	tf_reward_sub_->registerCallback(boost::bind(&HierarchicalPlanners::rewardMapCallback, this, _1));

	// Declaring the publisher of approximated body path
	body_path_pub_ = node_.advertise<nav_msgs::Path>("approximated_body_path", 1);
	contact_sequence_pub_ = node_.advertise<visualization_msgs::Marker>("contact_sequence", 1);

	body_path_msg_.header.frame_id = "world";
	contact_sequence_msg_.header.frame_id = "world";
}


HierarchicalPlanners::~HierarchicalPlanners()
{
	delete planning_ptr_, solver_ptr_;
}


void HierarchicalPlanners::init()
{
	//  Setup of the locomotion approach
	planning_ptr_ = new dwl::planning::HierarchicalPlanning();

	// Initialization of planning algorithm, which includes the initizalization and setup of solver algorithm
	solver_ptr_ = new dwl::planning::AnytimeRepairingAStar();//new dwl::planning::AStar();//new dwl::planning::Dijkstrap();
	dwl::environment::AdjacencyEnvironment* grid_adjacency_ptr = new dwl::environment::GridBasedBodyAdjacency();
	dwl::environment::AdjacencyEnvironment* lattice_adjacency_ptr = new dwl::environment::LatticeBasedBodyAdjacency();
	solver_ptr_->setAdjacencyModel(lattice_adjacency_ptr);//solver_ptr_->setAdjacencyModel(grid_adjacency_ptr);
	body_planner_.reset(solver_ptr_);
	planning_ptr_->reset(&body_planner_, &footstep_planner_, &environment_);//(solver_ptr_, &environment_);

	// Setting up the planner algorithm in the locomotion approach
	locomotor_.reset(planning_ptr_);

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

	std::vector<dwl::Cell> reward_map;

	// Converting the messages to reward_map format
	dwl::Cell reward_cell;
	for (int i = 0; i < msg->cell.size(); i++) {
		// Filling the reward per every cell
		reward_cell.key.x = msg->cell[i].key_x;
		reward_cell.key.y = msg->cell[i].key_y;
		reward_cell.key.z = msg->cell[i].key_z;
		reward_cell.reward = msg->cell[i].reward;
		reward_cell.plane_size = msg->plane_size;
		reward_cell.height_size = msg->height_size;

		// Adding the reward cell to the queue
		reward_map.push_back(reward_cell);
	}

	// Adding the cost map
	locomotor_.setTerrainInformation(reward_map);

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
//	start_pose.position[0] = robot_position(0);
//	start_pose.position[1] = robot_position(1);
	goal_pose.position[0] = 3.50;
	goal_pose.position[1] = -0.85;
	dwl::Orientation orientation(0, 0, 1.45);
	Eigen::Quaterniond q;
	orientation.getQuaternion(q);
	goal_pose.orientation = q;
	locomotor_.resetGoal(goal_pose);


	// Computing the locomotion plan
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	locomotor_.compute(robot_pose_);
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);

	body_path_ = locomotor_.getBodyPath();
	contact_sequence_ = locomotor_.getContactSequence();
}


void HierarchicalPlanners::publishBodyPath()
{
	body_path_msg_.header.stamp = ros::Time::now();
	body_path_msg_.poses.resize(body_path_.size());

	if (body_path_.size() != 0) {
		for (int i = 0; i < body_path_.size(); i++) {
			body_path_msg_.poses[i].pose.position.x = body_path_[i].position(0);
			body_path_msg_.poses[i].pose.position.y = body_path_[i].position(1);
			body_path_msg_.poses[i].pose.position.z = robot_pose_.position(2);
			body_path_msg_.poses[i].pose.orientation.w = body_path_[i].orientation.w();
			body_path_msg_.poses[i].pose.orientation.x = body_path_[i].orientation.x();
			body_path_msg_.poses[i].pose.orientation.y = body_path_[i].orientation.y();
			body_path_msg_.poses[i].pose.orientation.z = body_path_[i].orientation.z();
		}
		body_path_pub_.publish(body_path_msg_);

		std::vector<dwl::Pose> empty_body_path;
		body_path_.swap(empty_body_path);
	}
}


void HierarchicalPlanners::publishContactSequence()
{
	contact_sequence_msg_.header.stamp = ros::Time::now();
	contact_sequence_msg_.type = visualization_msgs::Marker::SPHERE_LIST;
	contact_sequence_msg_.ns = "contact_points";
	contact_sequence_msg_.id = 0;
	contact_sequence_msg_.scale.x = 0.05;
	contact_sequence_msg_.scale.y = 0.05;
	contact_sequence_msg_.scale.z = 0.05;
	contact_sequence_msg_.action = visualization_msgs::Marker::ADD;

	contact_sequence_msg_.points.resize(contact_sequence_.size());
	contact_sequence_msg_.colors.resize(contact_sequence_.size());
	if (contact_sequence_.size() != 0) {
		for (int i = 0; i < contact_sequence_.size(); i++) {
			contact_sequence_msg_.points[i].x = contact_sequence_[i].position(0);
			contact_sequence_msg_.points[i].y = contact_sequence_[i].position(1);
			contact_sequence_msg_.points[i].z = contact_sequence_[i].position(2) + 0.0275;

			int end_effector = contact_sequence_[i].end_effector;
			if (end_effector == 0) {
				contact_sequence_msg_.colors[i].r = 1.0f;
				contact_sequence_msg_.colors[i].g = 0.0f;
				contact_sequence_msg_.colors[i].b = 0.0f;
				contact_sequence_msg_.colors[i].a = 1.0;
			} else if (end_effector == 1) {
				contact_sequence_msg_.colors[i].r = 0.0f;
				contact_sequence_msg_.colors[i].g = 1.0f;
				contact_sequence_msg_.colors[i].b = 0.0f;
				contact_sequence_msg_.colors[i].a = 1.0;
			} else if (end_effector == 2) {
				contact_sequence_msg_.colors[i].r = 0.0f;
				contact_sequence_msg_.colors[i].g = 0.0f;
				contact_sequence_msg_.colors[i].b = 1.0f;
				contact_sequence_msg_.colors[i].a = 1.0;
			} else if (end_effector == 3) {
				contact_sequence_msg_.colors[i].r = 1.0f;
				contact_sequence_msg_.colors[i].g = 1.0f;
				contact_sequence_msg_.colors[i].b = 0.0f;
				contact_sequence_msg_.colors[i].a = 1.0;
			} else {
				contact_sequence_msg_.colors[i].r = 0.0f;
				contact_sequence_msg_.colors[i].g = 1.0f;
				contact_sequence_msg_.colors[i].b = 1.0f;
				contact_sequence_msg_.colors[i].a = 1.0;
			}
			//contact_sequence_msg_.lifetime = 0;//ros::Duration();
		}
		contact_sequence_pub_.publish(contact_sequence_msg_);

		std::vector<dwl::Contact> empty_contact_squence;
		contact_sequence_.swap(empty_contact_squence);
	}
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
		ros::Rate loop_rate(1000);
		while(ros::ok()) {
			planner.publishBodyPath();
			planner.publishContactSequence();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("hierarchical_planner exception: %s", e.what());
		return -1;
	}

	return 0;
}
