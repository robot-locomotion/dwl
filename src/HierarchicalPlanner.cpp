#include <dwl_planners/HierarchicalPlanner.h>


namespace dwl_planners
{

HierarchicalPlanners::HierarchicalPlanners(ros::NodeHandle node) : node_(node), planning_ptr_(NULL), body_planner_ptr_(NULL),
		footstep_planner_ptr_(NULL), body_path_solver_ptr_(NULL), base_frame_("base_link"), world_frame_("odom")
{

}


HierarchicalPlanners::~HierarchicalPlanners()
{
	delete planning_ptr_, footstep_planner_ptr_, body_path_solver_ptr_;
}


void HierarchicalPlanners::init()
{
	// Setting the subscribers and publishers
	reward_sub_ = node_.subscribe<terrain_server::RewardMap>("/reward_map", 1, &HierarchicalPlanners::rewardMapCallback, this);
	obstacle_sub_ = node_.subscribe<terrain_server::ObstacleMap>("/obstacle_map", 1, &HierarchicalPlanners::obstacleMapCallback, this);
	body_goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/body_goal", 1, &HierarchicalPlanners::resetGoalCallback, this);

	// Declaring the publisher of approximated body path
	body_path_pub_ = node_.advertise<nav_msgs::Path>("approximated_body_path", 1);
	contact_sequence_pub_ = node_.advertise<dwl_planners::ContactSequence>("contact_sequence", 1);
	contact_sequence_rviz_pub_ = node_.advertise<visualization_msgs::Marker>("contact_sequence_markers", 1);

	// Initializing the mutex
	if (pthread_mutex_init(&reward_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&reward_lock_, NULL), strerror(pthread_mutex_init(&reward_lock_, NULL)));
	if (pthread_mutex_unlock(&reward_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&reward_lock_, NULL), strerror(pthread_mutex_init(&reward_lock_, NULL)));

	if (pthread_mutex_init(&obstacle_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&obstacle_lock_, NULL), strerror(pthread_mutex_init(&obstacle_lock_, NULL)));
	if (pthread_mutex_unlock(&obstacle_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&obstacle_lock_, NULL), strerror(pthread_mutex_init(&obstacle_lock_, NULL)));

	if (pthread_mutex_init(&planner_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&planner_lock_, NULL), strerror(pthread_mutex_init(&planner_lock_, NULL)));
	if (pthread_mutex_unlock(&planner_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&planner_lock_, NULL), strerror(pthread_mutex_init(&planner_lock_, NULL)));


	// Getting the base and world frame
	node_.param("base_frame", base_frame_, base_frame_);
	node_.param("world_frame", world_frame_, world_frame_);
	body_path_msg_.header.frame_id = world_frame_;
	contact_sequence_msg_.header.frame_id = world_frame_;
	contact_sequence_rviz_msg_.header.frame_id = world_frame_;


	//  Setting the locomotion approach
	planning_ptr_ = new dwl::planning::HierarchicalPlanning();

	// Initi the body planner
	initBodyPlanner();

	// Init the contact planner
	initContactPlanner();

	// Setting the body and footstep planner, and the robot and environment information to the planner
	planning_ptr_->reset(&robot_, body_planner_ptr_, footstep_planner_ptr_, &environment_);

	// Setting up the planner algorithm in the locomotion approach
	locomotor_.reset(planning_ptr_);

	// Initialization of the locomotion algorithm
	locomotor_.init();

	// Initialization of the hierarchical planner
	// Getting the goal pose
	double x, y, yaw;
	dwl::Pose goal_pose;
	node_.getParam("hierarchical_planner/goal/x", x);
	node_.getParam("hierarchical_planner/goal/y", y);
	node_.getParam("hierarchical_planner/goal/yaw", yaw);
	goal_pose.position[0] = x;
	goal_pose.position[1] = y;
	dwl::Orientation orientation(0, 0, yaw);
	Eigen::Quaterniond q;
	orientation.getQuaternion(q);
	goal_pose.orientation = q;
	locomotor_.resetGoal(goal_pose);

	// Setting the allowed computation time of the locomotor
	double path_computation_time;
	if (node_.getParam("hierarchical_planner/body_planner/path_computation_time", path_computation_time))
		locomotor_.setComputationTime(path_computation_time, dwl::BodyPathSolver);
}


void HierarchicalPlanners::initBodyPlanner()
{
	// Setting the contact planner
	std::string planner_name;
	node_.param("hierarchical_planner/body_planner/type", planner_name, (std::string) "SearchBasedBody");
	if (planner_name == "SearchBasedBody")
		body_planner_ptr_ = new dwl::planning::SearchBasedBodyMotionPlanning();
	else
		body_planner_ptr_ = new dwl::planning::SearchBasedBodyMotionPlanning();

	// Getting the body path solver
	std::string path_solver_name;
	node_.param("hierarchical_planner/body_planner/path_solver", path_solver_name, (std::string) "AnytimeRepairingAStar");
	if (path_solver_name == "AnytimeRepairingAStar")
		body_path_solver_ptr_ = new dwl::planning::AnytimeRepairingAStar();
	else if (path_solver_name == "AStar")
		body_path_solver_ptr_ = new dwl::planning::AStar();
	else if (path_solver_name == "Dijkstrap")
		body_path_solver_ptr_ = new dwl::planning::Dijkstrap();
	else
		body_path_solver_ptr_ = new dwl::planning::AnytimeRepairingAStar();


	// Getting the adjacency model
	std::string adjacency_model_name;
	dwl::environment::AdjacencyEnvironment* adjacency_ptr;

	node_.param("hierarchical_planner/body_planner/adjacency/name", adjacency_model_name, (std::string) "LatticeBasedBodyAdjacency");
	if (adjacency_model_name == "LatticeBasedBodyAdjacency") {
		adjacency_ptr = new dwl::environment::LatticeBasedBodyAdjacency();

		// Setting the features for the body planner
		bool potential_collision_enable, potential_orientation_enable;
		node_.param("hierarchical_planner/body_planner/adjacency/features/potential_leg_collision/enable", potential_collision_enable, false);
		node_.param("hierarchical_planner/body_planner/adjacency/features/potential_body_orientation/enable", potential_orientation_enable, false);

		if (potential_collision_enable) {
			dwl::environment::Feature* potential_collision_ptr = new dwl::environment::PotentialLegCollisionFeature();

			// Setting the weight
			double weight, default_weight = 1;
			node_.param("hierarchical_planner/body_planner/adjacency/features/potential_leg_collision/weight", weight, default_weight);
			potential_collision_ptr->setWeight(weight);
			adjacency_ptr->addFeature(potential_collision_ptr);
		}

		if (potential_orientation_enable) {
			dwl::environment::Feature* potential_orientation_ptr = new dwl::environment::PotentialBodyOrientationFeature();

			// Setting the weight
			double weight, default_weight = 1;
			node_.param("hierarchical_planner/body_planner/adjacency/features/potential_body_orientation/weight", weight, default_weight);
			potential_orientation_ptr->setWeight(weight);
			adjacency_ptr->addFeature(potential_orientation_ptr);
		}
	} else if (adjacency_model_name == "GridBasedBodyAdjacency")
		adjacency_ptr = new dwl::environment::GridBasedBodyAdjacency();
	else
		adjacency_ptr = new dwl::environment::LatticeBasedBodyAdjacency();


	// Setting the body planner
	body_path_solver_ptr_->setAdjacencyModel(adjacency_ptr);
	body_planner_ptr_->reset(body_path_solver_ptr_);

}


void HierarchicalPlanners::initContactPlanner()
{
	// Setting the contact planner
	std::string planner_name;
	node_.param("hierarchical_planner/contact_planner/type", planner_name, (std::string) "GreedyFootstep");
	if (planner_name == "GreedyFootstep")
		footstep_planner_ptr_ = new dwl::planning::GreedyFootstepPlanning();
	else
		footstep_planner_ptr_ = new dwl::planning::GreedyFootstepPlanning();

	// Setting the features for the footstep planner
	bool support_enable, collision_enable, orientation_enable;
	node_.param("hierarchical_planner/contact_planner/features/support_triangle/enable", support_enable, false);
	node_.param("hierarchical_planner/contact_planner/features/leg_collision/enable", collision_enable, false);
	node_.param("hierarchical_planner/contact_planner/features/body_orientation/enable", orientation_enable, false);
	if (support_enable) {
		dwl::environment::Feature* support_ptr = new dwl::environment::SupportTriangleFeature();

		// Setting the weight
		double weight, default_weight = 1;
		node_.param("hierarchical_planner/contact_planner/features/support_triangle/weight", weight, default_weight);
		support_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(support_ptr);
	}

	if (collision_enable) {
		dwl::environment::Feature* collision_ptr = new dwl::environment::LegCollisionFeature();

		// Setting the weight
		double weight, default_weight = 1;
		node_.param("hierarchical_planner/contact_planner/features/leg_collision/weight", weight, default_weight);
		collision_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(collision_ptr);
	}

	if (orientation_enable) {
		dwl::environment::Feature* orientation_ptr = new dwl::environment::BodyOrientationFeature();

		// Setting the weight
		double weight, default_weight = 1;
		node_.param("hierarchical_planner/contact_planner/features/body_orientation/weight", weight, default_weight);
		orientation_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(orientation_ptr);
	}

	// Setting the contact horizon
	double contact_horizon;
	node_.param("hierarchical_planner/contact_planner/horizon", contact_horizon, 0.0);
	footstep_planner_ptr_->setContactHorizon(contact_horizon);
}


bool HierarchicalPlanners::compute()
{
	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform, lf_transform, rf_transform, lh_transform, rh_transform;
	try {
		tf_listener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), tf_transform);

		tf_listener_.waitForTransform(world_frame_, "lf_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, "lf_foot", ros::Time(0), lf_transform);

		tf_listener_.waitForTransform(world_frame_, "rf_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, "rf_foot", ros::Time(0), rf_transform);

		tf_listener_.waitForTransform(world_frame_, "lh_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, "lh_foot", ros::Time(0), lh_transform);

		tf_listener_.waitForTransform(world_frame_, "rh_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, "rh_foot", ros::Time(0), rh_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
	}

	// Setting the current contacts of the robot
	std::vector<dwl::Contact> current_footholds;
	dwl::Contact contact;
	contact.end_effector = 0;
	contact.position << lf_transform.getOrigin()[0], lf_transform.getOrigin()[1], lf_transform.getOrigin()[2];
	current_footholds.push_back(contact);
	contact.end_effector = 1;
	contact.position << rf_transform.getOrigin()[0], rf_transform.getOrigin()[1], rf_transform.getOrigin()[2];
	current_footholds.push_back(contact);
	contact.end_effector = 2;
	contact.position << lh_transform.getOrigin()[0], lh_transform.getOrigin()[1], lh_transform.getOrigin()[2];
	current_footholds.push_back(contact);
	contact.end_effector = 3;
	contact.position << rh_transform.getOrigin()[0], rh_transform.getOrigin()[1], rh_transform.getOrigin()[2];
	current_footholds.push_back(contact);

	robot_.setCurrentContacts(current_footholds);

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

	// Setting the current pose
	current_pose_.position = robot_position;
	current_pose_.orientation = robot_orientation;

	// Computing the locomotion plan
	bool solution = false;

	if (pthread_mutex_trylock(&planner_lock_) == 0) {
		timespec start_rt, end_rt;
		clock_gettime(CLOCK_REALTIME, &start_rt);

		solution = locomotor_.compute(current_pose_);

		clock_gettime(CLOCK_REALTIME, &end_rt);
		double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
		ROS_INFO("The duration of computation of plan is %f seg.", duration);
	}
	else
		pthread_mutex_unlock(&planner_lock_);

	body_path_ = locomotor_.getBodyPath();
	contact_sequence_ = locomotor_.getContactSequence();

	return solution;
}


void HierarchicalPlanners::rewardMapCallback(const terrain_server::RewardMapConstPtr& msg)
{
	std::vector<dwl::RewardCell> reward_map;

	// Converting the messages to reward_map format
	dwl::RewardCell reward_cell;
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
	if (pthread_mutex_trylock(&reward_lock_) == 0) {
		timespec start_rt, end_rt;
		clock_gettime(CLOCK_REALTIME, &start_rt);

		locomotor_.setTerrainInformation(reward_map);

		clock_gettime(CLOCK_REALTIME, &end_rt);
		double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
		ROS_INFO("The duration of setting the reward information of the environment is %f seg.", duration);
	}
	else
		pthread_mutex_unlock(&reward_lock_);
}


void HierarchicalPlanners::obstacleMapCallback(const terrain_server::ObstacleMapConstPtr& msg)
{
	std::vector<dwl::Cell> obstacle_map;

	// Converting the messages to obstacle_map format
	dwl::Cell obstacle_cell;
	for (int i = 0; i < msg->cell.size(); i++) {
		// Filling the reward per every cell
		obstacle_cell.key.x = msg->cell[i].key_x;
		obstacle_cell.key.y = msg->cell[i].key_y;
		obstacle_cell.key.z = msg->cell[i].key_z;
		obstacle_cell.plane_size = msg->plane_size;
		obstacle_cell.height_size = msg->height_size;

		// Adding the obstacle cell to the queue
		obstacle_map.push_back(obstacle_cell);
	}

	// Adding the obstacle map
	if (pthread_mutex_trylock(&obstacle_lock_) == 0) {
		timespec start_rt, end_rt;
		clock_gettime(CLOCK_REALTIME, &start_rt);

		locomotor_.setTerrainInformation(obstacle_map);

		clock_gettime(CLOCK_REALTIME, &end_rt);
		double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
		ROS_INFO("The duration of setting the obstacle information of the environment is %f seg.", duration);
	}
	else
		pthread_mutex_unlock(&obstacle_lock_);
}


void HierarchicalPlanners::resetGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	dwl::Pose goal_pose;
	goal_pose.position[0] = msg->pose.position.x;
	goal_pose.position[1] = msg->pose.position.y;

	Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);//wxyz
	goal_pose.orientation = q;

	dwl::Orientation orientation(q);
	double roll, pitch, yaw;
	orientation.getRPY(roll, pitch, yaw);

	// Setting the new goal pose
	locomotor_.resetGoal(goal_pose);
	ROS_INFO("Setting the new goal pose (%f,%f,%f)", goal_pose.position[0], goal_pose.position[1], yaw);
}


void HierarchicalPlanners::publishBodyPath()
{
	// Publishing the body path if there is at least one subscriber
	if (body_path_pub_.getNumSubscribers() > 0) {
		body_path_msg_.header.stamp = ros::Time::now();
		body_path_msg_.poses.resize(body_path_.size());

		for (int i = 0; i < body_path_.size(); i++) {
			body_path_msg_.poses[i].pose.position.x = body_path_[i].position(0);
			body_path_msg_.poses[i].pose.position.y = body_path_[i].position(1);
			body_path_msg_.poses[i].pose.position.z = current_pose_.position(2);
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
	// Publishing the contact sequence if there is at least one subscriber
	if (contact_sequence_pub_.getNumSubscribers() > 0) {
		contact_sequence_msg_.header.stamp = ros::Time::now();
		contact_sequence_msg_.contacts.resize(contact_sequence_.size());
		for (int i = 0; i < contact_sequence_.size(); i++) {
			contact_sequence_msg_.contacts[i].end_effector = contact_sequence_[i].end_effector + 1; //This for compatibility with the controller
			contact_sequence_msg_.contacts[i].position.x = contact_sequence_[i].position(0);
			contact_sequence_msg_.contacts[i].position.y = contact_sequence_[i].position(1);
			contact_sequence_msg_.contacts[i].position.z = contact_sequence_[i].position(2);
		}

		contact_sequence_pub_.publish(contact_sequence_msg_);
	}

	// Publishing the contact sequence for visualization if there is at least one subscriber
	if (contact_sequence_rviz_pub_.getNumSubscribers() > 0) {
		contact_sequence_rviz_msg_.header.stamp = ros::Time::now();
		contact_sequence_rviz_msg_.type = visualization_msgs::Marker::SPHERE_LIST;
		contact_sequence_rviz_msg_.ns = "contact_points";
		contact_sequence_rviz_msg_.id = 0;
		contact_sequence_rviz_msg_.scale.x = 0.04;
		contact_sequence_rviz_msg_.scale.y = 0.04;
		contact_sequence_rviz_msg_.scale.z = 0.04;
		contact_sequence_rviz_msg_.action = visualization_msgs::Marker::ADD;

		contact_sequence_rviz_msg_.points.resize(contact_sequence_.size());
		contact_sequence_rviz_msg_.colors.resize(contact_sequence_.size());
		std::cout << "Contacts" << std::endl;
		for (int i = 0; i < contact_sequence_.size(); i++) {
			contact_sequence_rviz_msg_.points[i].x = contact_sequence_[i].position(0);
			contact_sequence_rviz_msg_.points[i].y = contact_sequence_[i].position(1);
			contact_sequence_rviz_msg_.points[i].z = contact_sequence_[i].position(2) + 0.03;

			int end_effector = contact_sequence_[i].end_effector;
			if (end_effector == 0) {//TODO Remove the offset
				std::cout << "1\t" << contact_sequence_[i].position(0) << "\t" << contact_sequence_[i].position(1) << "\t" << contact_sequence_[i].position(2)+0.575648 << std::endl;
				contact_sequence_rviz_msg_.colors[i].r = 0.45;
				contact_sequence_rviz_msg_.colors[i].g = 0.29;
				contact_sequence_rviz_msg_.colors[i].b = 0.09;
				contact_sequence_rviz_msg_.colors[i].a = 1.0;
			} else if (end_effector == 1) {
				std::cout << "2\t" << contact_sequence_[i].position(0) << "\t" << contact_sequence_[i].position(1) << "\t" << contact_sequence_[i].position(2)+0.575648 << std::endl;
				contact_sequence_rviz_msg_.colors[i].r = 1.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 0.0;
				contact_sequence_rviz_msg_.colors[i].a = 1.0;
			} else if (end_effector == 2) {
				std::cout << "3\t" << contact_sequence_[i].position(0) << "\t" << contact_sequence_[i].position(1) << "\t" << contact_sequence_[i].position(2)+0.575648 << std::endl;
				contact_sequence_rviz_msg_.colors[i].r = 0.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 0.0;
				contact_sequence_rviz_msg_.colors[i].a = 1.0;
			} else if (end_effector == 3) {
				std::cout << "4\t" << contact_sequence_[i].position(0) << "\t" << contact_sequence_[i].position(1) << "\t" << contact_sequence_[i].position(2)+0.575648 << std::endl;
				contact_sequence_rviz_msg_.colors[i].r = 0.09;
				contact_sequence_rviz_msg_.colors[i].g = 0.11;
				contact_sequence_rviz_msg_.colors[i].b = 0.7;
				contact_sequence_rviz_msg_.colors[i].a = 1.0;
			} else {
				contact_sequence_rviz_msg_.colors[i].r = 0.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 1.0;
				contact_sequence_rviz_msg_.colors[i].a = 1.0;
			}
			//contact_sequence_msg_.lifetime = 0;//ros::Duration();
		}
		contact_sequence_rviz_pub_.publish(contact_sequence_rviz_msg_);
	}

	// Cleaning old information
	std::vector<dwl::Contact> empty_contact_squence;
	contact_sequence_.swap(empty_contact_squence);
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

			if (planner.compute()) {
				planner.publishBodyPath();
				planner.publishContactSequence();
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("hierarchical_planner exception: %s", e.what());
		return -1;
	}

	return 0;
}
