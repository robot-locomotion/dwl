#include <dwl_planners/FootstepHierarchicalPlanner.h>


namespace dwl_planners
{

FootstepHierarchicalPlanners::FootstepHierarchicalPlanners(ros::NodeHandle node) : private_node_(node),
		body_planner_ptr_(NULL), footstep_planner_ptr_(NULL),
		body_path_solver_ptr_(NULL), adjacency_ptr_(NULL), base_frame_("base_link"),
		world_frame_("odom")
{

}


FootstepHierarchicalPlanners::~FootstepHierarchicalPlanners()
{
	delete footstep_planner_ptr_;
	delete body_path_solver_ptr_;
}


void FootstepHierarchicalPlanners::init()
{
	// Setting the subscribers and publishers
	reward_sub_ = node_.subscribe<terrain_server::RewardMap>("/reward_map", 1,
			&FootstepHierarchicalPlanners::rewardMapCallback, this);
	obstacle_sub_ = node_.subscribe<terrain_server::ObstacleMap>("/obstacle_map", 1,
			&FootstepHierarchicalPlanners::obstacleMapCallback, this);
	body_goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/body_goal", 1,
			&FootstepHierarchicalPlanners::resetGoalCallback, this);

	// Declaring the publisher of approximated body path
	body_path_pub_ = node_.advertise<nav_msgs::Path>("body_path", 1);
	contact_sequence_pub_ = node_.advertise<dwl_planners::ContactSequence>("contact_sequence", 1);
	contact_sequence_rviz_pub_ = node_.advertise<visualization_msgs::Marker>("contact_sequence_markers", 1);
	contact_regions_pub_ = node_.advertise<dwl_planners::ContactRegion>("contact_regions", 1);

	// Initializing the mutex
	if (pthread_mutex_init(&reward_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&reward_lock_, NULL),
				strerror(pthread_mutex_init(&reward_lock_, NULL)));
	if (pthread_mutex_unlock(&reward_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&reward_lock_, NULL),
				strerror(pthread_mutex_init(&reward_lock_, NULL)));

	if (pthread_mutex_init(&obstacle_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&obstacle_lock_, NULL),
				strerror(pthread_mutex_init(&obstacle_lock_, NULL)));
	if (pthread_mutex_unlock(&obstacle_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&obstacle_lock_, NULL),
				strerror(pthread_mutex_init(&obstacle_lock_, NULL)));

	if (pthread_mutex_init(&planner_lock_, NULL) != 0)
		ROS_ERROR("Could not initialized mutex (%i : %s).", pthread_mutex_init(&planner_lock_, NULL),
				strerror(pthread_mutex_init(&planner_lock_, NULL)));
	if (pthread_mutex_unlock(&planner_lock_) != 0)
		ROS_ERROR("Could not unlocked mutex (%i : %s).", pthread_mutex_init(&planner_lock_, NULL),
				strerror(pthread_mutex_init(&planner_lock_, NULL)));


	// Getting the base and world frame
	private_node_.param("base_frame", base_frame_, base_frame_);
	private_node_.param("world_frame", world_frame_, world_frame_);
	body_path_msg_.header.frame_id = world_frame_;
	contact_sequence_msg_.header.frame_id = world_frame_;
	contact_sequence_rviz_msg_.header.frame_id = world_frame_;
	contact_regions_msg_.header.frame_id = world_frame_;

	// Initializes the robot properties
	initRobot();

	// Initializes the body planner
	initBodyPlanner();

	// Initializes the contact planner
	initContactPlanner();

	// Setting the body and footstep planner, and the robot and environment information to the planner
	planning_.reset(&robot_, body_planner_ptr_, footstep_planner_ptr_, &environment_);

	// Initialization of the footstep hierarchical planner algorithm
	planning_.initPlan();

	// Initialization of the hierarchical planner
	// Getting the goal pose
	double x, y, yaw;
	dwl::Pose goal_pose;
	private_node_.getParam("goal/x", x);
	private_node_.getParam("goal/y", y);
	private_node_.getParam("goal/yaw", yaw);
	goal_pose.position[0] = x;
	goal_pose.position[1] = y;
	
	Eigen::Quaterniond q = dwl::math::getQuaternion(Eigen::Vector3d(0.0, 0.0, yaw));
	goal_pose.orientation = q;
	planning_.resetGoal(goal_pose);

	// Setting the allowed computation time of the footstep hierarchical planner
	double path_computation_time;
	if (private_node_.getParam("body_planner/path_computation_time", path_computation_time))
		planning_.setComputationTime(path_computation_time, dwl::BodyPathSolver);
}


void FootstepHierarchicalPlanners::initRobot()
{
	// Initializes the robot properties
	std::string properties_path;
	if (private_node_.getParam("robot/properties", properties_path))
		robot_.read(properties_path);
	else
		ROS_WARN("The properties was not defined, this could be necessary.");

	// Initializes robot body primitives
	std::string body_primitive_path;
	if (private_node_.getParam("robot/body_movement_primitives", body_primitive_path))
		robot_.getBodyMotorPrimitive().read(body_primitive_path);
	else
		ROS_WARN("The body movement primitives was not defined, this could be necessary.");
}


void FootstepHierarchicalPlanners::initBodyPlanner()
{
	// Setting the contact planner
	std::string planner_name;
	private_node_.param("body_planner/type", planner_name, (std::string) "SearchBasedBody");
	if (planner_name == "SearchBasedBody")
		body_planner_ptr_ = new dwl::locomotion::SearchBasedBodyMotionPlanning();
	else
		body_planner_ptr_ = new dwl::locomotion::SearchBasedBodyMotionPlanning();

	// Getting the body path solver
	std::string path_solver_name;
	private_node_.param("body_planner/path_solver", path_solver_name, (std::string) "AnytimeRepairingAStar");
	if (path_solver_name == "AnytimeRepairingAStar") {
		// Reads the initial inflation value
		double inflation = 3;
		private_node_.param("body_planner/initial_inflation", inflation, inflation);
		body_path_solver_ptr_ = new dwl::solver::AnytimeRepairingAStar(inflation);
	} else if (path_solver_name == "AStar")
		body_path_solver_ptr_ = new dwl::solver::AStar();
	else if (path_solver_name == "Dijkstrap")
		body_path_solver_ptr_ = new dwl::solver::Dijkstrap();
	else
		body_path_solver_ptr_ = new dwl::solver::AnytimeRepairingAStar();


	// Getting the adjacency model
	std::string adjacency_model_name;
	std::string path = "body_planner/adjacency/";
	private_node_.param(path + "name", adjacency_model_name, (std::string) "LatticeBasedBodyAdjacency");
	if (adjacency_model_name == "LatticeBasedBodyAdjacency")
		adjacency_ptr_ = new dwl::environment::LatticeBasedBodyAdjacency();
	else
		if (adjacency_model_name == "GridBasedBodyAdjacency")
		adjacency_ptr_ = new dwl::environment::GridBasedBodyAdjacency();
	else
		ROS_ERROR("Wrong adjacency model");

	// Setting the features for the body planner
	bool potential_collision_enable, potential_orientation_enable;
	private_node_.param(path + "features/potential_leg_collision/enable", potential_collision_enable, false);
	private_node_.param(path + "features/potential_body_orientation/enable", potential_orientation_enable, false);

	if (potential_collision_enable) {
		double clearance, collision;
		private_node_.param(path + "features/potential_leg_collision/clearance", clearance, 0.0);
		private_node_.param(path + "features/potential_leg_collision/collision", collision, 0.3);
		dwl::environment::Feature* potential_collision_ptr =
				new dwl::environment::PotentialLegCollisionFeature(clearance, collision);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/potential_leg_collision/weight", weight, default_weight);
		potential_collision_ptr->setWeight(weight);
		adjacency_ptr_->addFeature(potential_collision_ptr);
	}

	if (potential_orientation_enable) {
		double max_roll, max_pitch;
		private_node_.param(path + "features/potential_body_orientation/max_roll", max_roll, 30.0);
		private_node_.param(path + "features/potential_body_orientation/max_pitch", max_pitch, 30.0);
		dwl::environment::Feature* potential_orientation_ptr =
				new dwl::environment::PotentialBodyOrientationFeature(max_roll, max_pitch);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/potential_body_orientation/weight", weight, default_weight);
		potential_orientation_ptr->setWeight(weight);
		adjacency_ptr_->addFeature(potential_orientation_ptr);
	}

	// Setting the body planner
	body_path_solver_ptr_->setAdjacencyModel(adjacency_ptr_);
	body_planner_ptr_->reset(body_path_solver_ptr_);
}


void FootstepHierarchicalPlanners::initContactPlanner()
{
	// Setting the contact planner
	std::string planner_name;
	std::string path = "contact_planner/";
	bool remove_enable;
	double distance_threshold;
	private_node_.param(path + "remove_footholds/enable", remove_enable, false);
	if (remove_enable)
		private_node_.param(path + "remove_footholds/distance_threshold", distance_threshold,
				std::numeric_limits<double>::max());

	private_node_.param(path + "type", planner_name, (std::string) "GreedyFootstep");
	if (planner_name == "GreedyFootstep")
		footstep_planner_ptr_ = new dwl::locomotion::GreedyFootstepPlanning(remove_enable, distance_threshold);
	else
		footstep_planner_ptr_ = new dwl::locomotion::GreedyFootstepPlanning(remove_enable, distance_threshold);

	// Setting the features for the footstep planner
	bool support_enable, collision_enable, orientation_enable, kinematic_enable, stance_enable;
	private_node_.param(path + "features/support_triangle/enable", support_enable, false);
	private_node_.param(path + "features/leg_collision/enable", collision_enable, false);
	private_node_.param(path + "features/body_orientation/enable", orientation_enable, false);
	private_node_.param(path + "features/kinematic_feasibility/enable", kinematic_enable, false);
	private_node_.param(path + "features/stance_posture/enable", stance_enable, false);
	if (support_enable) {
		double stable_inradii, unstable_inradii;
		private_node_.param(path + "features/support_triangle/stable_inraddi", stable_inradii, 0.13);
		private_node_.param(path + "features/support_triangle/unstable_inraddi", unstable_inradii, 0.08);
		dwl::environment::Feature* support_ptr =
				new dwl::environment::SupportTriangleFeature(stable_inradii, unstable_inradii);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/support_triangle/weight", weight, default_weight);
		support_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(support_ptr);
	}

	if (collision_enable) {
		double clearance, collision;
		private_node_.param(path + "features/leg_collision/clearance", clearance, 0.0);
		private_node_.param(path + "features/leg_collision/collision", collision, 0.3);
		dwl::environment::Feature* collision_ptr =
				new dwl::environment::LegCollisionFeature(clearance, collision);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/leg_collision/weight", weight, default_weight);
		collision_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(collision_ptr);
	}

	if (orientation_enable) {
		double max_roll, max_pitch;
		private_node_.param(path + "features/body_orientation/max_roll", max_roll, 30.0);
		private_node_.param(path + "features/body_orientation/max_pitch", max_pitch, 30.0);
		dwl::environment::Feature* orientation_ptr =
				new dwl::environment::BodyOrientationFeature(max_roll, max_pitch);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/body_orientation/weight", weight, default_weight);
		orientation_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(orientation_ptr);
	}

	if (kinematic_enable) {
		double kin_lim_x, kin_lim_y, stable_displacement;
		private_node_.param(path + "features/kinematic_feasibility/kin_lim_x", kin_lim_x, 0.16);
		private_node_.param(path + "features/kinematic_feasibility/kin_lim_x", kin_lim_y, 0.08);
		private_node_.param(path + "features/kinematic_feasibility/stable_displacement", stable_displacement, 0.15);
		dwl::environment::Feature* kinematic_ptr =
				new dwl::environment::KinematicFeasibilityFeature(kin_lim_x, kin_lim_y, stable_displacement);

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/kinematic_feasibility/weight", weight, default_weight);
		kinematic_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(kinematic_ptr);
	}

	if (stance_enable) {
		double kin_lim_x, kin_lim_y, stable_displacement;
		private_node_.param(path + "features/kinematic_feasibility/kin_lim_x", kin_lim_x, 0.16);
		private_node_.param(path + "features/kinematic_feasibility/kin_lim_x", kin_lim_y, 0.08);
		private_node_.param(path + "features/kinematic_feasibility/stable_displacement", stable_displacement, 0.15);
		dwl::environment::Feature* stance_ptr = new dwl::environment::StancePostureFeature();

		// Setting the weight
		double weight, default_weight = 1;
		private_node_.param(path + "features/stance_posture/weight", weight, default_weight);
		stance_ptr->setWeight(weight);
		footstep_planner_ptr_->addFeature(stance_ptr);
	}

	// Setting the contact horizon
	double contact_horizon;
	private_node_.param(path + "horizon", contact_horizon, 0.0);
	footstep_planner_ptr_->setContactHorizon(contact_horizon);
}


bool FootstepHierarchicalPlanners::compute()
{
	bool solution = false;

	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform, lf_transform, rf_transform, lh_transform, rh_transform;
	try {
		tf_listener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), tf_transform);

		tf_listener_.waitForTransform(base_frame_, "lf_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(base_frame_, "lf_foot", ros::Time(0), lf_transform);

		tf_listener_.waitForTransform(base_frame_, "rf_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(base_frame_, "rf_foot", ros::Time(0), rf_transform);

		tf_listener_.waitForTransform(base_frame_, "lh_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(base_frame_, "lh_foot", ros::Time(0), lh_transform);

		tf_listener_.waitForTransform(base_frame_, "rh_foot", ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(base_frame_, "rh_foot", ros::Time(0), rh_transform);

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
		if (pthread_mutex_trylock(&planner_lock_) == 0) {
			timespec start_rt, end_rt;
			clock_gettime(CLOCK_REALTIME, &start_rt);

			solution = planning_.computePlan(current_pose_);

			if (solution) {
				clock_gettime(CLOCK_REALTIME, &end_rt);
				double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
				ROS_INFO("The duration of computation of plan is %f seg.", duration);
			}
		}
		else
			pthread_mutex_unlock(&planner_lock_);

		body_path_ = planning_.getBodyPath();
		contact_search_region_ = footstep_planner_ptr_->getContactSearchRegions();
		contact_sequence_ = planning_.getContactSequence();
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");

		return false;
	}

	return solution;
}


void FootstepHierarchicalPlanners::rewardMapCallback(const terrain_server::RewardMapConstPtr& msg)
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

		planning_.setEnvironmentInformation(reward_map);

		clock_gettime(CLOCK_REALTIME, &end_rt);
		double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
		ROS_INFO("The duration of setting the reward information of the environment is %f seg.", duration);
	}
	else
		pthread_mutex_unlock(&reward_lock_);
}


void FootstepHierarchicalPlanners::obstacleMapCallback(const terrain_server::ObstacleMapConstPtr& msg)
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

		planning_.setEnvironmentInformation(obstacle_map);

		clock_gettime(CLOCK_REALTIME, &end_rt);
		double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
		ROS_INFO("The duration of setting the obstacle information of the environment is %f seg.", duration);
	} else
		pthread_mutex_unlock(&obstacle_lock_);
}


void FootstepHierarchicalPlanners::resetGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	dwl::Pose goal_pose;
	goal_pose.position[0] = msg->pose.position.x;
	goal_pose.position[1] = msg->pose.position.y;

	Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
						 msg->pose.orientation.y, msg->pose.orientation.z);
	goal_pose.orientation = q;

	double yaw = dwl::math::getYaw(dwl::math::getRPY(q));

	// Setting the new goal pose
	planning_.resetGoal(goal_pose);
	ROS_INFO("Setting the new goal pose (%f,%f,%f)", goal_pose.position[0], goal_pose.position[1], yaw);
}


void FootstepHierarchicalPlanners::publishBodyPath()
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


void FootstepHierarchicalPlanners::publishContactSequence()
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
		contact_sequence_rviz_msg_.scale.x = 0.03;
		contact_sequence_rviz_msg_.scale.y = 0.03;
		contact_sequence_rviz_msg_.scale.z = 0.03;
		contact_sequence_rviz_msg_.action = visualization_msgs::Marker::ADD;

		contact_sequence_rviz_msg_.points.resize(contact_sequence_.size());
		contact_sequence_rviz_msg_.colors.resize(contact_sequence_.size());
		for (int i = 0; i < contact_sequence_.size(); i++) {
			contact_sequence_rviz_msg_.points[i].x = contact_sequence_[i].position(0);
			contact_sequence_rviz_msg_.points[i].y = contact_sequence_[i].position(1);
			contact_sequence_rviz_msg_.points[i].z = contact_sequence_[i].position(2) + 0.015;

			int end_effector = contact_sequence_[i].end_effector;
			if (end_effector == 0) {
				contact_sequence_rviz_msg_.colors[i].r = 0.45;
				contact_sequence_rviz_msg_.colors[i].g = 0.29;
				contact_sequence_rviz_msg_.colors[i].b = 0.09;
				contact_sequence_rviz_msg_.colors[i].a = 0.5;
			} else if (end_effector == 1) {
				contact_sequence_rviz_msg_.colors[i].r = 1.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 0.0;
				contact_sequence_rviz_msg_.colors[i].a = 0.5;
			} else if (end_effector == 2) {
				contact_sequence_rviz_msg_.colors[i].r = 0.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 0.0;
				contact_sequence_rviz_msg_.colors[i].a = 0.5;
			} else if (end_effector == 3) {
				contact_sequence_rviz_msg_.colors[i].r = 0.09;
				contact_sequence_rviz_msg_.colors[i].g = 0.11;
				contact_sequence_rviz_msg_.colors[i].b = 0.7;
				contact_sequence_rviz_msg_.colors[i].a = 0.5;
			} else {
				contact_sequence_rviz_msg_.colors[i].r = 0.0;
				contact_sequence_rviz_msg_.colors[i].g = 1.0;
				contact_sequence_rviz_msg_.colors[i].b = 1.0;
				contact_sequence_rviz_msg_.colors[i].a = 0.5;
			}
			//contact_sequence_msg_.lifetime = 0;//ros::Duration();
		}
		contact_sequence_rviz_pub_.publish(contact_sequence_rviz_msg_);
	}

	// Cleaning old information
	std::vector<dwl::Contact> empty_contact_squence;
	contact_sequence_.swap(empty_contact_squence);
}


void FootstepHierarchicalPlanners::publishContactRegions()
{
	// Publishing the contacts regions
		if (contact_regions_pub_.getNumSubscribers() > 0) {
			contact_regions_msg_.header.stamp = ros::Time::now();
			contact_regions_msg_.regions.resize(contact_search_region_.size());

			double size_x, size_y, size_z;
			Eigen::Vector3d cell_position;
			for (int i = 0; i < contact_search_region_.size(); i++) {
				int end_effector = contact_search_region_[i].end_effector;
				Eigen::Vector3d nominal_contact = contact_search_region_[i].position;
				dwl::SearchArea contact_region = contact_search_region_[i].region;

				// Computing the width, height and center of the cell
				size_x = contact_region.max_x - contact_region.min_x;
				size_y = contact_region.max_y - contact_region.min_y;
				size_z = contact_region.max_z - contact_region.min_z;
				cell_position(0) = nominal_contact(0);
				cell_position(1) = nominal_contact(1);
				cell_position(2) = nominal_contact(2);

				contact_regions_msg_.regions[i].size.x = size_x;
				contact_regions_msg_.regions[i].size.y = size_y;
				contact_regions_msg_.regions[i].size.z = size_z;
				contact_regions_msg_.regions[i].pose.position.x = cell_position(0);
				contact_regions_msg_.regions[i].pose.position.y = cell_position(1);
				contact_regions_msg_.regions[i].pose.position.z = cell_position(2);

				contact_regions_msg_.regions[i].pose.orientation.w = 1; //body_path_msg_.poses[i].pose.orientation.w;
				contact_regions_msg_.regions[i].pose.orientation.x = 0; //body_path_msg_.poses[i].pose.orientation.x;
				contact_regions_msg_.regions[i].pose.orientation.y = 0; //body_path_msg_.poses[i].pose.orientation.y;
				contact_regions_msg_.regions[i].pose.orientation.z = 0; //body_path_msg_.poses[i].pose.orientation.z;

				if (end_effector == 0) {
					contact_regions_msg_.regions[i].color.r = 0.45;
					contact_regions_msg_.regions[i].color.g = 0.29;
					contact_regions_msg_.regions[i].color.b = 0.09;
					contact_regions_msg_.regions[i].color.a = 1.0;
				} else if (end_effector == 1) {
					contact_regions_msg_.regions[i].color.r = 1.0;
					contact_regions_msg_.regions[i].color.g = 1.0;
					contact_regions_msg_.regions[i].color.b = 0.0;
					contact_regions_msg_.regions[i].color.a = 1.0;
				} else if (end_effector == 2) {
					contact_regions_msg_.regions[i].color.r = 0.0;
					contact_regions_msg_.regions[i].color.g = 1.0;
					contact_regions_msg_.regions[i].color.b = 0.0;
					contact_regions_msg_.regions[i].color.a = 1.0;
				} else if (end_effector == 3) {
					contact_regions_msg_.regions[i].color.r = 0.09;
					contact_regions_msg_.regions[i].color.g = 0.11;
					contact_regions_msg_.regions[i].color.b = 0.7;
					contact_regions_msg_.regions[i].color.a = 1.0;
				} else {
					contact_regions_msg_.regions[i].color.r = 0.0;
					contact_regions_msg_.regions[i].color.g = 1.0;
					contact_regions_msg_.regions[i].color.b = 1.0;
					contact_regions_msg_.regions[i].color.a = 1.0;
				}
			}

			contact_regions_pub_.publish(contact_regions_msg_);
		}

		// Cleaning old information
		std::vector<dwl::ContactSearchRegion> empty_contact_regions;
		contact_search_region_.swap(empty_contact_regions);




//	if (contact_regions_pub_.getNumSubscribers() > 0) {
//		contact_regions_msg_.header.stamp = ros::Time::now();
//		contact_regions_msg_.cell_width = footstep_width_;
//		contact_regions_msg_.cell_height = footstep_height_;
//		contact_regions_msg_.cells.resize(nominal_contacts_.size());
//		contact_regions_msg_.colors.resize(nominal_contacts_.size());
//		for (int i = 0; i < nominal_contacts_.size(); i++) {
//			contact_regions_msg_.cells[i].position.x = nominal_contacts_[i].position(0) + footstep_height_offset_;
//			contact_regions_msg_.cells[i].position.y = nominal_contacts_[i].position(1) + footstep_width_offset_;
//			contact_regions_msg_.cells[i].position.z = nominal_contacts_[i].position(2);
//
//			contact_regions_msg_.cells[i].orientation.w = 1; //body_path_msg_.poses[i].pose.orientation.w;
//			contact_regions_msg_.cells[i].orientation.x = 0; //body_path_msg_.poses[i].pose.orientation.x;
//			contact_regions_msg_.cells[i].orientation.y = 0; //body_path_msg_.poses[i].pose.orientation.y;
//			contact_regions_msg_.cells[i].orientation.z = 0; //body_path_msg_.poses[i].pose.orientation.z;
//			// TODO orientation
//
//			int end_effector = nominal_contacts_[i].end_effector;
//			if (end_effector == 0) {
//				contact_regions_msg_.colors[i].r = 0.45;
//				contact_regions_msg_.colors[i].g = 0.29;
//				contact_regions_msg_.colors[i].b = 0.09;
//				contact_regions_msg_.colors[i].a = 1.0;
//			} else if (end_effector == 1) {
//				contact_regions_msg_.colors[i].r = 1.0;
//				contact_regions_msg_.colors[i].g = 1.0;
//				contact_regions_msg_.colors[i].b = 0.0;
//				contact_regions_msg_.colors[i].a = 1.0;
//			} else if (end_effector == 2) {
//				contact_regions_msg_.colors[i].r = 0.0;
//				contact_regions_msg_.colors[i].g = 1.0;
//				contact_regions_msg_.colors[i].b = 0.0;
//				contact_regions_msg_.colors[i].a = 1.0;
//			} else if (end_effector == 3) {
//				contact_regions_msg_.colors[i].r = 0.09;
//				contact_regions_msg_.colors[i].g = 0.11;
//				contact_regions_msg_.colors[i].b = 0.7;
//				contact_regions_msg_.colors[i].a = 1.0;
//			} else {
//				contact_regions_msg_.colors[i].r = 0.0;
//				contact_regions_msg_.colors[i].g = 1.0;
//				contact_regions_msg_.colors[i].b = 1.0;
//				contact_regions_msg_.colors[i].a = 1.0;
//			}
//		}
//
//		contact_regions_pub_.publish(contact_regions_msg_);
//	}
//
//	// Cleaning old information
//	std::vector<dwl::Contact> empty_contact_squence;
//	nominal_contacts_.swap(empty_contact_squence);
}

} //@namespace dwl_planners



int main(int argc, char **argv)
{
	ros::init(argc, argv, "hierarchical_planner");

	dwl_planners::FootstepHierarchicalPlanners planner;

	planner.init();
	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);

		while (ros::ok()) {
			if (planner.compute()) {
				planner.publishBodyPath();
				planner.publishContactSequence();
				planner.publishContactRegions();
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch (std::runtime_error& e) {
		ROS_ERROR("hierarchical_planner exception: %s", e.what());
		return -1;
	}

	return 0;
}
