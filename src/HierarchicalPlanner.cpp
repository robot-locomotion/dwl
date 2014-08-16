#include <dwl_planners/HierarchicalPlanner.h>


namespace dwl_planners
{

HierarchicalPlanners::HierarchicalPlanners(ros::NodeHandle node) : node_(node), planning_ptr_(NULL), body_path_solver_ptr_(NULL),
		base_frame_("base_link"), world_frame_("odom")
{

}


HierarchicalPlanners::~HierarchicalPlanners()
{
	delete planning_ptr_, body_path_solver_ptr_;
}


void HierarchicalPlanners::init()
{
	//  Setup of the locomotion approach
	planning_ptr_ = new dwl::planning::HierarchicalPlanning();

	reward_sub_ = node_.subscribe<terrain_server::RewardMap>("/reward_map", 1, &HierarchicalPlanners::rewardMapCallback, this);//new message_filters::Subscriber<terrain_server::RewardMap> (node_, "reward_map", 5);
	//tf_reward_sub_ = new tf::MessageFilter<terrain_server::RewardMap> (*reward_sub_, tf_listener_, world_frame_, 5);
	//tf_reward_sub_->registerCallback(boost::bind(&HierarchicalPlanners::rewardMapCallback, this, _1));
	obstacle_sub_ = node_.subscribe<terrain_server::ObstacleMap>("/obstacle_map", 1, &HierarchicalPlanners::obstacleMapCallback, this);
	body_goal_sub_ = node_.subscribe<dwl_planners::BodyGoal>("/body_goal", 1, &HierarchicalPlanners::resetGoalCallback, this);

	// Declaring the publisher of approximated body path
	body_path_pub_ = node_.advertise<nav_msgs::Path>("approximated_body_path", 1);
	contact_sequence_pub_ = node_.advertise<visualization_msgs::Marker>("contact_sequence", 1);

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

	// Initialization of the body planner
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
	node_.param("hierarchical_planner/body_planner/adjacency", adjacency_model_name, (std::string) "LatticeBasedBodyAdjacency");
	if (adjacency_model_name == "LatticeBasedBodyAdjacency")
		adjacency_ptr = new dwl::environment::LatticeBasedBodyAdjacency();
	else if (adjacency_model_name == "GridBasedBodyAdjacency")
		adjacency_ptr = new dwl::environment::GridBasedBodyAdjacency();
	else
		adjacency_ptr = new dwl::environment::LatticeBasedBodyAdjacency();

	// Setting the body planner
	body_path_solver_ptr_->setAdjacencyModel(adjacency_ptr);
	body_planner_.reset(body_path_solver_ptr_);

	// Setting the body and footstep planner, and the environment to the planning
	planning_ptr_->reset(&body_planner_, &footstep_planner_, &environment_);

	// Setting up the planner algorithm in the locomotion approach
	locomotor_.reset(planning_ptr_);

	// Initialization of the locomotion algorithm
	locomotor_.init();

	// Setting the allowed computation time of the locomotor
	double path_computation_time;
	if (node_.getParam("hierararchical_planner/body_planner/path_computation_time", path_computation_time))
		locomotor_.setComputationTime(path_computation_time, dwl::BodyPathSolver);
}


bool HierarchicalPlanners::compute()
{
	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
		tf_listener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), tf_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
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

	// Setting the current pose
	current_pose_.position = robot_position;
	current_pose_.orientation = robot_orientation;

	// Computing the locomotion plan
	bool solution = false;
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	if (pthread_mutex_trylock(&planner_lock_) == 0)
		solution = locomotor_.compute(current_pose_);
	else
		pthread_mutex_unlock(&planner_lock_);

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of optimization problem is %f seg.", duration);

	body_path_ = locomotor_.getBodyPath();
	contact_sequence_ = locomotor_.getContactSequence();


	return solution;
}


void HierarchicalPlanners::rewardMapCallback(const terrain_server::RewardMapConstPtr& msg)
{
	// Getting the transformation between the world to robot frame
	/*tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform(world_frame_, base_frame_, msg->header.stamp, tf_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}*/

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
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);

	if (pthread_mutex_trylock(&reward_lock_) == 0)
		locomotor_.setTerrainInformation(reward_map);
	else
		pthread_mutex_unlock(&reward_lock_);

	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration = (end_rt.tv_sec - start_rt.tv_sec) + 1e-9 * (end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of setting the information of the environment is %f seg.", duration);
}


void HierarchicalPlanners::obstacleMapCallback(const terrain_server::ObstacleMapConstPtr& msg)
{
	std::vector<dwl::Cell> obstacle_map;

	// Converting the messages to reward_map format
	dwl::Cell obstacle_cell;
	for (int i = 0; i < msg->cell.size(); i++) {
		// Filling the reward per every cell
		obstacle_cell.key.x = msg->cell[i].key_x;
		obstacle_cell.key.y = msg->cell[i].key_y;
		obstacle_cell.key.z = msg->cell[i].key_z;
		obstacle_cell.plane_size = msg->plane_size;
		obstacle_cell.height_size = msg->height_size;

		// Adding the reward cell to the queue
		obstacle_map.push_back(obstacle_cell);
	}

	// Adding the obstacle map
	if (pthread_mutex_trylock(&obstacle_lock_) == 0)
		locomotor_.setTerrainInformation(obstacle_map);
	else
		pthread_mutex_unlock(&obstacle_lock_);
}


void HierarchicalPlanners::resetGoalCallback(const dwl_planners::BodyGoalConstPtr& msg)
{
	dwl::Pose goal_pose;
	goal_pose.position[0] = msg->x;
	goal_pose.position[1] = msg->y;
	dwl::Orientation orientation(0, 0, msg->yaw);
	Eigen::Quaterniond q;
	orientation.getQuaternion(q);
	goal_pose.orientation = q;

	// Setting the new goal pose
	locomotor_.resetGoal(goal_pose);
}


void HierarchicalPlanners::publishBodyPath()
{
	// Publishing the body path if there is at least one subscriber
	if (body_path_pub_.getNumSubscribers() > 0) {
		body_path_msg_.header.stamp = ros::Time::now();
		body_path_msg_.poses.resize(body_path_.size());

		if (body_path_.size() != 0) {
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
}


void HierarchicalPlanners::publishContactSequence()
{
	// Publishing the contact sequence if there is at least one subscriber
	if (contact_sequence_pub_.getNumSubscribers() > 0) {
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
					std::cout << "Contact position 0 = " << contact_sequence_[i].position(0) << " " << contact_sequence_[i].position(1) << " " << contact_sequence_[i].position(2) << std::endl;
					contact_sequence_msg_.colors[i].r = 1.0;
					contact_sequence_msg_.colors[i].g = 0.0;
					contact_sequence_msg_.colors[i].b = 0.0;
					contact_sequence_msg_.colors[i].a = 1.0;
				} else if (end_effector == 1) {
					std::cout << "Contact position 1 = " << contact_sequence_[i].position(0) << " " << contact_sequence_[i].position(1) << " " << contact_sequence_[i].position(2) << std::endl;
					contact_sequence_msg_.colors[i].r = 1.0;
					contact_sequence_msg_.colors[i].g = 1.0;
					contact_sequence_msg_.colors[i].b = 0.0;
					contact_sequence_msg_.colors[i].a = 1.0;
				} else if (end_effector == 2) {
					std::cout << "Contact position 2 = " << contact_sequence_[i].position(0) << " " << contact_sequence_[i].position(1) << " " << contact_sequence_[i].position(2) << std::endl;
					contact_sequence_msg_.colors[i].r = 0.0;
					contact_sequence_msg_.colors[i].g = 1.0;
					contact_sequence_msg_.colors[i].b = 0.0;
					contact_sequence_msg_.colors[i].a = 1.0;
				} else if (end_effector == 3) {
					std::cout << "Contact position 3 = " << contact_sequence_[i].position(0) << " " << contact_sequence_[i].position(1) << " " << contact_sequence_[i].position(2) << std::endl;
					contact_sequence_msg_.colors[i].r = 0.0;
					contact_sequence_msg_.colors[i].g = 0.0;
					contact_sequence_msg_.colors[i].b = 1.0;
					contact_sequence_msg_.colors[i].a = 1.0;
				} else {
					contact_sequence_msg_.colors[i].r = 0.0;
					contact_sequence_msg_.colors[i].g = 1.0;
					contact_sequence_msg_.colors[i].b = 1.0;
					contact_sequence_msg_.colors[i].a = 1.0;
				}
				//contact_sequence_msg_.lifetime = 0;//ros::Duration();
			}
			contact_sequence_pub_.publish(contact_sequence_msg_);

			std::vector<dwl::Contact> empty_contact_squence;
			contact_sequence_.swap(empty_contact_squence);
		}
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
