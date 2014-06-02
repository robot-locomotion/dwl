#include <dwl_planners/HierarchicalPlanner.h>


namespace dwl_planners
{

HierarchicalPlanners::HierarchicalPlanners(ros::NodeHandle node) : node_(node), planning_ptr_(NULL), solver_ptr_(NULL), cost_map_ptr_(NULL)
{

}


HierarchicalPlanners::~HierarchicalPlanners()
{

}


void HierarchicalPlanners::init()
{
	// Setting the subscription to the reward_map message
	reward_sub_ = node_.subscribe("reward_map", 1, &HierarchicalPlanners::rewardMapCallback, this);

	//  Setup of the locomotion approach
	planning_ptr_ = new dwl::planning::HierarchicalPlanning();

	// Initialization of planning algorithm, which includes the initizalization and setup of solver algorithm
	solver_ptr_ = new dwl::planning::DijkstrapAlgorithm();
	planning_ptr_->reset(solver_ptr_);

	cost_map_ptr_ = new dwl::planning::CostMap();

	// Setting up the planner algorithm in the locomotion approach
	locomotor_.reset(planning_ptr_);
	locomotor_.addCost(cost_map_ptr_);

	locomotor_.init();

	dwl::planning::BodyPose start_pose, goal_pose;
	start_pose.position[0] = -1.5;
	start_pose.position[1] = 0.0;
	goal_pose.position[0] = 5.0;
	goal_pose.position[1] = 0.0;
	locomotor_.update(start_pose, goal_pose);
}


void HierarchicalPlanners::rewardMapCallback(const reward_map_server::RewardMapConstPtr& msg)
{
	std::vector<dwl::environment::Cell> reward_map;
	locomotor_.setGridMapResolution(msg->cell_size);

	// Converting the messages to reward_map format
	dwl::environment::Cell reward_cell;
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
	Eigen::Vector3d robot_state = Eigen::Vector3d::Zero();
	cost_map_ptr_->setCostMap(reward_map);

	// Computing the locomotion plan
	locomotor_.compute();
}

} //@namespace dwl_planners



int main(int argc, char **argv)
{
	ros::init(argc, argv, "hierarchical_planner");

	ros::NodeHandle node;
	dwl_planners::HierarchicalPlanners planner(node);

	planner.init();
	ros::spin();




	// Initizalization and computing of the whole-body locomotion problem
	//dwl::planning::BodyPose start, goal;




/*
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
*/
	return 0;
}
