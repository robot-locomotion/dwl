#include <dwl_planners/ConstrainedWholeBodyPlanner.h>


namespace dwl_planners
{

ConstrainedWholeBodyPlanner::ConstrainedWholeBodyPlanner()
{
	desired_state_.base_pos << 0, 0, 0, 0, 0, -0.1;
	current_state_.joint_pos = Eigen::VectorXd::Zero(2);
	current_state_.joint_pos << 0.6, -1.5;
	current_state_.joint_vel = Eigen::VectorXd::Zero(2);
	current_state_.joint_acc = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff << 9.33031, 27.6003;

	floating_base_system_.LZ.active = true;
	floating_base_system_.LZ.id = 0;
}


ConstrainedWholeBodyPlanner::~ConstrainedWholeBodyPlanner()
{

}


void ConstrainedWholeBodyPlanner::init()
{
	// Initializes Ipopt solver
	dwl::solver::IpoptNLP* ipopt_solver = new dwl::solver::IpoptNLP();
	dwl::solver::OptimizationSolver* solver = ipopt_solver;

	planning_.init(solver);


	// Initializes the dynamical system constraint
	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::model::ConstrainedDynamicalSystem* system = new dwl::model::ConstrainedDynamicalSystem();
	dwl::model::DynamicalSystem* dynamical_system = system;


	dwl::rbd::BodySelector active_contact;
	active_contact.push_back("foot");
	system->setActiveEndEffectors(active_contact);

	dynamical_system->modelFromURDFFile(model_file, &floating_base_system_, true);






	dwl::LocomotionState weights;
	weights.base_pos = 1000 * dwl::rbd::Vector6d::Ones();
	weights.base_vel = 10 * dwl::rbd::Vector6d::Ones();
	weights.joint_eff = 0.001 * Eigen::Vector2d::Ones();


	dwl::model::Cost* state_tracking_cost = new dwl::model::StateTrackingEnergyCost();
	state_tracking_cost->setWeights(weights);
	dwl::model::Cost* control_cost = new dwl::model::ControlEnergyCost();
	control_cost->setWeights(weights);


	planning_.addDynamicalSystem(dynamical_system);
	planning_.addCost(state_tracking_cost);
	planning_.addCost(control_cost);


	planning_.setHorizon(3);
}


bool ConstrainedWholeBodyPlanner::compute()
{
	return planning_.compute(current_state_, desired_state_, 1000);
}


void ConstrainedWholeBodyPlanner::publishWholeBodyTrajectory()
{
	// Publishing the motion plan if there is at least one subscriber
//	if (motion_plan_pub_.getNumSubscribers() > 0) {
		robot_trajectory_msg_.header.stamp = ros::Time::now();

		std::vector<dwl::LocomotionState> trajectory = planning_.getWholeBodyTrajectory();

		for (unsigned int i = 0; i < trajectory.size(); i++) {
			std::cout << "\n\nPoint = " << i << std::endl;
			std::cout << "base_pos = " << trajectory[i].base_pos.transpose() << std::endl;
			std::cout << "joint_pos = " << trajectory[i].joint_pos.transpose() << std::endl;
			std::cout << "base_vel = " << trajectory[i].base_vel.transpose() << std::endl;
			std::cout << "joint_vel = " << trajectory[i].joint_vel.transpose() << std::endl;
			std::cout << "base_acc = " << trajectory[i].base_acc.transpose() << std::endl;
			std::cout << "joint_acc = " << trajectory[i].joint_acc.transpose() << std::endl;
			std::cout << "base_eff = " << trajectory[i].base_eff.transpose() << std::endl;
			std::cout << "joint_eff = " << trajectory[i].joint_eff.transpose() << std::endl;
		}

		robot_trajectory_msg_.trajectory.resize(trajectory.size());
//	}
}

} //@namespace dwl_planners




int main(int argc, char **argv)
{
	ros::init(argc, argv, "constrained_whole_body_planner");

	dwl_planners::ConstrainedWholeBodyPlanner planner;

	planner.init();
	ros::spinOnce();

//	try {
//		ros::Rate loop_rate(100);
//
//		while (ros::ok()) {
			if (planner.compute()) {
				planner.publishWholeBodyTrajectory();
//				planner.publishContactSequence();
//				planner.publishContactRegions();
			}
//			ros::spinOnce();
//			loop_rate.sleep();
//		}
//	} catch (std::runtime_error& e) {
//		ROS_ERROR("hierarchical_planner exception: %s", e.what());
//		return -1;
//	}

	return 0;
}

