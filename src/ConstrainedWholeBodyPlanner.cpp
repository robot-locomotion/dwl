#include <dwl_planners/ConstrainedWholeBodyPlanner.h>


namespace dwl_planners
{

ConstrainedWholeBodyPlanner::ConstrainedWholeBodyPlanner() : solver_(NULL)
{
	ipopt_solver_ = new dwl::solver::IpoptNLP();
	constrained_system_ = new dwl::model::ConstrainedDynamicalSystem();


	current_state_.joint_pos = Eigen::VectorXd::Zero(2);
	current_state_.joint_pos << 0.6, -1.5;
	current_state_.joint_vel = Eigen::VectorXd::Zero(2);
	current_state_.joint_acc = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff << 9.33031, 27.6003;
}


ConstrainedWholeBodyPlanner::~ConstrainedWholeBodyPlanner()
{

}


void ConstrainedWholeBodyPlanner::init()
{
	solver_ = ipopt_solver_;




	floating_base_system_.LZ.active = true;
	floating_base_system_.LZ.id = 0;


	dwl::rbd::BodySelector active_contact;
	active_contact.push_back("foot");
	constrained_system_->setActiveEndEffectors(active_contact);


	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::model::DynamicalSystem* dynamical_system = constrained_system_;
	dynamical_system->modelFromURDFFile(model_file, &floating_base_system_, true);


	dwl::LocomotionState desired_state;
	desired_state.base_pos << 0, 0, 0, 0, 0, -0.1;


	dwl::LocomotionState weights;
	weights.base_pos = 1000 * dwl::rbd::Vector6d::Ones();
	weights.base_vel = 10 * dwl::rbd::Vector6d::Ones();
	weights.joint_eff = 0.001 * Eigen::Vector2d::Ones();


	dwl::model::Cost* state_tracking_cost = new dwl::model::StateTrackingEnergyCost();
	state_tracking_cost->setDesiredState(desired_state);
	state_tracking_cost->setWeights(weights);
	dwl::model::Cost* control_cost = new dwl::model::ControlEnergyCost();
	control_cost->setWeights(weights);
	constrained_system_->setStartingState(current_state_);


	ipopt_solver_->getIpopt().getOptimizationModel().addDynamicalSystem(dynamical_system);
	ipopt_solver_->getIpopt().getOptimizationModel().addCost(state_tracking_cost);
	ipopt_solver_->getIpopt().getOptimizationModel().addCost(control_cost);


	ipopt_solver_->getIpopt().getOptimizationModel().getDynamicalSystem()->setInitialState(current_state_);
	ipopt_solver_->getIpopt().getOptimizationModel().setHorizon(3);


//	planning_ptr_->reset(&robot_, solver_, environment_);


	solver_->init();
}


bool ConstrainedWholeBodyPlanner::compute()
{
	return solver_->compute();
}


void ConstrainedWholeBodyPlanner::publishWholeBodyTrajectory()
{
	// Publishing the motion plan if there is at least one subscriber
//	if (motion_plan_pub_.getNumSubscribers() > 0) {
//		body_path_msg_.header.stamp = ros::Time::now();
//		body_path_msg_.poses.resize(body_path_.size());
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

