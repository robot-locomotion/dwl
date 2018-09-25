#include <dwl/RobotStates.h>


namespace dwl
{

RobotStates::RobotStates() : num_joints_(0), num_feet_(0), force_threshold_(0.)
{

}


RobotStates::~RobotStates()
{

}


void RobotStates::reset(model::FloatingBaseSystem& fbs,
						model::WholeBodyKinematics& wkin,
				        model::WholeBodyDynamics& wdyn)
{
	// Creating the floating-base system and whole-body kinematics shared pointers
	fbs_ = std::make_shared<model::FloatingBaseSystem>(fbs);
	wkin_ = std::make_shared<model::WholeBodyKinematics>(wkin);
	wdyn_ = std::make_shared<model::WholeBodyDynamics>(wdyn);

	// Setting the whole-body dynamics from floating-base model
	wkin_->reset(fbs);
	wdyn_->reset(fbs, wkin);

	// Getting some system properties
	num_joints_ = fbs_->getJointDoF();
	num_feet_ = fbs_->getNumberOfEndEffectors(model::FOOT);
	feet_ = fbs_->getEndEffectorList(model::FOOT);

	// Getting the default position of the CoM system w.r.t. the base frame
	Eigen::VectorXd q0 = fbs_->getDefaultPosture();
	com_pos_B_ = wkin_->computeCoM(dwl::SE3(), q0);
}


void RobotStates::setForceThreshold(double force_threshold)
{
	force_threshold_ = force_threshold;
}


const WholeBodyState& RobotStates::getWholeBodyState(const ReducedBodyState& state)
{
	// Adding the time
	ws_.time = state.time;

	// From the reduced-body state we do not know the joint states, so we neglect
	// the joint-related components of the CoM. Therefore, we transform the
	// CoM states assuming that CoM is fixed-point in the base
	Eigen::Vector3d com_pos_W =
			frame_tf_.fromBaseToWorldFrame(com_pos_B_,
										   state.getCoMSE3().getRPY());
	se3::SE3 b_X_c(Eigen::Matrix3d::Identity(), -com_pos_B_);
	ws_.setBaseSE3(dwl::SE3(state.getCoMSE3().data.act(b_X_c)));
	ws_.setBaseVelocity_W(
			dwl::Motion(state.getCoMVelocity_W().data.se3Action(b_X_c)));
	ws_.setBaseAcceleration_W(
			dwl::Motion(state.getCoMAcceleration_W().data.se3Action(b_X_c)));


	// Adding the contact positions, velocities, accelerations and condition
	// w.r.t the base frame
	for (unsigned int f = 0; f < num_feet_; ++f) {
		std::string name = feet_[f];

		// Setting up the contact position
		se3::SE3 foot_pos_B = state.getFootSE3_B(name).data;
		ws_.setContactSE3_B(name,
				dwl::SE3(foot_pos_B.translation() - b_X_c.translation(),
						 foot_pos_B.rotation()));

		// Setting up the contact velocity
		ws_.setContactVelocity_B(name, state.getFootVelocity_B(name));

		// Setting up the contact acceleration
		ws_.setContactAcceleration_B(name, state.getFootAcceleration_B(name));

		// Setting up the contact condition
		dwl::SE3Map::const_iterator it = state.support_region.find(name);
		if (it != state.support_region.end())
			ws_.setContactCondition(name, true);
		else
			ws_.setContactCondition(name, false);
	}

	// Adding the joint positions, velocities and accelerations
	ws_.setJointPosition(Eigen::VectorXd::Zero(num_joints_));
	ws_.setJointVelocity(Eigen::VectorXd::Zero(num_joints_));
	ws_.setJointAcceleration(Eigen::VectorXd::Zero(num_joints_));

	// Computing the joint positions
	wkin_->computeJointPosition(ws_.joint_pos,
								ws_.getContactSE3_B());

	// Computing the joint velocities
	wkin_->computeJointVelocity(ws_.joint_vel,
								ws_.getJointPosition(),
								ws_.getContactVelocity_B());

	// Computing the joint accelerations
	wkin_->computeJointAcceleration(ws_.joint_acc,
									ws_.getJointPosition(),
									ws_.getJointVelocity(),
									ws_.getContactAcceleration_B());

	// Setting up the desired joint efforts equals to zero
	ws_.joint_eff = Eigen::VectorXd::Zero(num_joints_);


	return ws_;
}


const ReducedBodyState& RobotStates::getReducedBodyState(const WholeBodyState& state)
{
	// Adding the actual time
	rs_.time = state.time;

	// Getting the world to base transformation
	Eigen::Vector3d base_t = state.getBaseSE3().getTranslation();
	Eigen::Matrix3d base_R = state.getBaseSE3().getRotation();

	// Computing the CoM position, velocity and acceleration
	// Neglecting the joint accelerations components
	se3::SE3 b_X_c(Eigen::Matrix3d::Identity(), com_pos_B_);
	rs_.setCoMSE3(dwl::SE3(state.getBaseSE3().data.act(b_X_c)));

	Eigen::Vector3d com, com_d, com_dd;
	wkin_->computeCoMRate(com, com_d, com_dd,
						  state.getBaseSE3(), state.getJointPosition(),
						  state.getBaseVelocity_W(), state.getJointVelocity(),
						  state.getBaseAcceleration_W(), state.getJointAcceleration());
	rs_.setCoMVelocity_W(dwl::Motion(com_d, state.getBaseVelocity_W().getAngular()));
	rs_.setCoMAcceleration_W(dwl::Motion(com_dd, state.getBaseAcceleration_W().getAngular()));

	// Computing the CoP in the world frame
	Eigen::Vector3d cop_B;
	wdyn_->computeCenterOfPressure(cop_B,
								   state.getContactWrench_B(),
								   state.getContactSE3_B());
	rs_.setCoPPosition_W(base_t + base_R * cop_B);

	// Getting the support region w.r.t the world frame. The support region
	// is defined by the active contacts
	dwl::model::ElementList active_contacts;
	wdyn_->getActiveContacts(active_contacts,
							 state.getContactWrench_B(),
							 force_threshold_);
	rs_.support_region.clear();
	for (unsigned int i = 0; i < active_contacts.size(); ++i) {
		std::string name = active_contacts[i];

		// Setting up the contact position
		se3::SE3 contact_pos_B = state.getContactSE3_B(name).data;
		rs_.setFootSE3_B(name,
				dwl::SE3(contact_pos_B.translation() - b_X_c.translation(),
						 contact_pos_B.rotation()));

		// Setting up the foot velocities
		rs_.setFootVelocity_B(name, state.getContactVelocity_B(name));

		// Setting up the foot accelerations
		rs_.setFootAcceleration_B(name, state.getContactAcceleration_B(name));

		// Computing the support region
		rs_.setSupportRegion(name,
				state.getBaseSE3().data.act(state.getContactSE3_B(name).data));
	}

	return rs_;
}


const WholeBodyTrajectory& RobotStates::getWholeBodyTrajectory(const ReducedBodyTrajectory& trajectory)
{
	// Getting the number of points defined in the reduced-body trajectory
	unsigned int num_points = trajectory.size();

	// Resizing the full trajectory vector
	wt_.clear();
	wt_.resize(num_points);

	// Getting the full trajectory
	for (unsigned int k = 0; k < num_points; ++k)
		wt_[k] = getWholeBodyState(trajectory[k]);

	return wt_;
}


const ReducedBodyTrajectory& RobotStates::getReducedBodyTrajectory(const WholeBodyTrajectory& trajectory)
{
	// Getting the number of points defined in the whole-body trajectory
	unsigned int num_points = trajectory.size();

	// Resizing the full trajectory vector
	rt_.clear();
	rt_.resize(num_points);

	// Getting the reduced trajectory

	for (unsigned int k = 0; k < num_points; ++k)
		rt_[k] = getReducedBodyState(trajectory[k]);

	return rt_;
}

} //@namespace dwl
