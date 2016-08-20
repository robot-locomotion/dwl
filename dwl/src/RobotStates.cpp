#include <dwl/RobotStates.h>


namespace dwl
{

RobotStates::RobotStates() : num_joints_(0), num_feet_(0), force_threshold_(0.)
{

}


RobotStates::~RobotStates()
{

}


void RobotStates::reset(const model::WholeBodyDynamics& dynamics)
{
	// Resetting the dynamics, kinematics and system
	wdyn_ = dynamics;
	fbs_ = wdyn_.getFloatingBaseSystem();
	wkin_ = wdyn_.getWholeBodyKinematics();

	// Getting some system properties
	num_joints_ = fbs_.getJointDoF();
	num_feet_ = fbs_.getNumberOfEndEffectors(model::FOOT);
	feet_ = fbs_.getEndEffectorNames(model::FOOT);

	// Getting the default position of the CoM system w.r.t. the base frame
	Eigen::VectorXd q0 = fbs_.getDefaultPosture();
	com_pos_B_ = fbs_.getSystemCoM(rbd::Vector6d::Zero(), q0);
}


void RobotStates::setForceThreshold(double force_threshold)
{
	force_threshold_ = force_threshold;
}


const WholeBodyState& RobotStates::getWholeBodyState(const ReducedBodyState& state)
{
	// Adding the time
	ws_.time = state.time;

	// From the preview model we do not know the joint states, so we neglect
	// the joint-related components of the CoM. Therefore, we transform the
	// CoM states assuming that CoM is fixed-point in the base
	Eigen::Vector3d com_pos_W =
			frame_tf_.fromBaseToWorldFrame(com_pos_B_,
										   state.getRPY_W());
	ws_.setBasePosition_W(state.getCoMPosition_W() - com_pos_W);
	ws_.setBaseVelocity_W(computeBaseVelocity_W(state, com_pos_W));
	ws_.setBaseAcceleration_W(computeBaseAcceleration_W(state, com_pos_W));

	ws_.setBaseRPY_W(state.getRPY_W());
	ws_.setBaseAngularVelocity_W(state.getAngularVelocity_W());
	ws_.setBaseAngularAcceleration_W(state.getAngularAcceleration_W());


	// Adding the contact positions, velocities, accelerations and condition
	// w.r.t the base frame
	dwl::rbd::BodyVector3d feet_pos;
	for (unsigned int f = 0; f < num_feet_; f++) {
		std::string name = feet_[f];

		// Setting up the contact position
		Eigen::Vector3d contact_pos_B =	state.getFootPosition_B(name) + com_pos_B_;
		ws_.setContactPosition_B(name, contact_pos_B);
		feet_pos[name] = contact_pos_B; // for IK computation

		// Setting up the contact velocity
		Eigen::Vector3d contact_vel_W = state.getFootVelocity_W(name);
		ws_.setContactVelocity_W(name, contact_vel_W);

		// Setting up the contact acceleration
		Eigen::Vector3d contact_acc_W = state.getFootAcceleration_W(name);
		ws_.setContactAcceleration_W(name, contact_vel_W, contact_acc_W);

		// Setting up the contact condition
		rbd::BodyVector3d::const_iterator support_it = state.support_region.find(name);
		if (support_it != state.support_region.end())
			ws_.setContactCondition(name, true);
		else
			ws_.setContactCondition(name, false);
	}

	// Adding the joint positions, velocities and accelerations
	ws_.joint_pos = Eigen::VectorXd::Zero(num_joints_);
	ws_.joint_vel = Eigen::VectorXd::Zero(num_joints_);
	ws_.joint_acc = Eigen::VectorXd::Zero(num_joints_);

	// Computing the joint positions
	wkin_.computeInverseKinematics(ws_.joint_pos,
								   feet_pos);

	// Computing the joint velocities
	wkin_.computeJointVelocity(ws_.joint_vel,
							   ws_.joint_pos,
							   ws_.contact_vel,
							   feet_);

	// Computing the joint accelerations
	wkin_.computeJoinAcceleration(ws_.joint_acc,
								  ws_.joint_pos,
								  ws_.joint_vel,
								  ws_.contact_vel,
								  feet_);

	// Setting up the desired joint efforts equals to zero
	ws_.joint_eff = Eigen::VectorXd::Zero(num_joints_);


	return ws_;
}


const ReducedBodyState& RobotStates::getReducedBodyState(const WholeBodyState& state)
{
	// Adding the actual time
	rs_.time = state.time;

	// Getting the world to base transformation
	Eigen::Vector3d base_traslation = state.getBasePosition_W();
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(state.getBaseRPY_W());

	// Computing the CoM position, velocity and acceleration
	// Neglecting the joint accelerations components
	rs_.setCoMPosition_W(state.getBasePosition_W() + W_rot_B * com_pos_B_);
	rs_.setCoMVelocity_W(fbs_.getSystemCoMRate(state.base_pos,
											   state.joint_pos,
											   state.base_vel,
											   state.joint_vel));
	rs_.setCoMAcceleration_W(state.getBaseAcceleration_W());

	rs_.setRPY_W(state.getBaseRPY_W());
	rs_.setAngularVelocity_W(state.getBaseAngularVelocity_W());
	rs_.setAngularAcceleration_W(state.getBaseAngularAcceleration_W());

	// Computing the CoP in the world frame
	Eigen::Vector3d cop_B;
	wdyn_.computeCenterOfPressure(cop_B,
								  state.contact_eff,
								  state.contact_pos,
								  feet_);
	rs_.setCoPPosition_W(base_traslation + W_rot_B * cop_B);

	// Getting the support region w.r.t the world frame. The support region
	// is defined by the active contacts
	rbd::BodySelector active_contacts;
	wdyn_.getActiveContacts(active_contacts,
							state.contact_eff,
							force_threshold_);
	rs_.support_region.clear();
	for (unsigned int i = 0; i < active_contacts.size(); i++) {
		std::string name = active_contacts[i];

		rs_.support_region[name] = base_traslation +
				W_rot_B * state.getContactPosition_B(name);
	}

	// Adding the contact positions, velocities and accelerations
	// w.r.t the CoM frame
	for (unsigned int f = 0; f < num_feet_; f++) {
		std::string name = feet_[f];

		// Setting up the contact position
		rs_.setFootPosition_B(name, state.getContactPosition_B(name) - com_pos_B_);

		// Setting up the contact velocity
		rs_.setFootVelocity_W(name, state.getContactVelocity_W(name));

		// Setting up the contact acceleration
		rs_.setFootAcceleration_W(name, state.getContactAcceleration_W(name));
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
	for (unsigned int k = 0; k < num_points; k++)
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

	for (unsigned int k = 0; k < num_points; k++)
		rt_[k] = getReducedBodyState(trajectory[k]);

	return rt_;
}


Eigen::Vector3d RobotStates::computeBaseVelocity_W(const ReducedBodyState& state,
												   const Eigen::Vector3d& com_pos_W)
{
	// Computing the base velocity using the equation:
	// Xd^W_com = Xd^W_base + omega_base x X^W_com/base
	return state.getCoMVelocity_W() - state.getAngularVelocity_W().cross(com_pos_W);
}


Eigen::Vector3d RobotStates::computeBaseAcceleration_W(const ReducedBodyState& state,
													   const Eigen::Vector3d& com_pos_W)
{
	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega =
			math::skewSymmetricMatrixFromVector(state.getAngularVelocity_W());
	Eigen::Matrix3d C_omega_dot =
			math::skewSymmetricMatrixFromVector(state.getAngularAcceleration_W());

	// Computing the base acceleration relatives to the CoM, which is expressed
	// in the world frame. Here we use the equation:
	// Xdd^W_com = Xdd^W_base + [C(wd^W) + C(w^W) * C(w^W)] X^W_com/base
	return state.getCoMAcceleration_W() -
			(C_omega_dot + C_omega * C_omega) * com_pos_W;
}

} //@namespace dwl
