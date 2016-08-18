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
		rbd::BodyVector3d::const_iterator foot_pos_it = state.foot_pos.find(name);
		if (foot_pos_it != state.foot_pos.end()) {
			Eigen::Vector3d foot_pos = foot_pos_it->second + com_pos_B_;
			ws_.contact_pos[name] = foot_pos;
			feet_pos[name] = foot_pos; // for IK computation
		}

		// Setting up the contact velocity
		rbd::BodyVector3d::const_iterator foot_vel_it = state.foot_vel.find(name);
		if (foot_vel_it != state.foot_vel.end())
			ws_.contact_vel[name] = foot_vel_it->second;

		// Setting up the contact acceleration
		rbd::BodyVector3d::const_iterator foot_acc_it = state.foot_acc.find(name);
		if (foot_acc_it != state.foot_acc.end())
			ws_.contact_acc[name] = foot_acc_it->second;


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
	Eigen::Vector3d base_rpy = state.getBaseRPY_W();
	Eigen::Matrix3d base_rotation = math::getRotationMatrix(base_rpy);

	// Computing the CoM position, velocity and acceleration
	// Neglecting the joint accelerations components
	rs_.com_pos =
			state.getBasePosition_W() + base_rotation * com_pos_B_;
	rs_.com_vel = fbs_.getSystemCoMRate(state.base_pos,
										state.joint_pos,
										state.base_vel,
										state.joint_vel);
	rs_.com_acc = state.getBaseAcceleration_W();

	rs_.angular_pos = state.getBaseRPY_W();
	rs_.angular_vel = state.getBaseAngularVelocity_W();
	rs_.angular_acc = state.getBaseAngularAcceleration_W();

	// Computing the CoP in the world frame
	Eigen::Vector3d cop_B;
	wdyn_.computeCenterOfPressure(cop_B,
								  state.contact_eff,
								  state.contact_pos,
								  feet_);
	rs_.cop = base_traslation + base_rotation * cop_B;

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
				base_rotation * state.getContactPosition_B(name);
	}

	// Adding the contact positions, velocities and accelerations
	// w.r.t the CoM frame
	for (unsigned int f = 0; f < num_feet_; f++) {
		std::string name = feet_[f];

		// Setting up the contact position
		rbd::BodyVectorXd::const_iterator contact_pos_it = state.contact_pos.find(name);
		if (contact_pos_it != state.contact_pos.end())
			rs_.foot_pos[name] = contact_pos_it->second - com_pos_B_;

		// Setting up the contact velocity
		rbd::BodyVectorXd::const_iterator contact_vel_it = state.contact_vel.find(name);
		if (contact_vel_it != state.contact_vel.end())
			rs_.foot_vel[name] = contact_vel_it->second;

		// Setting up the contact acceleration
		rbd::BodyVectorXd::const_iterator contact_acc_it = state.contact_acc.find(name);
		if (contact_acc_it != state.contact_acc.end())
			rs_.foot_acc[name] = contact_acc_it->second;
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
