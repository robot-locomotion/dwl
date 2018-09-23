#include <dwl/ReducedBodyState.h>


namespace dwl
{

ReducedBodyState::ReducedBodyState() : time(0.),
		cop(Eigen::Vector3d::Zero())

{

}


ReducedBodyState::~ReducedBodyState()
{

}


const double& ReducedBodyState::getTime() const
{
	return time;
}


const dwl::SE3& ReducedBodyState::getCoMSE3() const
{
	return com_pos;
}


const dwl::SE3& ReducedBodyState::getCoMSE3_H()
{
	// Getting the RPY vector of the horizontal frame
	Eigen::Vector3d rpy = com_pos.getRPY();
	rpy(rbd::X) *= 0.;
	rpy(rbd::Y) *= 0.;

	// Setting the new rotation matrix
	se3_.setTranslation(com_pos.getTranslation());
	se3_.setRotation(math::getRotationMatrix(rpy));
	return se3_;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_W() const
{
	return com_vel;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_B()
{
	// Mapping the velocity in the inertial frame to the base one
	se3::SE3 w_R_b(com_pos.getRotation(),
				   Eigen::Vector3d::Zero());
	motion_.data = com_vel.data.se3ActionInverse(w_R_b);
	return motion_;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_H()
{
	// Mapping the velocity in the inertial frame to the horizontal one
	se3::SE3 w_R_b(getCoMSE3_H().getRotation(),
				   Eigen::Vector3d::Zero());
	motion_.data = com_vel.data.se3ActionInverse(w_R_b);
	return motion_;
}


const Eigen::Vector3d& ReducedBodyState::getRPYVelocity_W()
{
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(com_pos.getRotation()).transpose();
	vec3_ = EAR * com_vel.getAngular();
	return vec3_;
}


const dwl::Motion& ReducedBodyState::getCoMAcceleration_W() const
{
	return com_acc;
}


const dwl::Motion& ReducedBodyState::getCoMAcceleration_B()
{
	// Mapping the acceleration in the inertial frame to the base one
	se3::SE3 w_R_b(com_pos.getRotation(),
				   Eigen::Vector3d::Zero());
	motion_.data = com_acc.data.se3ActionInverse(w_R_b);
	return motion_;
}


const dwl::Motion& ReducedBodyState::getCoMAcceleration_H()
{
	// Mapping the acceleration in the inertial frame to the horizontal one
	se3::SE3 w_R_b(getCoMSE3_H().getRotation(),
				   Eigen::Vector3d::Zero());
	motion_.data = com_acc.data.se3ActionInverse(w_R_b);
	return motion_;
}


Eigen::Vector3d ReducedBodyState::getRPYAcceleration_W()
{
	// rpy_ddot = EAR^-1 * (omega_dot - EAR_dot * rpy_dot)
	Eigen::Vector3d rpy = com_pos.getRPY();
	Eigen::Vector3d rpy_d = getRPYVelocity_W();
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(rpy).inverse();
	Eigen::Matrix3d EARinv_dot =
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_d);
	vec3_ = EAR * (com_vel.getAngular() - EARinv_dot * rpy_d);
	return vec3_;
}


const Eigen::Vector3d& ReducedBodyState::getCoPPosition_W() const
{
	return cop;
}


const dwl::SE3& ReducedBodyState::getFootSE3_W(SE3Iterator it)
{
	// Mapping the contact SE3 expressed in the inertial frame to the base one
	se3_.data = com_pos.data.act(it->second.data);
	return se3_;
}


const dwl::SE3& ReducedBodyState::getFootSE3_W(const std::string& name)
{
	// Getting the contact iterator
	SE3Iterator it = foot_pos.find(name);
	if (it == foot_pos.end())
		return null_se3_;

	return getFootSE3_W(it);
}


dwl::SE3Map ReducedBodyState::getFootSE3_W()
{
	dwl::SE3Map pos_W;
	for (SE3Iterator it = foot_pos.begin();
			it != foot_pos.end(); ++it) {
		std::string name = it->first;
		pos_W[name] = getFootSE3_W(it);
	}

	return pos_W;
}


const dwl::SE3& ReducedBodyState::getFootSE3_B(SE3Iterator it) const
{
	return it->second;
}


const dwl::SE3& ReducedBodyState::getFootSE3_B(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getFootSE3_B().find(name);
	if (it == foot_pos.end())
		return null_se3_;

	return getFootSE3_B(it);
}


const dwl::SE3Map& ReducedBodyState::getFootSE3_B() const
{
	return foot_pos;
}


const dwl::SE3& ReducedBodyState::getFootSE3_H(SE3Iterator it)
{
	// Mapping the contact SE3 expressed in the horizontal frame to the base one
	se3_.data = getCoMSE3_H().data.act(it->second.data);
	return se3_;
}


const dwl::SE3& ReducedBodyState::getFootSE3_H(const std::string& name)
{
	// Getting the contact iterator
	SE3Iterator it = getFootSE3_B().find(name);
	if (it == foot_pos.end())
		return null_se3_;

	return getFootSE3_H(it);
}


dwl::SE3Map ReducedBodyState::getFootSE3_H()
{
	dwl::SE3Map pos_H;
	for (SE3Iterator it = getFootSE3_B().begin();
			it != getFootSE3_B().end(); ++it) {
		std::string name = it->first;
		pos_H[name] = getFootSE3_H(it);
	}

	return pos_H;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_W(MotionIterator it)
{
	const se3::SE3& w_X_b = com_pos.data;
	motion_.data = com_vel.data + it->second.data.se3Action(w_X_b);
	return motion_;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_W(const std::string& name)
{
	// Getting the contact iterator
	MotionIterator it = getFootVelocity_B().find(name);
	if (it == foot_vel.end())
		return null_motion_;

	return getFootVelocity_W(it);
}


dwl::MotionMap ReducedBodyState::getFootVelocity_W()
{
	dwl::MotionMap vel_W;
	for (MotionIterator it = getFootVelocity_B().begin();
			it != getFootVelocity_B().end(); ++it) {
		std::string name = it->first;
		vel_W[name] = getFootVelocity_W(it);
	}

	return vel_W;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_B(MotionIterator it) const
{
	return it->second;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_B(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = foot_vel.find(name);
	if (it == foot_vel.end())
		return null_motion_;

	return getFootVelocity_B(it);
}


const dwl::MotionMap& ReducedBodyState::getFootVelocity_B() const
{
	return foot_vel;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_H(MotionIterator it)
{
	const se3::SE3& h_X_b = getCoMSE3_H().data;
	motion_.data = it->second.data.se3Action(h_X_b);
	return motion_;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_H(const std::string& name)
{
	MotionIterator it = getFootVelocity_B().find(name);
	return getFootVelocity_H(it);
}


dwl::MotionMap ReducedBodyState::getFootVelocity_H()
{
	dwl::MotionMap foot_vel_H;
	for (MotionIterator foot_it = getFootVelocity_B().begin();
			foot_it != getFootVelocity_B().end(); ++foot_it) {
		std::string name = foot_it->first;
		foot_vel_H[name] = getFootVelocity_H(foot_it);
	}

	return foot_vel_H;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_W(MotionIterator it)
{
	const se3::SE3& w_X_b = com_pos.data;
	const se3::Motion& v = getFootVelocity_W(it->first).data;
	motion_.data = com_acc.data + it->second.data.se3Action(w_X_b);
	motion_.data.linear() += v.angular().cross(v.linear());
	return motion_;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_W(const std::string& name)
{
	MotionIterator it = getFootAcceleration_B().find(name);
	if (it == foot_acc.end())
		return null_motion_;

	return getFootAcceleration_W(it);
}


dwl::MotionMap ReducedBodyState::getFootAcceleration_W()
{
	dwl::MotionMap acc_W;
	for (MotionIterator it = getFootAcceleration_B().begin();
			it != getFootAcceleration_B().end(); ++it) {
		std::string name = it->first;
		acc_W[name] = getFootAcceleration_W(it);
	}

	return acc_W;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_B(MotionIterator it) const
{
	return it->second;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_B(const std::string& name) const
{
	MotionIterator it = getFootAcceleration_B().find(name);
	if (it == foot_acc.end())
		return null_motion_;

	return getFootAcceleration_B(it);
}


const dwl::MotionMap& ReducedBodyState::getFootAcceleration_B() const
{
	return foot_acc;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_H(MotionIterator it)
{
	const se3::SE3& w_X_h = getCoMSE3_H().data;
	const se3::Motion& v = getFootVelocity_W(it->first).data;
	motion_.data = it->second.data.se3Action(w_X_h);
	motion_.data.linear() += v.angular().cross(v.linear());
	return motion_;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_H(const std::string& name)
{
	MotionIterator it = getFootAcceleration_B().find(name);
	if (it == foot_acc.end())
		return null_motion_;

	return getFootAcceleration_H(it);
}


dwl::MotionMap ReducedBodyState::getFootAcceleration_H()
{
	dwl::MotionMap acc_H;
	for (MotionIterator it = getFootAcceleration_B().begin();
			it != getFootAcceleration_B().end(); ++it) {
		std::string name = it->first;
		acc_H[name] = getFootAcceleration_H(it);
	}

	return acc_H;
}


const dwl::SE3Map& ReducedBodyState::getSupportRegion() const
{
	return support_region;
}


void ReducedBodyState::setTime(const double& _time)
{
	time = _time;
}


void ReducedBodyState::setCoMSE3(const dwl::SE3& pos_W)
{
	com_pos = pos_W;
}


void ReducedBodyState::setCoMVelocity_W(const dwl::Motion& vel_W)
{
	com_vel = vel_W;
}


void ReducedBodyState::setCoMVelocity_B(const dwl::Motion& vel_B)
{
	// Mapping the CoM velocity to the inertial frame
	se3::SE3 w_R_b(com_pos.getRotation(),
				   Eigen::Vector3d::Zero());
	com_vel.data = vel_B.data.se3Action(w_R_b);
}


void ReducedBodyState::setCoMVelocity_H(const dwl::Motion& vel_H)
{
	// Mapping the base velocity to the inertial frame
	se3::SE3 w_R_b(getCoMSE3_H().getRotation(),
				   Eigen::Vector3d::Zero());
	com_vel.data = vel_H.data.se3Action(w_R_b);
}


void ReducedBodyState::setRPYVelocity_W(const Eigen::Vector3d& rpy_rate)
{
	// Mapping the RPY velocity to angular one
	com_vel.setAngular(
			math::getInverseEulerAnglesRatesMatrix(com_pos.getRotation()) * rpy_rate);
}


void ReducedBodyState::setCoMAcceleration_W(const dwl::Motion& acc_W)
{
	com_acc = acc_W;
}


void ReducedBodyState::setCoMAcceleration_B(const dwl::Motion& acc_B)
{
	// Mapping the base acceleration to the inertial frame
	se3::SE3 w_R_b(com_pos.getRotation(),
				   Eigen::Vector3d::Zero());
	com_acc.data = acc_B.data.se3Action(w_R_b);
}


void ReducedBodyState::setCoMAcceleration_H(const dwl::Motion& acc_H)
{
	// Mapping the base acceleration to the inertial frame
	se3::SE3 w_R_b(getCoMSE3_H().getRotation(),
				   Eigen::Vector3d::Zero());
	com_acc.data = acc_H.data.se3Action(w_R_b);
}


void ReducedBodyState::setRPYAcceleration_W(const Eigen::Vector3d& rpy_acc)
{
	// omega_dot = EAR * rpy_ddot + EAR_dot * rpy_dot
	Eigen::Vector3d rpy = com_pos.getRPY();
	Eigen::Vector3d rpy_vel = getRPYVelocity_W();
	com_acc.setAngular(
			math::getInverseEulerAnglesRatesMatrix(rpy) * rpy_acc +
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_vel) * rpy_vel);
}


void ReducedBodyState::setCoPPosition_W(const Eigen::Vector3d& cop_W)
{
	cop = cop_W;
}


void ReducedBodyState::setFootSE3_W(SE3Iterator it)
{
	setFootSE3_W(it->first, it->second);
}


void ReducedBodyState::setFootSE3_W(const std::string& name,
										 const dwl::SE3& pos_W)
{
	// Mapping the contact SE3 from the inertial to base frame
	foot_pos[name].data = com_pos.data.actInv(pos_W.data);
}


void ReducedBodyState::setFootSE3_W(const dwl::SE3Map& pos_W)
{
	for (SE3Iterator it = pos_W.begin();
			it != pos_W.end(); ++it)
		setFootSE3_W(it);
}


void ReducedBodyState::setFootSE3_B(SE3Iterator it)
{
	setFootSE3_B(it->first, it->second);
}


void ReducedBodyState::setFootSE3_B(const std::string& name,
									const dwl::SE3& pos_B)
{
	foot_pos[name] = pos_B;
}


void ReducedBodyState::setFootSE3_B(const dwl::SE3Map& pos_B)
{
	foot_pos = pos_B;
}


void ReducedBodyState::setFootSE3_H(SE3Iterator it)
{
	setFootSE3_H(it->first, it->second);
}


void ReducedBodyState::setFootSE3_H(const std::string& name,
									const dwl::SE3& pos_H)
{
	// Mapping the contact SE3 from the inertial to horizontal frame
	foot_pos[name].data = getCoMSE3_H().data.actInv(pos_H.data);
}


void ReducedBodyState::setFootSE3_H(const dwl::SE3Map& pos_H)
{
	for (SE3Iterator foot_it = pos_H.begin();
			foot_it != pos_H.end(); foot_it++)
		setFootSE3_H(foot_it);
}


void ReducedBodyState::setFootVelocity_W(MotionIterator it)
{
	setFootVelocity_W(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_W(const std::string& name,
										 const dwl::Motion& vel_W)
{
	const se3::SE3& w_X_b = com_pos.data;
	foot_vel[name].data =
			(vel_W.data - com_vel.data).se3ActionInverse(w_X_b);
}


void ReducedBodyState::setFootVelocity_W(const dwl::MotionMap& vel_W)
{
	for (MotionIterator foot_it = vel_W.begin();
			foot_it != vel_W.end(); foot_it++)
		setFootVelocity_W(foot_it);
}


void ReducedBodyState::setFootVelocity_B(MotionIterator it)
{
	setFootVelocity_B(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_B(const std::string& name,
										 const dwl::Motion& vel_B)
{
	foot_vel[name] = vel_B;
}


void ReducedBodyState::setFootVelocity_B(const dwl::MotionMap& vel_B)
{
	foot_vel = vel_B;
}


void ReducedBodyState::setFootVelocity_H(MotionIterator it)
{
	setFootVelocity_H(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_H(const std::string& name,
										 const dwl::Motion& vel_H)
{
	const se3::SE3& w_X_h = getCoMSE3_H().data;
	foot_vel[name].data = vel_H.data.se3ActionInverse(w_X_h);
}


void ReducedBodyState::setFootVelocity_H(const dwl::MotionMap& vel_H)
{
	for (MotionIterator it = vel_H.begin();
			it != vel_H.end(); ++it)
		setFootVelocity_H(it);
}


void ReducedBodyState::setFootAcceleration_W(MotionIterator it)
{
	setFootAcceleration_W(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_W(const std::string& name,
											 const dwl::Motion& acc_W)
{
	const se3::SE3& w_X_b = com_pos.data;
	foot_acc[name].data =
			(acc_W.data - com_acc.data).se3ActionInverse(w_X_b);
}


void ReducedBodyState::setFootAcceleration_W(const dwl::MotionMap& acc_W)
{
	for (MotionIterator acc_it = acc_W.begin();
			acc_it != acc_W.end(); acc_it++) {
		setFootAcceleration_W(acc_it);
	}
}


void ReducedBodyState::setFootAcceleration_B(MotionIterator it)
{
	setFootAcceleration_B(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_B(const std::string& name,
											 const dwl::Motion& acc_B)
{
	foot_acc[name] = acc_B;
}


void ReducedBodyState::setFootAcceleration_B(const dwl::MotionMap& acc_B)
{
	foot_acc = acc_B;
}


void ReducedBodyState::setFootAcceleration_H(MotionIterator it)
{
	setFootAcceleration_H(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_H(const std::string& name,
											 const dwl::Motion& acc_H)
{
	const se3::SE3& w_X_h = getCoMSE3_H().data;
	foot_acc[name].data = acc_H.data.se3ActionInverse(w_X_h);
}


void ReducedBodyState::setFootAcceleration_H(const dwl::MotionMap& acc_H)
{
	for (MotionIterator acc_it = acc_H.begin();
			acc_it != acc_H.end(); acc_it++) {
		setFootAcceleration_H(acc_it);
	}
}

void ReducedBodyState::setSupportRegion(const dwl::SE3Map& support)
{
	support_region = support;
}

} //@namespace dwl
