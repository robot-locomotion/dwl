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


const dwl::SE3& ReducedBodyState::getCoMSE3_H() const
{
	return com_pos_H;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_W() const
{
	return com_vel;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_B() const
{
	return com_vel_B;
}


const dwl::Motion& ReducedBodyState::getCoMVelocity_H() const
{
	return com_vel_H;
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


const dwl::Motion& ReducedBodyState::getCoMAcceleration_B() const
{
	return com_acc_B;
}


const dwl::Motion& ReducedBodyState::getCoMAcceleration_H() const
{
	return com_acc_H;
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


const dwl::SE3& ReducedBodyState::getFootSE3_W(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getFootSE3_W().find(name);
	if (it == getFootSE3_W().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& ReducedBodyState::getFootSE3_W() const
{
	return foot_pos_W;
}


const dwl::SE3& ReducedBodyState::getFootSE3_B(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getFootSE3_B().find(name);
	if (it == getFootSE3_B().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& ReducedBodyState::getFootSE3_B() const
{
	return foot_pos;
}


const dwl::SE3& ReducedBodyState::getFootSE3_H(const std::string& name) const
{
	SE3Iterator it = getFootSE3_H().find(name);
	if (it == getFootSE3_H().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& ReducedBodyState::getFootSE3_H() const
{
	return foot_pos_H;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_W(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootVelocity_W().find(name);
	if (it == getFootVelocity_W().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootVelocity_W() const
{
	return foot_vel_W;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_B(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootVelocity_B().find(name);
	if (it == getFootVelocity_B().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootVelocity_B() const
{
	return foot_vel;
}


const dwl::Motion& ReducedBodyState::getFootVelocity_H(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootVelocity_H().find(name);
	if (it == getFootVelocity_H().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootVelocity_H() const
{
	return foot_vel_H;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_W(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootAcceleration_W().find(name);
	if (it == getFootAcceleration_W().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootAcceleration_W() const
{
	return foot_acc_W;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_B(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootAcceleration_B().find(name);
	if (it == getFootAcceleration_B().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootAcceleration_B() const
{
	return foot_acc;
}


const dwl::Motion& ReducedBodyState::getFootAcceleration_H(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getFootAcceleration_H().find(name);
	if (it == getFootAcceleration_H().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& ReducedBodyState::getFootAcceleration_H() const
{
	return foot_acc_H;
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

	// Computing and setting up the CoM SE3 in the horizontal frame
	Eigen::Vector3d rpy = com_pos.getRPY();
	rpy(rbd::X) *= 0.;
	rpy(rbd::Y) *= 0.;
	com_pos_H.setTranslation(com_pos.getTranslation());
	com_pos_H.setRotation(math::getRotationMatrix(rpy));

	// Updating the rotation transformation between frames
	w_R_b.rotation() = com_pos.getRotation();
	w_R_b.translation() = Eigen::Vector3d::Zero();
	w_R_h.rotation() = com_pos_H.getRotation();
	w_R_h.translation() = Eigen::Vector3d::Zero();
}


void ReducedBodyState::setCoMVelocity_W(const dwl::Motion& vel_W)
{
	com_vel = vel_W;
	com_vel_B.data = com_vel.data.se3ActionInverse(w_R_b);
	com_vel_H.data = com_vel.data.se3ActionInverse(w_R_h);
}


void ReducedBodyState::setCoMVelocity_B(const dwl::Motion& vel_B)
{
	com_vel.data = vel_B.data.se3Action(w_R_b);
	com_vel_B = vel_B;
	com_vel_H.data = com_vel.data.se3ActionInverse(w_R_h);
}


void ReducedBodyState::setCoMVelocity_H(const dwl::Motion& vel_H)
{
	com_vel.data = vel_H.data.se3Action(w_R_h);
	com_vel_B.data = vel_H.data.se3Action(w_R_b);
	com_vel_H = vel_H;
}


void ReducedBodyState::setRPYVelocity_W(const Eigen::Vector3d& rpy_rate)
{
	// Mapping the RPY velocity to angular one
	vec3_ = math::getInverseEulerAnglesRatesMatrix(com_pos.getRotation()) * rpy_rate;
	setCoMVelocity_W(dwl::Motion(getCoMVelocity_W().getLinear(), vec3_));
}


void ReducedBodyState::setCoMAcceleration_W(const dwl::Motion& acc_W)
{
	com_acc = acc_W;
	com_acc_B.data = com_acc.data.se3ActionInverse(w_R_b);
	com_acc_H.data = com_acc.data.se3ActionInverse(w_R_h);
}


void ReducedBodyState::setCoMAcceleration_B(const dwl::Motion& acc_B)
{
	com_acc.data = acc_B.data.se3Action(w_R_b);
	com_acc_B = acc_B;
	com_acc_H.data = com_acc.data.se3ActionInverse(w_R_h);
}


void ReducedBodyState::setCoMAcceleration_H(const dwl::Motion& acc_H)
{
	com_acc.data = acc_H.data.se3Action(w_R_h);
	com_acc_B.data = acc_H.data.se3Action(w_R_b);
	com_acc_H = acc_H;
}


void ReducedBodyState::setRPYAcceleration_W(const Eigen::Vector3d& rpy_acc)
{
	// omega_dot = EAR * rpy_ddot + EAR_dot * rpy_dot
	Eigen::Vector3d rpy = com_pos.getRPY();
	Eigen::Vector3d rpy_vel = getRPYVelocity_W();
	vec3_ = math::getInverseEulerAnglesRatesMatrix(rpy) * rpy_acc +
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_vel) * rpy_vel;
	setCoMAcceleration_W(dwl::Motion(getCoMAcceleration_W().getLinear(), vec3_));
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
	const se3::SE3& pos_B = getCoMSE3().data.actInv(pos_W.data);
	foot_pos_W[name] = pos_W;
	foot_pos[name].data = pos_B;
	foot_pos_H[name].data = getCoMSE3_H().data.act(pos_B);
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
	foot_pos_W[name].data = getCoMSE3().data.act(pos_B.data);
	foot_pos[name] = pos_B;
	foot_pos_H[name].data = getCoMSE3_H().data.act(pos_B.data);
}


void ReducedBodyState::setFootSE3_B(const dwl::SE3Map& pos_B)
{
	for (SE3Iterator it = pos_B.begin();
			it != pos_B.end(); ++it)
		setFootSE3_B(it);
}


void ReducedBodyState::setFootSE3_H(SE3Iterator it)
{
	setFootSE3_H(it->first, it->second);
}


void ReducedBodyState::setFootSE3_H(const std::string& name,
									const dwl::SE3& pos_H)
{
	const se3::SE3& pos_B = getCoMSE3_H().data.actInv(pos_H.data);
	foot_pos_W[name].data = getCoMSE3().data.act(pos_B);
	foot_pos[name].data = pos_B;
	foot_pos_H[name] = pos_H;
}


void ReducedBodyState::setFootSE3_H(const dwl::SE3Map& pos_H)
{
	for (SE3Iterator it = pos_H.begin();
			it != pos_H.end(); ++it)
		setFootSE3_H(it);
}


void ReducedBodyState::setFootVelocity_W(MotionIterator it)
{
	setFootVelocity_W(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_W(const std::string& name,
										 const dwl::Motion& vel_W)
{
	const se3::Motion& vel_B =
			(vel_W.data - com_vel.data).se3ActionInverse(getCoMSE3().data);
	foot_vel_W[name] = vel_W;
	foot_vel[name].data = vel_B;
	foot_vel_H[name].data = vel_B.se3Action(getCoMSE3_H().data);
}


void ReducedBodyState::setFootVelocity_W(const dwl::MotionMap& vel_W)
{
	for (MotionIterator it = vel_W.begin();
			it != vel_W.end(); ++it)
		setFootVelocity_W(it);
}


void ReducedBodyState::setFootVelocity_B(MotionIterator it)
{
	setFootVelocity_B(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_B(const std::string& name,
										 const dwl::Motion& vel_B)
{
	foot_vel[name].data = com_vel.data + vel_B.data.se3Action(getCoMSE3().data);
	foot_vel[name] = vel_B;
	foot_vel_H[name].data = vel_B.data.se3Action(getCoMSE3_H().data);
}


void ReducedBodyState::setFootVelocity_B(const dwl::MotionMap& vel_B)
{
	for (MotionIterator it = vel_B.begin();
			it != vel_B.end(); ++it)
		setFootVelocity_B(it);
}


void ReducedBodyState::setFootVelocity_H(MotionIterator it)
{
	setFootVelocity_H(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_H(const std::string& name,
										 const dwl::Motion& vel_H)
{
	const se3::Motion& vel_B = vel_H.data.se3ActionInverse(getCoMSE3_H().data);
	foot_vel_W[name].data =
			com_vel.data + vel_B.se3Action(getCoMSE3().data);
	foot_vel[name].data = vel_B;
	foot_vel_H[name] = vel_H;
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
	const se3::Motion& acc_B =
			(acc_W.data - com_acc.data).se3ActionInverse(getCoMSE3().data);
	foot_vel_W[name] = acc_W;
	foot_vel[name].data = acc_B;
	foot_vel_H[name].data = acc_B.se3Action(getCoMSE3_H().data);
}


void ReducedBodyState::setFootAcceleration_W(const dwl::MotionMap& acc_W)
{
	for (MotionIterator it = acc_W.begin();
			it != acc_W.end(); ++it) {
		setFootAcceleration_W(it);
	}
}


void ReducedBodyState::setFootAcceleration_B(MotionIterator it)
{
	setFootAcceleration_B(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_B(const std::string& name,
											 const dwl::Motion& acc_B)
{
	foot_acc[name].data = com_acc.data + acc_B.data.se3Action(getCoMSE3().data);
	foot_acc[name] = acc_B;
	foot_acc_H[name].data = acc_B.data.se3Action(getCoMSE3_H().data);
}


void ReducedBodyState::setFootAcceleration_B(const dwl::MotionMap& acc_B)
{
	for (MotionIterator it = acc_B.begin();
			it != acc_B.end(); ++it) {
		setFootAcceleration_B(it);
	}
}


void ReducedBodyState::setFootAcceleration_H(MotionIterator it)
{
	setFootAcceleration_H(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_H(const std::string& name,
											 const dwl::Motion& acc_H)
{
	const se3::Motion& acc_B = acc_H.data.se3ActionInverse(getCoMSE3_H().data);
	foot_acc_W[name].data =
			com_acc.data + acc_B.se3Action(getCoMSE3().data);
	foot_acc[name].data = acc_B;
	foot_acc_H[name] = acc_H;
}


void ReducedBodyState::setFootAcceleration_H(const dwl::MotionMap& acc_H)
{
	for (MotionIterator it = acc_H.begin();
			it != acc_H.end(); ++it) {
		setFootAcceleration_H(it);
	}
}

void ReducedBodyState::setSupportRegion(const dwl::SE3Map& support)
{
	support_region = support;
}

void ReducedBodyState::setSupportRegion(const std::string& name,
										const dwl::SE3& point)
{
	support_region[name] = point;
}

} //@namespace dwl
