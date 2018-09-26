#include <dwl/WholeBodyState.h>


namespace dwl
{

WholeBodyState::WholeBodyState(unsigned int num_joints) :
		time(0.), duration(0.), num_joints_(num_joints), default_joint_value_(0.)
{
	if (num_joints_ != 0) {
		joint_pos.setZero(num_joints_);
		joint_vel.setZero(num_joints_);
		joint_acc.setZero(num_joints_);
		joint_eff.setZero(num_joints_);
	}
}


WholeBodyState::~WholeBodyState()
{

}


const double& WholeBodyState::getTime() const
{
	return time;
}


const dwl::SE3& WholeBodyState::getBaseSE3() const
{
	return base_pos;
}


const dwl::SE3& WholeBodyState::getBaseSE3_H() const
{
	return base_pos_H;
}


const dwl::Motion& WholeBodyState::getBaseVelocity_W() const
{
	return base_vel;
}


const dwl::Motion& WholeBodyState::getBaseVelocity_B() const
{
	return base_vel_B;
}


const dwl::Motion& WholeBodyState::getBaseVelocity_H() const
{
	return base_vel_H;
}


const Eigen::Vector3d& WholeBodyState::getBaseRPYVelocity_W()
{
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(base_pos.getRotation()).transpose();
	vec3_ = EAR * base_vel.getAngular();
	return vec3_;
}


const dwl::Motion& WholeBodyState::getBaseAcceleration_W() const
{
	return base_acc;
}


const dwl::Motion& WholeBodyState::getBaseAcceleration_B() const
{
	return base_acc_B;
}

const dwl::Motion& WholeBodyState::getBaseAcceleration_H() const
{
	return base_acc_H;
}


const Eigen::Vector3d& WholeBodyState::getBaseRPYAcceleration_W()
{
	// rpy_ddot = EAR^-1 * (omega_dot - EAR_dot * rpy_dot)
	Eigen::Vector3d rpy = base_pos.getRPY();
	Eigen::Vector3d rpy_d = getBaseRPYVelocity_W();
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(rpy).inverse();
	Eigen::Matrix3d EARinv_dot =
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_d);
	vec3_ = EAR * (base_vel.getAngular() - EARinv_dot * rpy_d);
	return vec3_;
}

const dwl::Force& WholeBodyState::getBaseWrench_W() const
{
	return base_eff;
}

const Eigen::VectorXd& WholeBodyState::getJointPosition() const
{
	return joint_pos;
}


const double& WholeBodyState::getJointPosition(const unsigned int& index) const
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint position.\n"
				COLOR_RESET);
		return default_joint_value_;
	}

	return joint_pos(index);
}


const Eigen::VectorXd& WholeBodyState::getJointVelocity() const
{
	return joint_vel;
}


const double& WholeBodyState::getJointVelocity(const unsigned int& index) const
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint velocity.\n"
				COLOR_RESET);
		return default_joint_value_;
	}

	return joint_vel(index);
}


const Eigen::VectorXd& WholeBodyState::getJointAcceleration() const
{
	return joint_acc;
}


const double& WholeBodyState::getJointAcceleration(const unsigned int& index) const
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint acceleration.\n"
				COLOR_RESET);
		return default_joint_value_;
	}

	return joint_acc(index);
}


const Eigen::VectorXd& WholeBodyState::getJointEffort() const
{
	return joint_eff;
}


const double& WholeBodyState::getJointEffort(const unsigned int& index) const
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be get the joint effort.\n"
				COLOR_RESET);
		return default_joint_value_;
	}

	return joint_eff(index);
}


const unsigned int WholeBodyState::getJointDoF() const
{
	return num_joints_;
}


const dwl::SE3& WholeBodyState::getContactSE3_W(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getContactSE3_W().find(name);
	if (it == getContactSE3_W().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& WholeBodyState::getContactSE3_W() const
{
	return contact_pos_W;
}


const dwl::SE3& WholeBodyState::getContactSE3_B(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getContactSE3_B().find(name);
	if (it == getContactSE3_B().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& WholeBodyState::getContactSE3_B() const
{
	return contact_pos;
}


const dwl::SE3& WholeBodyState::getContactSE3_H(const std::string& name) const
{
	// Getting the contact iterator
	SE3Iterator it = getContactSE3_H().find(name);
	if (it == getContactSE3_H().end())
		return null_se3_;

	return it->second;
}


const dwl::SE3Map& WholeBodyState::getContactSE3_H() const
{
	return contact_pos_H;
}


const dwl::Motion& WholeBodyState::getContactVelocity_W(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getContactVelocity_W().find(name);
	if (it == getContactVelocity_W().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactVelocity_W() const
{
	return contact_vel_W;
}


const dwl::Motion& WholeBodyState::getContactVelocity_B(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getContactVelocity_B().find(name);
	if (it == getContactVelocity_B().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactVelocity_B() const
{
	return contact_vel;
}


const dwl::Motion& WholeBodyState::getContactVelocity_H(const std::string& name) const
{
	// Getting the contact iterator
	MotionIterator it = getContactVelocity_H().find(name);
	if (it == getContactVelocity_H().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactVelocity_H() const
{
	return contact_vel_H;
}


const dwl::Motion& WholeBodyState::getContactAcceleration_W(const std::string& name) const
{
	MotionIterator it = getContactAcceleration_W().find(name);
	if (it == getContactAcceleration_W().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactAcceleration_W() const
{
	return contact_acc_W;
}


const dwl::Motion& WholeBodyState::getContactAcceleration_B(const std::string& name) const
{
	MotionIterator it = getContactAcceleration_B().find(name);
	if (it == getContactAcceleration_B().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactAcceleration_B() const
{
	return contact_acc;
}


const dwl::Motion& WholeBodyState::getContactAcceleration_H(const std::string& name) const
{
	MotionIterator it = getContactAcceleration_H().find(name);
	if (it == getContactAcceleration_H().end())
		return null_motion_;

	return it->second;
}


const dwl::MotionMap& WholeBodyState::getContactAcceleration_H() const
{
	return contact_acc_H;
}


const dwl::ForceMap& WholeBodyState::getContactWrench_B() const
{
	return contact_eff;
}


const dwl::Force& WholeBodyState::getContactWrench_B(const std::string& name) const
{
	ForceIterator it = contact_eff.find(name);
	if (it == contact_eff.end())
		return null_force_;
	else
		return it->second;
}


bool WholeBodyState::getContactCondition(const std::string& name,
										 const double& force_threshold) const
{
	// Returns inactive in case that the contact wrench is not defined
	ForceIterator it = contact_eff.find(name);
	if (it == contact_eff.end())
		return false;

	if (it->second.data.toVector().norm() > force_threshold)
		return true;
	else
		return false;
}


void WholeBodyState::setTime(const double& _time)
{
	time = _time;
}


void WholeBodyState::setBaseSE3(const dwl::SE3& pos)
{
	base_pos = pos;

	// Computing and setting up the CoM SE3 in the horizontal frame
	Eigen::Vector3d rpy = base_pos.getRPY();
	rpy(rbd::X) *= 0.;
	rpy(rbd::Y) *= 0.;
	base_pos_H.setTranslation(base_pos.getTranslation());
	base_pos_H.setRotation(math::getRotationMatrix(rpy));

	// Updating the rotation transformation between frames
	w_R_b.rotation() = base_pos.getRotation();
	w_R_b.translation() = Eigen::Vector3d::Zero();
	w_R_h.rotation() = base_pos_H.getRotation();
	w_R_h.translation() = Eigen::Vector3d::Zero();
}


void WholeBodyState::setBaseVelocity_W(const dwl::Motion& vel_W)
{
	base_vel = vel_W;
	base_vel_B.data = base_vel.data.se3ActionInverse(w_R_b);
	base_vel_H.data = base_vel.data.se3ActionInverse(w_R_h);
}


void WholeBodyState::setBaseVelocity_B(const dwl::Motion& vel_B)
{
	base_vel.data = vel_B.data.se3Action(w_R_b);
	base_vel_B = vel_B;
	base_vel_H.data = base_vel.data.se3ActionInverse(w_R_h);
}


void WholeBodyState::setBaseVelocity_H(const dwl::Motion& vel_H)
{
	base_vel.data = vel_H.data.se3Action(w_R_h);
	base_vel_B.data = vel_H.data.se3Action(w_R_b);
	base_vel_H = vel_H;
}

void WholeBodyState::setBaseRPYVelocity_W(const Eigen::Vector3d& rpy_rate)
{
	// Mapping the RPY velocity to angular one
	vec3_ = math::getInverseEulerAnglesRatesMatrix(base_pos.getRotation()) * rpy_rate;
	setBaseVelocity_W(dwl::Motion(getBaseVelocity_W().getLinear(), vec3_));
}


void WholeBodyState::setBaseAcceleration_W(const dwl::Motion& acc_W)
{
	base_acc = acc_W;
	base_acc_B.data = base_acc.data.se3ActionInverse(w_R_b);
	base_acc_H.data = base_acc.data.se3ActionInverse(w_R_h);
}


void WholeBodyState::setBaseAcceleration_B(const dwl::Motion& acc_B)
{
	base_acc.data = acc_B.data.se3Action(w_R_b);
	base_acc_B = acc_B;
	base_acc_H.data = base_acc.data.se3ActionInverse(w_R_h);
}


void WholeBodyState::setBaseAcceleration_H(const dwl::Motion& acc_H)
{
	base_acc.data = acc_H.data.se3Action(w_R_h);
	base_acc_B.data = acc_H.data.se3Action(w_R_b);
	base_acc_H = acc_H;
}


void WholeBodyState::setBaseRPYAcceleration_W(const Eigen::Vector3d& rpy_acc)
{
	// omega_dot = EAR * rpy_ddot + EAR_dot * rpy_dot
	Eigen::Vector3d rpy = base_pos.getRPY();
	Eigen::Vector3d rpy_vel = getBaseRPYVelocity_W();
	vec3_ = math::getInverseEulerAnglesRatesMatrix(rpy) * rpy_acc +
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_vel) * rpy_vel;
	setBaseAcceleration_W(dwl::Motion(getBaseAcceleration_W().getLinear(), vec3_));
}


void WholeBodyState::setJointPosition(const Eigen::VectorXd& pos)
{
	joint_pos = pos;
}


void WholeBodyState::setJointPosition(const double& pos,
									  const unsigned int& index)
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint position.\n"
				COLOR_RESET);
		return;
	}

	joint_pos(index) = pos;
}


void WholeBodyState::setJointVelocity(const Eigen::VectorXd& vel)
{
	joint_vel = vel;
}


void WholeBodyState::setJointVelocity(const double& vel,
									  const unsigned int& index)
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint velocity.\n"
				COLOR_RESET);
		return;
	}

	joint_vel(index) = vel;
}


void WholeBodyState::setJointAcceleration(const Eigen::VectorXd& acc)
{
	joint_acc = acc;
}


void WholeBodyState::setJointAcceleration(const double& acc,
										  const unsigned int& index)
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint acceleration.\n"
				COLOR_RESET);
		return;
	}

	joint_acc(index) = acc;
}


void WholeBodyState::setJointEffort(const double& eff,
									const unsigned int& index)
{
	if (index >= num_joints_) {
		printf(YELLOW "Warning: the index is bigger than the number of joints."
				" It cannot be set the joint effort.\n"
				COLOR_RESET);
		return;
	}

	joint_eff(index) = eff;
}


void WholeBodyState::setJointEffort(const Eigen::VectorXd& eff)
{
	joint_eff = eff;
}


void WholeBodyState::setJointDoF(const unsigned int& num_joints)
{
	num_joints_ = num_joints;
	joint_pos.setZero(num_joints_);
	joint_vel.setZero(num_joints_);
	joint_acc.setZero(num_joints_);
	joint_eff.setZero(num_joints_);
}


void WholeBodyState::setContactSE3_W(SE3Iterator it)
{
	setContactSE3_W(it->first, it->second);
}


void WholeBodyState::setContactSE3_W(const std::string& name,
									 const dwl::SE3& pos_W)
{
	const se3::SE3& pos_B = getBaseSE3().data.actInv(pos_W.data);
	contact_pos_W[name] = pos_W;
	contact_pos[name].data = pos_B;
	contact_pos_H[name].data = getBaseSE3_H().data.act(pos_B);
}


void WholeBodyState::setContactSE3_W(const dwl::SE3Map& pos_W)
{
	for (SE3Iterator it = pos_W.begin();
			it != pos_W.end(); ++it)
		setContactSE3_W(it);
}


void WholeBodyState::setContactSE3_B(SE3Iterator it)
{
	setContactSE3_B(it->first, it->second);
}


void WholeBodyState::setContactSE3_B(const std::string& name,
									 const dwl::SE3& pos_B)
{
	contact_pos_W[name].data = getBaseSE3().data.act(pos_B.data);
	contact_pos[name] = pos_B;
	contact_pos_H[name].data = getBaseSE3_H().data.act(pos_B.data);
}


void WholeBodyState::setContactSE3_B(const dwl::SE3Map& pos_B)
{
	for (SE3Iterator it = pos_B.begin();
			it != pos_B.end(); ++it)
		setContactSE3_B(it);
}


void WholeBodyState::setContactSE3_H(SE3Iterator it)
{
	setContactSE3_H(it->first, it->second);
}


void WholeBodyState::setContactSE3_H(const std::string& name,
									 const dwl::SE3& pos_H)
{
	const se3::SE3& pos_B = getBaseSE3_H().data.actInv(pos_H.data);
	contact_pos_W[name].data = getBaseSE3().data.act(pos_B);
	contact_pos[name].data = pos_B;
	contact_pos_H[name] = pos_H;
}


void WholeBodyState::setContactSE3_H(const dwl::SE3Map& pos_H)
{
	for (SE3Iterator it = pos_H.begin();
			it != pos_H.end(); ++it)
		setContactSE3_H(it);
}


void WholeBodyState::setContactVelocity_W(MotionIterator it)
{
	setContactVelocity_W(it->first, it->second);
}


void WholeBodyState::setContactVelocity_W(const std::string& name,
										  const dwl::Motion& vel_W)
{
	const se3::Motion& vel_B =
			(vel_W.data - base_vel.data).se3ActionInverse(getBaseSE3().data);
	contact_vel_W[name] = vel_W;
	contact_vel[name].data = vel_B;
	contact_vel_H[name].data = vel_B.se3Action(getBaseSE3_H().data);
}


void WholeBodyState::setContactVelocity_W(const dwl::MotionMap& vel_W)
{
	for (MotionIterator it = vel_W.begin();
			it != vel_W.end(); ++it)
		setContactVelocity_W(it);
}


void WholeBodyState::setContactVelocity_B(MotionIterator it)
{
	setContactVelocity_B(it->first, it->second);
}


void WholeBodyState::setContactVelocity_B(const std::string& name,
										  const dwl::Motion& vel_B)
{
	contact_vel[name].data = base_vel.data + vel_B.data.se3Action(getBaseSE3().data);
	contact_vel[name] = vel_B;
	contact_vel_H[name].data = vel_B.data.se3Action(getBaseSE3_H().data);
}


void WholeBodyState::setContactVelocity_B(const dwl::MotionMap& vel_B)
{
	for (MotionIterator it = vel_B.begin();
			it != vel_B.end(); ++it)
		setContactVelocity_B(it);
}


void WholeBodyState::setContactVelocity_H(MotionIterator it)
{
	setContactVelocity_H(it->first, it->second);
}


void WholeBodyState::setContactVelocity_H(const std::string& name,
										  const dwl::Motion& vel_H)
{
	const se3::Motion& vel_B = vel_H.data.se3ActionInverse(getBaseSE3_H().data);
	contact_vel_W[name].data =
			base_vel.data + vel_B.se3Action(getBaseSE3().data);
	contact_vel[name].data = vel_B;
	contact_vel_H[name] = vel_H;
}


void WholeBodyState::setContactVelocity_H(const dwl::MotionMap& vel_H)
{
	for (MotionIterator it = vel_H.begin();
			it != vel_H.end(); ++it)
		setContactVelocity_H(it);
}


void WholeBodyState::setContactAcceleration_W(MotionIterator it)
{
	setContactAcceleration_W(it->first, it->second);
}


void WholeBodyState::setContactAcceleration_W(const std::string& name,
											  const dwl::Motion& acc_W)
{
	const se3::Motion& acc_B =
			(acc_W.data - base_acc.data).se3ActionInverse(getBaseSE3().data);
	contact_vel_W[name] = acc_W;
	contact_vel[name].data = acc_B;
	contact_vel_H[name].data = acc_B.se3Action(getBaseSE3_H().data);
}


void WholeBodyState::setContactAcceleration_W(const dwl::MotionMap& acc_W)
{
	for (MotionIterator it = acc_W.begin();
			it != acc_W.end(); ++it) {
		setContactAcceleration_W(it);
	}
}


void WholeBodyState::setContactAcceleration_B(MotionIterator it)
{
	setContactAcceleration_B(it->first, it->second);
}


void WholeBodyState::setContactAcceleration_B(const std::string& name,
											  const dwl::Motion& acc_B)
{
	contact_acc[name].data = base_acc.data + acc_B.data.se3Action(getBaseSE3().data);
	contact_acc[name] = acc_B;
	contact_acc_H[name].data = acc_B.data.se3Action(getBaseSE3_H().data);
}


void WholeBodyState::setContactAcceleration_B(const dwl::MotionMap& acc_B)
{
	for (MotionIterator it = acc_B.begin();
			it != acc_B.end(); ++it) {
		setContactAcceleration_B(it);
	}
}


void WholeBodyState::setContactAcceleration_H(MotionIterator it)
{
	setContactAcceleration_H(it->first, it->second);
}


void WholeBodyState::setContactAcceleration_H(const std::string& name,
											  const dwl::Motion& acc_H)
{
	const se3::Motion& acc_B = acc_H.data.se3ActionInverse(getBaseSE3_H().data);
	contact_acc_W[name].data =
			base_acc.data + acc_B.se3Action(getBaseSE3().data);
	contact_acc[name].data = acc_B;
	contact_acc_H[name] = acc_H;
}


void WholeBodyState::setContactAcceleration_H(const dwl::MotionMap& acc_H)
{
	for (MotionIterator it = acc_H.begin();
			it != acc_H.end(); ++it) {
		setContactAcceleration_H(it);
	}
}


void WholeBodyState::setContactWrench_B(const dwl::ForceMap& eff)
{
	contact_eff = eff;
}


void WholeBodyState::setContactWrench_B(const std::string& name,
									    const dwl::Force& eff)
{
	contact_eff[name] = eff;
}


void WholeBodyState::setContactCondition(const std::string& name,
										 const bool& condition)
{
	if (condition) {
		contact_eff[name].setLinear(ACTIVE_CONTACT);
		contact_eff[name].setAngular(ACTIVE_CONTACT);
	} else {
		contact_eff[name].setLinear(INACTIVE_CONTACT);
		contact_eff[name].setAngular(INACTIVE_CONTACT);
	}
}

} //@namespace dwl
