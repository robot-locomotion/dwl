#include <dwl/WholeBodyState.h>


namespace dwl
{

WholeBodyState::WholeBodyState(unsigned int num_joints) :
		time(0.), duration(0.), num_joints_(num_joints)
{
	base_pos.setZero();
	base_vel.setZero();
	base_acc.setZero();
	base_eff.setZero();

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


Eigen::Vector3d WholeBodyState::getBasePosition_W() const
{
	return base_pos.segment<3>(rbd::LX);
}


Eigen::Quaterniond WholeBodyState::getBaseOrientation_W() const
{
	return math::getQuaternion(getBaseRPY_W());
}


Eigen::Vector3d WholeBodyState::getBaseRPY_W() const
{
	return base_pos.segment<3>(rbd::AX);
}


Eigen::Quaterniond WholeBodyState::getBaseOrientation_H() const
{
	return Eigen::Quaterniond(math::getRotationMatrix(getBaseRPY_H()));
}


Eigen::Vector3d WholeBodyState::getBaseRPY_H() const
{
	return Eigen::Vector3d(0., 0., base_pos(rbd::AZ));
}


Eigen::Vector3d WholeBodyState::getBaseVelocity_W() const
{
	return base_vel.segment<3>(rbd::LX);
}


Eigen::Vector3d WholeBodyState::getBaseVelocity_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getBaseVelocity_W(),
										  getBaseOrientation_W());
}


Eigen::Vector3d WholeBodyState::getBaseVelocity_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getBaseVelocity_W(),
												getBaseRPY_W());
}


Eigen::Vector3d WholeBodyState::getBaseAngularVelocity_W() const
{
	return base_vel.segment<3>(rbd::AX);
}


Eigen::Vector3d WholeBodyState::getBaseAngularVelocity_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getBaseAngularVelocity_W(),
										  getBaseOrientation_W());
}


Eigen::Vector3d WholeBodyState::getBaseAngularVelocity_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getBaseAngularVelocity_W(),
												getBaseRPY_W());
}


Eigen::Vector3d WholeBodyState::getBaseRPYVelocity() const
{
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(getBaseRPY_W()).inverse();
	return EAR * getBaseAngularVelocity_W();
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_W() const
{
	return base_acc.segment<3>(rbd::LX);
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getBaseAcceleration_W(),
										  getBaseOrientation_W());
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getBaseAcceleration_W(),
												getBaseRPY_W());
}


Eigen::Vector3d WholeBodyState::getBaseAngularAcceleration_W() const
{
	return base_acc.segment<3>(rbd::AX);
}


Eigen::Vector3d WholeBodyState::getBaseAngularAcceleration_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getBaseAngularAcceleration_W(),
										  getBaseOrientation_W());
}


Eigen::Vector3d WholeBodyState::getBaseAngularAcceleration_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getBaseAngularAcceleration_W(),
												getBaseRPY_W());
}


Eigen::Vector3d WholeBodyState::getBaseRPYAcceleration() const
{
	// rpy_ddot = EAR^-1 * (omega_dot - EAR_dot * rpy_dot)
	Eigen::Vector3d rpy = getBaseRPY_W();
	Eigen::Vector3d rpy_d = getBaseRPYVelocity();
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(rpy).inverse();
	Eigen::Matrix3d EARinv_dot =
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_d);
	return EAR * (getBaseAngularAcceleration_W() - EARinv_dot * rpy_d);
}


void WholeBodyState::setBasePosition_W(const Eigen::Vector3d& pos)
{
	rbd::linearPart(base_pos) = pos;
}


void WholeBodyState::setBaseOrientation_W(const Eigen::Quaterniond& orient)
{
	rbd::angularPart(base_pos) = math::getRPY(orient);
}


void WholeBodyState::setBaseRPY_W(const Eigen::Vector3d& rpy)
{
	rbd::angularPart(base_pos) = rpy;
}


void WholeBodyState::setBaseVelocity_W(const Eigen::Vector3d& vel_W)
{
	rbd::linearPart(base_vel) = vel_W;
}


void WholeBodyState::setBaseVelocity_B(const Eigen::Vector3d& vel_B)
{
	rbd::linearPart(base_vel) =
			frame_tf_.fromBaseToWorldFrame(vel_B, getBaseOrientation_W());
}


void WholeBodyState::setBaseVelocity_H(const Eigen::Vector3d& vel_H)
{
	rbd::linearPart(base_vel) =
			frame_tf_.fromHorizontalToWorldFrame(vel_H, getBaseRPY_W());
}


void WholeBodyState::setBaseAngularVelocity_W(const Eigen::Vector3d& rate_W)
{
	rbd::angularPart(base_vel) = rate_W;
}


void WholeBodyState::setBaseAngularVelocity_B(const Eigen::Vector3d& rate_B)
{
	rbd::angularPart(base_vel) =
			frame_tf_.fromBaseToWorldFrame(rate_B, getBaseOrientation_W());
}


void WholeBodyState::setBaseAngularVelocity_H(const Eigen::Vector3d& rate_H)
{
	rbd::angularPart(base_vel) =
			frame_tf_.fromHorizontalToWorldFrame(rate_H, getBaseRPY_W());
}


void WholeBodyState::setBaseRPYVelocity(const Eigen::Vector3d& rpy_rate)
{
	rbd::angularPart(base_vel) =
			math::getInverseEulerAnglesRatesMatrix(getBaseRPY_W()) * rpy_rate;
}


void WholeBodyState::setBaseAcceleration_W(const Eigen::Vector3d& acc_W)
{
	rbd::linearPart(base_acc) = acc_W;
}


void WholeBodyState::setBaseAcceleration_B(const Eigen::Vector3d& acc_B)
{
	rbd::linearPart(base_acc) =
			frame_tf_.fromBaseToWorldFrame(acc_B, getBaseOrientation_W());
}


void WholeBodyState::setBaseAcceleration_H(const Eigen::Vector3d& acc_H)
{
	rbd::linearPart(base_acc) =
			frame_tf_.fromHorizontalToWorldFrame(acc_H, getBaseRPY_W());
}


void WholeBodyState::setBaseAngularAcceleration_W(const Eigen::Vector3d& rotacc_W)
{
	rbd::angularPart(base_acc) = rotacc_W;
}


void WholeBodyState::setBaseAngularAcceleration_B(const Eigen::Vector3d& rotacc_B)
{
	rbd::angularPart(base_acc) =
			frame_tf_.fromBaseToWorldFrame(rotacc_B, getBaseOrientation_W());
}


void WholeBodyState::setBaseAngularAcceleration_H(const Eigen::Vector3d& rotacc_H)
{
	rbd::angularPart(base_acc) =
			frame_tf_.fromHorizontalToWorldFrame(rotacc_H, getBaseRPY_W());
}


void WholeBodyState::setBaseRPYAcceleration(const Eigen::Vector3d& rpy_rate,
											const Eigen::Vector3d& rpy_acc)
{
	// omega_dot = EAR * rpy_ddot + EAR_dot * rpy_dot
	Eigen::Vector3d rpy = getBaseRPY_W();
	rbd::angularPart(base_acc) =
			math::getInverseEulerAnglesRatesMatrix(rpy) * rpy_acc +
			math::getInverseEulerAnglesRatesMatrix_dot(rpy, rpy_rate) * rpy_rate;
}


void WholeBodyState::setJointDoF(const unsigned int& num_joints)
{
	num_joints_ = num_joints;
	joint_pos.setZero(num_joints_);
	joint_vel.setZero(num_joints_);
	joint_acc.setZero(num_joints_);
	joint_eff.setZero(num_joints_);
}


const Eigen::VectorXd& WholeBodyState::getJointPosition() const
{
	return joint_pos;
}


const double& WholeBodyState::getJointPosition(const unsigned int& index) const
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	return joint_pos(index);
}


const Eigen::VectorXd& WholeBodyState::getJointVelocity() const
{
	return joint_vel;
}


const double& WholeBodyState::getJointVelocity(const unsigned int& index) const
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	return joint_vel(index);
}


const Eigen::VectorXd& WholeBodyState::getJointAcceleration() const
{
	return joint_acc;
}


const double& WholeBodyState::getJointAcceleration(const unsigned int& index) const
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	return joint_acc(index);
}


const double& WholeBodyState::getJointEffort(const unsigned int& index) const
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	return joint_eff(index);
}


const Eigen::VectorXd& WholeBodyState::getJointEffort() const
{
	return joint_eff;
}


const unsigned int WholeBodyState::getJointDof() const
{
	return num_joints_;
}


void WholeBodyState::setJointPosition(const Eigen::VectorXd& pos)
{
	joint_pos = pos;
}


void WholeBodyState::setJointPosition(const double& pos,
									  const unsigned int& index)
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	joint_pos(index) = pos;
}


void WholeBodyState::setJointVelocity(const Eigen::VectorXd& vel)
{
	joint_vel = vel;
}


void WholeBodyState::setJointVelocity(const double& vel,
									  const unsigned int& index)
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	joint_vel(index) = vel;
}


void WholeBodyState::setJointAcceleration(const Eigen::VectorXd& acc)
{
	joint_acc = acc;
}


void WholeBodyState::setJointAcceleration(const double& acc,
										  const unsigned int& index)
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	joint_acc(index) = acc;
}


void WholeBodyState::setJointEffort(const double& eff,
									const unsigned int& index)
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	joint_eff(index) = eff;
}


void WholeBodyState::setJointEffort(const Eigen::VectorXd& eff)
{
	joint_eff = eff;
}


Eigen::VectorXd WholeBodyState::getContactPosition_W(ContactIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactPosition_W(const std::string& name) const
{
	ContactIterator contact_it = getContactPosition_B().find(name);
	return getContactPosition_W(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactPosition_W() const
{
	rbd::BodyVectorXd contact_pos_W;
	for (ContactIterator contact_it = getContactPosition_B().begin();
			contact_it != getContactPosition_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_pos_W[name] = getContactPosition_W(contact_it);
	}

	return contact_pos_W;
}


const Eigen::VectorXd& WholeBodyState::getContactPosition_B(ContactIterator it) const
{
	return it->second;
}


const Eigen::VectorXd& WholeBodyState::getContactPosition_B(const std::string& name) const
{
	ContactIterator contact_it = getContactPosition_B().find(name);
	return getContactPosition_B(contact_it);
}


const rbd::BodyVectorXd& WholeBodyState::getContactPosition_B() const
{
	return contact_pos;
}


Eigen::VectorXd WholeBodyState::getContactPosition_H(ContactIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactPosition_H(const std::string& name) const
{
	ContactIterator contact_it = getContactPosition_B().find(name);
	return getContactPosition_H(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactPosition_H() const
{
	rbd::BodyVectorXd contact_pos_H;
	for (ContactIterator contact_it = getContactPosition_B().begin();
			contact_it != getContactPosition_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_pos_H[name] = getContactPosition_H(contact_it);
	}

	return contact_pos_H;
}


Eigen::VectorXd WholeBodyState::getContactVelocity_W(ContactIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactVelocity_W(const std::string& name) const
{
	ContactIterator contact_it = getContactVelocity_B().find(name);
	return getContactVelocity_W(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactVelocity_W() const
{
	rbd::BodyVectorXd contact_vel_W;
	for (ContactIterator contact_it = getContactVelocity_B().begin();
			contact_it != getContactVelocity_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_vel_W[name] = getContactVelocity_W(contact_it);
	}

	return contact_vel_W;
}


const Eigen::VectorXd& WholeBodyState::getContactVelocity_B(ContactIterator it) const
{
	return it->second;
}


const Eigen::VectorXd& WholeBodyState::getContactVelocity_B(const std::string& name) const
{
	ContactIterator contact_it = contact_vel.find(name);
	return getContactVelocity_B(contact_it);
}


const rbd::BodyVectorXd& WholeBodyState::getContactVelocity_B() const
{
	return contact_vel;
}


Eigen::VectorXd WholeBodyState::getContactVelocity_H(ContactIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactVelocity_H(const std::string& name) const
{
	ContactIterator contact_it = getContactVelocity_B().find(name);
	return getContactVelocity_H(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactVelocity_H() const
{
	rbd::BodyVectorXd contact_vel_H;
	for (ContactIterator contact_it = getContactVelocity_B().begin();
			contact_it != getContactVelocity_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_vel_H[name] = getContactVelocity_H(contact_it);
	}

	return contact_vel_H;
}


Eigen::VectorXd WholeBodyState::getContactAcceleration_W(ContactIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactAcceleration_W(const std::string& name) const
{
	ContactIterator contact_it = getContactAcceleration_B().find(name);
	return getContactAcceleration_W(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactAcceleration_W() const
{
	rbd::BodyVectorXd contact_acc_W;
	for (ContactIterator contact_it = getContactAcceleration_B().begin();
			contact_it != getContactAcceleration_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_acc_W[name] = getContactAcceleration_W(contact_it);
	}

	return contact_acc_W;
}


const Eigen::VectorXd& WholeBodyState::getContactAcceleration_B(ContactIterator it) const
{
	return it->second;
}


const Eigen::VectorXd& WholeBodyState::getContactAcceleration_B(const std::string& name) const
{
	ContactIterator contact_it = getContactAcceleration_B().find(name);
	return getContactAcceleration_B(contact_it);
}


const rbd::BodyVectorXd& WholeBodyState::getContactAcceleration_B() const
{
	return contact_acc;
}


Eigen::VectorXd WholeBodyState::getContactAcceleration_H(ContactIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getBaseRPY_W());
}


Eigen::VectorXd WholeBodyState::getContactAcceleration_H(const std::string& name) const
{
	ContactIterator contact_it = getContactAcceleration_B().find(name);
	return getContactAcceleration_H(contact_it);
}


rbd::BodyVectorXd WholeBodyState::getContactAcceleration_H() const
{
	rbd::BodyVectorXd contact_acc_H;
	for (ContactIterator contact_it = getContactAcceleration_B().begin();
			contact_it != getContactAcceleration_B().end(); contact_it++) {
		std::string name = contact_it->first;
		contact_acc_H[name] = getContactAcceleration_H(contact_it);
	}

	return contact_acc_H;
}


const rbd::BodyVector6d& WholeBodyState::getContactWrench_B() const
{
	return contact_eff;
}


const rbd::Vector6d& WholeBodyState::getContactWrench_B(const std::string& name) const
{
	return contact_eff.find(name)->second;
}


bool WholeBodyState::getContactCondition(const std::string& name,
										 const double& force_threshold) const
{
	// Returns inactive in case that the contact wrench is not defined
	rbd::BodyVector6d::const_iterator it = contact_eff.find(name);
	if (it == contact_eff.end())
		return false;

	if (it->second.norm() > force_threshold)
		return true;
	else
		return false;
}


bool WholeBodyState::getContactCondition(const std::string& name) const
{
	// Returns inactive in case that the contact wrench is not defined
	rbd::BodyVector6d::const_iterator it = contact_eff.find(name);
	if (it == contact_eff.end())
		return false;

	if (it->second == ACTIVE_CONTACT)
		return true;
	else
		return false;
}


void WholeBodyState::setContactPosition_W(ContactIterator it)
{
	setContactPosition_W(it->first, it->second);
}


void WholeBodyState::setContactPosition_W(const std::string& name,
										  const Eigen::VectorXd& pos_W)
{
	contact_pos[name] =
			frame_tf_.fromWorldToBaseFrame(pos_W - getBasePosition_W(),
										   getBaseRPY_W());
}


void WholeBodyState::setContactPosition_W(const rbd::BodyVectorXd& pos_W)
{
	for (ContactIterator contact_it = pos_W.begin();
			contact_it != pos_W.end(); contact_it++)
		setContactPosition_W(contact_it);
}


void WholeBodyState::setContactPosition_B(ContactIterator it)
{
	setContactPosition_B(it->first, it->second);
}


void WholeBodyState::setContactPosition_B(const std::string& name,
										  const Eigen::VectorXd& pos_B)
{
	contact_pos[name] = pos_B;
}


void WholeBodyState::setContactPosition_B(const rbd::BodyVectorXd& pos_B)
{
	contact_pos = pos_B;
}


void WholeBodyState::setContactPosition_H(ContactIterator it)
{
	setContactPosition_H(it->first, it->second);
}


void WholeBodyState::setContactPosition_H(const std::string& name,
										  const Eigen::VectorXd& pos_H)
{
	contact_pos[name] =
			frame_tf_.fromHorizontalToBaseFrame(pos_H, getBaseRPY_W());
}


void WholeBodyState::setContactPosition_H(const rbd::BodyVectorXd& pos_H)
{
	for (ContactIterator contact_it = pos_H.begin();
			contact_it != pos_H.end(); contact_it++)
		setContactPosition_H(contact_it);
}


void WholeBodyState::setContactVelocity_W(ContactIterator it)
{
	setContactVelocity_W(it->first, it->second);
}


void WholeBodyState::setContactVelocity_W(const std::string& name,
										  const Eigen::VectorXd& vel_W)
{
	// Computing the contact velocity relatives base expressed in the world frame
	Eigen::Vector3d vel_fb_W = computeRelativeContactVelocity_W(name, vel_W);

	// Transforming the contact velocity in the base frame
	contact_vel[name] = frame_tf_.fromWorldToBaseFrame(vel_fb_W, getBaseRPY_W());
}


void WholeBodyState::setContactVelocity_W(const rbd::BodyVectorXd& vel_W)
{
	for (ContactIterator contact_it = vel_W.begin();
			contact_it != vel_W.end(); contact_it++)
		setContactVelocity_W(contact_it);
}


void WholeBodyState::setContactVelocity_B(ContactIterator it)
{
	setContactVelocity_B(it->first, it->second);
}


void WholeBodyState::setContactVelocity_B(const std::string& name,
										  const Eigen::VectorXd& vel_B)
{
	contact_vel[name] = vel_B;
}


void WholeBodyState::setContactVelocity_B(const rbd::BodyVectorXd& vel_B)
{
	contact_vel = vel_B;
}


void WholeBodyState::setContactVelocity_H(ContactIterator it)
{
	setContactVelocity_H(it->first, it->second);
}


void WholeBodyState::setContactVelocity_H(const std::string& name,
										  const Eigen::VectorXd& vel_H)
{
	// Computing the contact velocity expressed in the world frame
	Eigen::Vector3d vel_W =
			frame_tf_.fromHorizontalToWorldFrame(vel_H, getBaseRPY_W());

	// Computing the contact velocity relatives base expressed in the world frame
	Eigen::Vector3d vel_fb_W = computeRelativeContactVelocity_W(name, vel_W);

	// Transforming the contact velocity in the base frame
	contact_vel[name] =
			frame_tf_.fromHorizontalToBaseFrame(vel_fb_W, getBaseRPY_W());
}


void WholeBodyState::setContactVelocity_H(const rbd::BodyVectorXd& vel_H)
{
	for (ContactIterator contact_it = vel_H.begin();
			contact_it != vel_H.end(); contact_it++)
		setContactVelocity_H(contact_it);
}


void WholeBodyState::setContactAcceleration_W(ContactIterator vel_it,
											  ContactIterator acc_it)
{
	setContactAcceleration_W(vel_it->first, vel_it->second, acc_it->second);
}


void WholeBodyState::setContactAcceleration_W(const std::string& name,
											  const Eigen::VectorXd& vel_W,
											  const Eigen::VectorXd& acc_W)
{
	// Computing the contact acceleration relatives base expressed in the world frame
	Eigen::Vector3d acc_fb_W = computeRelativeContactAcceleration_W(name, vel_W, acc_W);

	// Transforming the contact acceleration in the base frame
	contact_acc[name] = frame_tf_.fromWorldToBaseFrame(acc_fb_W, getBaseRPY_W());
}


void WholeBodyState::setContactAcceleration_W(const rbd::BodyVectorXd& vel_W,
											  const rbd::BodyVectorXd& acc_W)
{
	for (ContactIterator acc_it = acc_W.begin();
			acc_it != acc_W.end(); acc_it++) {
		ContactIterator vel_it = vel_W.find(acc_it->first);
		setContactAcceleration_W(vel_it, acc_it);
	}
}


void WholeBodyState::setContactAcceleration_B(ContactIterator it)
{
	setContactAcceleration_B(it->first, it->second);
}


void WholeBodyState::setContactAcceleration_B(const std::string& name,
											  const Eigen::VectorXd& acc_B)
{
	contact_acc[name] = acc_B;
}


void WholeBodyState::setContactAcceleration_B(const rbd::BodyVectorXd& acc_B)
{
	contact_acc = acc_B;
}


void WholeBodyState::setContactAcceleration_H(ContactIterator vel_it,
											  ContactIterator acc_it)
{
	setContactAcceleration_H(vel_it->first, vel_it->second, acc_it->second);
}


void WholeBodyState::setContactAcceleration_H(const std::string& name,
											  const Eigen::VectorXd& vel_H,
											  const Eigen::VectorXd& acc_H)
{
	// Computing the contact velocity and acceleration expressed in the world frame
	Eigen::Vector3d vel_W =
			frame_tf_.fromHorizontalToWorldFrame(vel_H, getBaseRPY_W());
	Eigen::Vector3d acc_W =
			frame_tf_.fromHorizontalToWorldFrame(acc_H, getBaseRPY_W());

	// Computing the contact acceleration relatives base expressed in the world frame
	Eigen::Vector3d acc_fb_W = computeRelativeContactAcceleration_W(name, vel_W, acc_W);

	// Transforming the contact acceleration in the base frame
	contact_acc[name] =
			frame_tf_.fromHorizontalToBaseFrame(acc_fb_W, getBaseRPY_W());
}


void WholeBodyState::setContactAcceleration_H(const rbd::BodyVectorXd& vel_H,
											  const rbd::BodyVectorXd& acc_H)
{
	for (ContactIterator acc_it = acc_H.begin();
			acc_it != acc_H.end(); acc_it++) {
		ContactIterator vel_it = vel_H.find(acc_it->first);
		setContactAcceleration_H(vel_it, acc_it);
	}
}


void WholeBodyState::setContactWrench_B(const rbd::BodyVector6d& eff)
{
	contact_eff = eff;
}


void WholeBodyState::setContactWrench_B(const std::string& name,
									    const rbd::Vector6d& eff)
{
	contact_eff[name] = eff;
}


void WholeBodyState::setContactCondition(const std::string& name,
										 const bool& condition)
{
	if (condition)
		contact_eff[name] = ACTIVE_CONTACT;
	else
		contact_eff[name] = INACTIVE_CONTACT;
}


Eigen::Vector3d WholeBodyState::computeRelativeContactVelocity_W(const std::string& name,
																 const Eigen::Vector3d& vel_W)
{
	// Computing the contact velocity relatives to the base, which is expressed in
	// the world frame. Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	return vel_W - getBaseVelocity_W() -
			getBaseAngularVelocity_W().cross((Eigen::Vector3d)getContactPosition_W(name));
}


Eigen::Vector3d WholeBodyState::computeRelativeContactAcceleration_W(const std::string& name,
																	 const Eigen::Vector3d& vel_W,
																	 const Eigen::Vector3d& acc_W)
{
	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega =
			math::skewSymmetricMatrixFromVector(getBaseAngularVelocity_W());
	Eigen::Matrix3d C_omega_dot =
			math::skewSymmetricMatrixFromVector(getBaseAngularAcceleration_W());

	// Computing the contact acceleration relatives to the base, which is expressed
	// in the world frame. Here we use the equation:
	// Xdd^W_foot = Xdd^W_base + [C(wd^W) + C(w^W) * C(w^W)] X^W_foot/base
	// + 2 C(w^W) Xd^W_foot/base
	return acc_W - getBaseAcceleration_W() -
			(C_omega_dot + C_omega * C_omega) * getContactPosition_W(name) -
			2 * C_omega * computeRelativeContactVelocity_W(name, vel_W);
}

} //@namespace dwl
