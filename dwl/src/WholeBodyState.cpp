#include <dwl/WholeBodyState.h>
// Michele 
// accB = getOrientation_W().inverse().toRotationMatrix()*accW
// accW = getOrientation_W().toRotationMatrix()*accB
// accH = common::rpyToRot(Vector3d(0, 0, yaw)) * accW


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
	return Eigen::Quaterniond(getRotWorldToHF());
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
	return getBaseOrientation_W().inverse().toRotationMatrix() * getBaseVelocity_W();
}


Eigen::Vector3d WholeBodyState::getBaseVelocity_H() const
{
	return getRotWorldToHF() * getBaseVelocity_W();
}


Eigen::Vector3d WholeBodyState::getBaseRotationRate_W() const
{
	return base_vel.segment<3>(rbd::AX);
}


Eigen::Vector3d WholeBodyState::getBaseRotationRate_B() const
{
	return getBaseOrientation_W().inverse().toRotationMatrix() * getBaseRotationRate_W();
}


Eigen::Vector3d WholeBodyState::getBaseRotationRate_H() const
{
	return getRotWorldToHF() * getBaseRotationRate_W();
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_W() const
{
	return base_acc.segment<3>(rbd::LX);
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_B() const
{
	return getBaseOrientation_W().inverse().toRotationMatrix() * getBaseAcceleration_W();
}


Eigen::Vector3d WholeBodyState::getBaseAcceleration_H() const
{
	return getRotWorldToHF() * getBaseAcceleration_W();
}


Eigen::Vector3d WholeBodyState::getBaseRotAcceleration_W() const
{
	return base_acc.segment<3>(rbd::AX);
}


Eigen::Vector3d WholeBodyState::getBaseRotAcceleration_B() const
{
	return getBaseOrientation_W().inverse().toRotationMatrix() * getBaseRotAcceleration_W();
}


Eigen::Vector3d WholeBodyState::getBaseRotAcceleration_H() const
{
	return getRotWorldToHF() * getBaseRotAcceleration_W();
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
	rbd::linearPart(base_vel) = getBaseOrientation_W().toRotationMatrix() * vel_B;
}


void WholeBodyState::setBaseVelocity_H(const Eigen::Vector3d& vel_H)
{
	rbd::linearPart(base_vel) = getRotWorldToHF().inverse() * vel_H;
}


void WholeBodyState::setBaseRotationRate_W(const Eigen::Vector3d& rate_W)
{
	rbd::angularPart(base_vel) = rate_W;
}


void WholeBodyState::setBaseRotationRate_B(const Eigen::Vector3d& rate_B)
{
	rbd::angularPart(base_vel) = getBaseOrientation_W().toRotationMatrix() * rate_B;
}


void WholeBodyState::setBaseRotationRate_H(const Eigen::Vector3d& rate_H)
{
	rbd::angularPart(base_vel) = getRotWorldToHF().inverse() * rate_H;
}


void WholeBodyState::setBaseAcceleration_W(const Eigen::Vector3d& acc_W)
{
	rbd::linearPart(base_acc) = acc_W;
}


void WholeBodyState::setBaseAcceleration_B(const Eigen::Vector3d& acc_B)
{
	rbd::linearPart(base_acc) = getBaseOrientation_W().toRotationMatrix() * acc_B;
}


void WholeBodyState::setBaseAcceleration_H(const Eigen::Vector3d& acc_H)
{
	rbd::linearPart(base_acc) = getRotWorldToHF().inverse() * acc_H;
}


void WholeBodyState::setBaseRotAcceleration_W(const Eigen::Vector3d& rotacc_W)
{
	rbd::angularPart(base_acc) = rotacc_W;
}


void WholeBodyState::setBaseRotAcceleration_B(const Eigen::Vector3d& rotacc_B)
{
	rbd::angularPart(base_acc) = getBaseOrientation_W().toRotationMatrix() * rotacc_B;
}


void WholeBodyState::setBaseRotAcceleration_H(const Eigen::Vector3d& rotacc_H)
{
	rbd::angularPart(base_acc) = getRotWorldToHF().inverse() * rotacc_H;
}


void WholeBodyState::setJointDoF(unsigned int num_joints)
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


const double& WholeBodyState::getJointPosition(unsigned int index) const
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


const double& WholeBodyState::getJointVelocity(unsigned int index) const
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


const double& WholeBodyState::getJointAcceleration(unsigned int index) const
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	return joint_acc(index);
}


const double& WholeBodyState::getJointEffort(unsigned int index) const
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


void WholeBodyState::setJointPosition(double pos,
									  unsigned int index)
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


void WholeBodyState::setJointVelocity(double vel,
									  unsigned int index)
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


void WholeBodyState::setJointAcceleration(double acc,
										  unsigned int index)
{
	if (index >= num_joints_)
		printf(YELLOW "Warning: the index is bigger than the number of joints\n"
				COLOR_RESET);

	joint_acc(index) = acc;
}


void WholeBodyState::setJointEffort(double eff,
									unsigned int index)
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


const rbd::BodyVector& WholeBodyState::getContactPosition_B() const
{
	return contact_pos;
}


const Eigen::VectorXd& WholeBodyState::getContactPosition_B(std::string name) const
{
	return contact_pos.find(name)->second;
}


const rbd::BodyVector& WholeBodyState::getContactVelocity_B() const
{
	return contact_vel;
}


const Eigen::VectorXd& WholeBodyState::getContactVelocity_B(std::string name) const
{
	return contact_vel.find(name)->second;
}


const rbd::BodyVector& WholeBodyState::getContactAcceleration_B() const
{
	return contact_acc;
}


const Eigen::VectorXd& WholeBodyState::getContactAcceleration_B(std::string name) const
{
	return contact_acc.find(name)->second;
}


const rbd::BodyWrench& WholeBodyState::getContactEffort_B() const
{
	return contact_eff;
}


const rbd::Vector6d& WholeBodyState::getContactEffort_B(std::string name) const
{
	return contact_eff.find(name)->second;
}


bool WholeBodyState::getContactCondition(std::string name,
										 double force_threshold) const
{
	// Returns inactive in case that the contact wrench is not defined
	rbd::BodyWrench::const_iterator it = contact_eff.find(name);
	if (it == contact_eff.end())
		return false;

	if (it->second.norm() > force_threshold)
		return true;
	else
		return false;
}


void WholeBodyState::setContactPosition_B(const rbd::BodyVector& pos)
{
	contact_pos = pos;
}


void WholeBodyState::setContactPosition_B(std::string name,
										  const Eigen::VectorXd& pos)
{
	contact_pos[name] = pos;
}


void WholeBodyState::setContactVelocity_B(const rbd::BodyVector& vel)
{
	contact_vel = vel;
}


void WholeBodyState::setContactVelocity_B(std::string name,
										  const Eigen::VectorXd& vel)
{
	contact_vel[name] = vel;
}


void WholeBodyState::setContactAcceleration_B(const rbd::BodyVector& acc)
{
	contact_acc = acc;
}


void WholeBodyState::setContactAcceleration_B(std::string name,
											  const Eigen::VectorXd& acc)
{
	contact_acc[name] = acc;
}


void WholeBodyState::setContactEffort_B(const rbd::BodyWrench& eff)
{
	contact_eff = eff;
}


void WholeBodyState::setContactEffort_B(std::string name,
									    const rbd::Vector6d& eff)
{
	contact_eff[name] = eff;
}


void WholeBodyState::setContactCondition(std::string name,
										 bool condition)
{
	if (condition)
		contact_eff[name] = ACTIVE_CONTACT;
	else
		contact_eff[name] = INACTIVE_CONTACT;
}

} //@namespace dwl
