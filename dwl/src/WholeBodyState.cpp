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


Eigen::Vector3d WholeBodyState::getPosition_W()
{
    return base_pos.segment<3>(rbd::LX);
}


Eigen::Quaterniond WholeBodyState::getOrientation_W()
{
	return math::getQuaternion(getRPY_W());
}


Eigen::Vector3d WholeBodyState::getRPY_W()
{
    return base_pos.segment<3>(rbd::AX);
}


Eigen::Quaterniond WholeBodyState::getOrientation_H()
{
	return Eigen::Quaterniond(getRotBaseToHF());
}


Eigen::Vector3d WholeBodyState::getVelocity_W()
{
	return base_vel.segment<3>(rbd::LX);
}


Eigen::Vector3d WholeBodyState::getVelocity_B()
{
    return (Eigen::Affine3d(getOrientation_W())).inverse() * getVelocity_W();
}


Eigen::Vector3d WholeBodyState::getVelocity_H()
{
    // put back the velocity in the base frame into the horizontal frame
    return getRotBaseToHF() * getVelocity_B();
}


Eigen::Vector3d WholeBodyState::getRotationRate_B()
{// TODO think if we should used the world as standard
	//	return (Eigen::Affine3d(getOrientation_W())).inverse() * rotvel_W; transform from W to B
    return base_vel.segment<3>(rbd::AX);
}


Eigen::Vector3d WholeBodyState::getAcceleration_W()
{
	return base_acc.segment<3>(rbd::LX);
}


Eigen::Vector3d WholeBodyState::getAcceleration_B()
{
	return (Eigen::Affine3d(getOrientation_W())).inverse() * getAcceleration_W();
}


Eigen::Vector3d WholeBodyState::getAcceleration_H()
{
    // put back the acceleration in the base frame into the horizontal frame
    return getRotBaseToHF() * getAcceleration_B();
}


Eigen::Vector3d WholeBodyState::getRotAcceleration_B()
{// TODO think if we should used the world as standard
//	return (Eigen::Affine3d(getOrientation_W())).inverse() * rotacc_W; transform from W to B
	return base_acc.segment<3>(rbd::AX);
}


void WholeBodyState::setPosition_W(const Eigen::Vector3d& pos)
{
	rbd::linearPart(base_pos) = pos;
}


void WholeBodyState::setOrientation_W(const Eigen::Quaterniond& orient)
{
	rbd::angularPart(base_pos) = math::getRPY(orient);
}


void WholeBodyState::setRPY_W(const Eigen::Vector3d& rpy)
{
	rbd::angularPart(base_pos) = rpy;
}


void WholeBodyState::setVelocity_W(const Eigen::Vector3d& vel)
{
	rbd::linearPart(base_vel) = vel;
}


//void WholeBodyState::setVelocity_B(const Eigen::Vector3d& vel)
//{
//	rbd::linearPart(base_vel) = Eigen::Affine3d(getOrientation_W()) * vel; ?
//}


void WholeBodyState::setRotationRate_W(const Eigen::Vector3d& rate)
{// TODO think if we should used the world as standard
//	rbd::angularPart(base_vel) = rate;
	rbd::angularPart(base_vel) = (Eigen::Affine3d(getOrientation_W())).inverse() * rate;
}


void WholeBodyState::setRotationRate_B(const Eigen::Vector3d& rate)
{// TODO think if we should used the world as standard
//	rbd::angularPart(base_vel) = Eigen::Affine3d(getOrientation_W()) * rate; ?
	rbd::angularPart(base_vel) = rate;
}


void WholeBodyState::setAcceleration_W(const Eigen::Vector3d& acc)
{
	rbd::linearPart(base_acc) = acc;
}


//void WholeBodyState::setAcceleration_B(const Eigen::Vector3d& acc)
//{
//	rbd::angularPart(base_acc) = Eigen::Affine3d(getOrientation_W()) * acc; ?
//}


void WholeBodyState::setRotAcceleration_W(const Eigen::Vector3d& rotacc)
{// TODO think if we should used the world as standard
//	rbd::angularPart(base_acc) = Eigen::Affine3d(getOrientation_W()) * rotacc; ?
	rbd::angularPart(base_acc) = rotacc;
}


void WholeBodyState::setRotAcceleration_B(const Eigen::Vector3d& rotacc)
{// TODO think if we should used the world as standard
//	rbd::angularPart(base_acc) = Eigen::Affine3d(getOrientation_W()) * rotacc; ?
	rbd::angularPart(base_acc) = rotacc;
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


const Eigen::VectorXd& WholeBodyState::getJointVelocity() const
{
	return joint_vel;
}


const Eigen::VectorXd& WholeBodyState::getJointAcceleration() const
{
	return joint_acc;
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


void WholeBodyState::setJointVelocity(const Eigen::VectorXd& vel)
{
	joint_vel = vel;
}


void WholeBodyState::setJointAcceleration(const Eigen::VectorXd& acc)
{
	joint_acc = acc;
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

} //@namespace dwl
