#include <dwl/WholeBodyState.h>


namespace dwl
{

WholeBodyState::WholeBodyState(unsigned int num_joints) :
		time(0.), duration(0.)
{
	base_pos.setZero();
	base_vel.setZero();
	base_acc.setZero();
	base_eff.setZero();

	if (num_joints != 0) {
		joint_pos.setZero(num_joints);
		joint_vel.setZero(num_joints);
		joint_acc.setZero(num_joints);
		joint_eff.setZero(num_joints);
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


const unsigned int WholeBodyState::getJointDof() const
{
	return joint_pos.size();
}


void WholeBodyState::setJointDoF(unsigned int num_joints)
{
	joint_pos.setZero(num_joints);
	joint_vel.setZero(num_joints);
	joint_acc.setZero(num_joints);
	joint_eff.setZero(num_joints);
}






} //@namespace dwl
