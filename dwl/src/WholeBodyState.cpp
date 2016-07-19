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


void WholeBodyState::setJointDoF(unsigned int num_joints)
{
	joint_pos.setZero(num_joints);
	joint_vel.setZero(num_joints);
	joint_acc.setZero(num_joints);
	joint_eff.setZero(num_joints);
}



const unsigned int WholeBodyState::getJointDof() const
{
	return joint_pos.size();
}


} //@namespace dwl
