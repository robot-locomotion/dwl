#include <dwl/ReducedBodyState.h>


namespace dwl
{

ReducedBodyState::ReducedBodyState() : time(0.)
{
	com_pos.setZero();
	angular_pos.setZero();
	com_vel.setZero();
	angular_vel.setZero();
	com_acc.setZero();
	angular_acc.setZero();
	cop.setZero();
}


ReducedBodyState::~ReducedBodyState()
{

}


Eigen::Vector3d ReducedBodyState::getCoMPosition_W() const
{
	return com_pos;
}


Eigen::Vector3d ReducedBodyState::getCoMPosition_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getCoMPosition_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getCoMPosition_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getCoMPosition_W(),
												getRPY_W());
}


Eigen::Quaterniond ReducedBodyState::getOrientation_W() const
{
	return math::getQuaternion(getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getRPY_W() const
{
	return angular_pos;
}


Eigen::Vector3d ReducedBodyState::getCoMVelocity_W() const
{
	return com_vel;
}


Eigen::Vector3d ReducedBodyState::getCoMVelocity_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getCoMVelocity_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getCoMVelocity_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getCoMVelocity_W(),
												getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getRotationRate_W() const
{
	return angular_vel;
}


Eigen::Vector3d ReducedBodyState::getRotationRate_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getRotationRate_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getRotationRate_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getRotationRate_W(),
												getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getCoMAcceleration_W() const
{
	return com_acc;
}


Eigen::Vector3d ReducedBodyState::getCoMAcceleration_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getCoMAcceleration_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getCoMAcceleration_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getCoMAcceleration_W(),
												getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getRotAcceleration_W() const
{
	return angular_acc;
}


Eigen::Vector3d ReducedBodyState::getRotAcceleration_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getRotAcceleration_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getRotAcceleration_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getRotAcceleration_W(),
												getRPY_W());
}


void ReducedBodyState::setCoMPosition_W(const Eigen::Vector3d& pos_W)
{
	com_pos = pos_W;
}


void ReducedBodyState::setCoMPosition_B(const Eigen::Vector3d& pos_B)
{
	com_pos = frame_tf_.fromBaseToWorldFrame(pos_B, getOrientation_W());
}


void ReducedBodyState::setCoMPosition_H(const Eigen::Vector3d& pos_H)
{
	com_pos = frame_tf_.fromHorizontalToWorldFrame(pos_H, getRPY_W());
}


void ReducedBodyState::setOrientation_W(const Eigen::Quaterniond& orient_W)
{
	angular_pos = math::getRPY(orient_W);
}


void ReducedBodyState::setRPY_W(const Eigen::Vector3d& rpy_W)
{
	angular_pos = rpy_W;
}


void ReducedBodyState::setCoMVelocity_W(const Eigen::Vector3d& vel_W)
{
	com_vel = vel_W;
}


void ReducedBodyState::setCoMVelocity_B(const Eigen::Vector3d& vel_B)
{
	com_vel = frame_tf_.fromBaseToWorldFrame(vel_B, getOrientation_W());
}


void ReducedBodyState::setCoMVelocity_H(const Eigen::Vector3d& vel_H)
{
	com_vel = frame_tf_.fromHorizontalToWorldFrame(vel_H, getRPY_W());
}


void ReducedBodyState::setRotationRate_W(const Eigen::Vector3d& rate_W)
{
	angular_vel = rate_W;
}


void ReducedBodyState::setRotationRate_B(const Eigen::Vector3d& rate_B)
{
	angular_vel = frame_tf_.fromBaseToWorldFrame(rate_B, getOrientation_W());
}


void ReducedBodyState::setRotationRate_H(const Eigen::Vector3d& rate_H)
{
	angular_vel = frame_tf_.fromHorizontalToWorldFrame(rate_H, getRPY_W());
}


void ReducedBodyState::setCoMAcceleration_W(const Eigen::Vector3d& acc_W)
{
	com_acc = acc_W;
}


void ReducedBodyState::setCoMAcceleration_B(const Eigen::Vector3d& acc_B)
{
	com_acc = frame_tf_.fromBaseToWorldFrame(acc_B, getOrientation_W());
}


void ReducedBodyState::setCoMAcceleration_H(const Eigen::Vector3d& acc_H)
{
	com_acc = frame_tf_.fromHorizontalToWorldFrame(acc_H, getRPY_W());
}


void ReducedBodyState::setRotAcceleration_W(const Eigen::Vector3d& rotacc_W)
{
	angular_acc = rotacc_W;
}


void ReducedBodyState::setRotAcceleration_B(const Eigen::Vector3d& rotacc_B)
{
	angular_acc = frame_tf_.fromBaseToWorldFrame(rotacc_B, getOrientation_W());
}


void ReducedBodyState::setRotAcceleration_H(const Eigen::Vector3d& rotacc_H)
{
	angular_acc = frame_tf_.fromHorizontalToWorldFrame(rotacc_H, getRPY_W());
}

} //@namespace dwl
