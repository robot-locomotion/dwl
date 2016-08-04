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


Eigen::Vector3d ReducedBodyState::getAngularVelocity_W() const
{
	return angular_vel;
}


Eigen::Vector3d ReducedBodyState::getAngularVelocity_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getAngularVelocity_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getAngularVelocity_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getAngularVelocity_W(),
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


Eigen::Vector3d ReducedBodyState::getAngularAcceleration_W() const
{
	return angular_acc;
}


Eigen::Vector3d ReducedBodyState::getAngularAcceleration_B() const
{
	return frame_tf_.fromWorldToBaseFrame(getAngularAcceleration_W(),
										  getOrientation_W());
}


Eigen::Vector3d ReducedBodyState::getAngularAcceleration_H() const
{
	return frame_tf_.fromWorldToHorizontalFrame(getAngularAcceleration_W(),
												getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootPosition_W(FootIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootPosition_W(std::string name) const
{
	FootIterator foot_it = foot_pos.find(name);
	return getFootPosition_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootPosition_W() const
{
	rbd::BodyVector3d foot_pos_W;
	for (FootIterator foot_it = foot_pos.begin();
			foot_it != foot_pos.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_pos_W[name] = getFootPosition_W(foot_it);
	}

	return foot_pos_W;
}


Eigen::Vector3d ReducedBodyState::getFootPosition_B(FootIterator it) const
{
	return it->second;
}


Eigen::Vector3d ReducedBodyState::getFootPosition_B(std::string name) const
{
	FootIterator foot_it = foot_pos.find(name);
	return getFootPosition_B(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootPosition_B() const
{
	return foot_pos;
}


Eigen::Vector3d ReducedBodyState::getFootPosition_H(FootIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootPosition_H(std::string name) const
{
	FootIterator foot_it = foot_pos.find(name);
	return getFootPosition_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootPosition_H() const
{
	rbd::BodyVector3d foot_pos_H;
	for (FootIterator foot_it = foot_pos.begin();
			foot_it != foot_pos.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_pos_H[name] = getFootPosition_H(foot_it);
	}

	return foot_pos_H;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_W(FootIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_W(std::string name) const
{
	FootIterator foot_it = foot_vel.find(name);
	return getFootVelocity_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootVelocity_W() const
{
	rbd::BodyVector3d foot_vel_W;
	for (FootIterator foot_it = foot_vel.begin();
			foot_it != foot_vel.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_vel_W[name] = getFootVelocity_W(foot_it);
	}

	return foot_vel_W;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_B(FootIterator it) const
{
	return it->second;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_B(std::string name) const
{
	FootIterator foot_it = foot_vel.find(name);
	return getFootVelocity_B(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootVelocity_B() const
{
	return foot_vel;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_H(FootIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_H(std::string name) const
{
	FootIterator foot_it = foot_vel.find(name);
	return getFootVelocity_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootVelocity_H() const
{
	rbd::BodyVector3d foot_vel_H;
	for (FootIterator foot_it = foot_vel.begin();
			foot_it != foot_vel.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_vel_H[name] = getFootVelocity_H(foot_it);
	}

	return foot_vel_H;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_W(FootIterator it) const
{
	return frame_tf_.fromBaseToWorldFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_W(std::string name) const
{
	FootIterator foot_it = foot_acc.find(name);
	return getFootAcceleration_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootAcceleration_W() const
{
	rbd::BodyVector3d foot_acc_W;
	for (FootIterator foot_it = foot_acc.begin();
			foot_it != foot_acc.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_acc_W[name] = getFootAcceleration_W(foot_it);
	}

	return foot_acc_W;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_B(FootIterator it) const
{
	return it->second;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_B(std::string name) const
{
	FootIterator foot_it = foot_acc.find(name);
	return getFootAcceleration_B(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootAcceleration_B() const
{
	return foot_acc;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_H(FootIterator it) const
{
	return frame_tf_.fromBaseToHorizontalFrame(it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_H(std::string name) const
{
	FootIterator foot_it = foot_acc.find(name);
	return getFootAcceleration_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootAcceleration_H() const
{
	rbd::BodyVector3d foot_acc_H;
	for (FootIterator foot_it = foot_acc.begin();
			foot_it != foot_acc.end(); foot_it++) {
		std::string name = foot_it->first;
		foot_acc_H[name] = getFootAcceleration_H(foot_it);
	}

	return foot_acc_H;
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


void ReducedBodyState::setAngularVelocity_W(const Eigen::Vector3d& rate_W)
{
	angular_vel = rate_W;
}


void ReducedBodyState::setAngularVelocity_B(const Eigen::Vector3d& rate_B)
{
	angular_vel = frame_tf_.fromBaseToWorldFrame(rate_B, getOrientation_W());
}


void ReducedBodyState::setAngularVelocity_H(const Eigen::Vector3d& rate_H)
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


void ReducedBodyState::setAngularAcceleration_W(const Eigen::Vector3d& rotacc_W)
{
	angular_acc = rotacc_W;
}


void ReducedBodyState::setAngularAcceleration_B(const Eigen::Vector3d& rotacc_B)
{
	angular_acc = frame_tf_.fromBaseToWorldFrame(rotacc_B, getOrientation_W());
}


void ReducedBodyState::setAngularAcceleration_H(const Eigen::Vector3d& rotacc_H)
{
	angular_acc = frame_tf_.fromHorizontalToWorldFrame(rotacc_H, getRPY_W());
}


void ReducedBodyState::setFootPosition_W(FootIterator it)
{
	setFootPosition_W(it->first, it->second);
}


void ReducedBodyState::setFootPosition_W(std::string name,
										 const Eigen::Vector3d& pos_W)
{
	foot_pos[name] =
			frame_tf_.fromWorldToBaseFrame(pos_W - getCoMPosition_W(),
										   getRPY_W());
}


void ReducedBodyState::setFootPosition_W(const rbd::BodyVector3d& pos_W)
{
	for (FootIterator foot_it = pos_W.begin();
			foot_it != pos_W.end(); foot_it++)
		setFootPosition_W(foot_it);
}


void ReducedBodyState::setFootPosition_B(FootIterator it)
{
	setFootPosition_B(it->first, it->second);
}


void ReducedBodyState::setFootPosition_B(std::string name,
										 const Eigen::Vector3d& pos_B)
{
	foot_pos[name] = pos_B;
}


void ReducedBodyState::setFootPosition_B(const rbd::BodyVector3d& pos_B)
{
	foot_pos = pos_B;
}


void ReducedBodyState::setFootPosition_H(FootIterator it)
{
	setFootPosition_H(it->first, it->second);
}


void ReducedBodyState::setFootPosition_H(std::string name,
										 const Eigen::Vector3d& pos_H)
{
	foot_pos[name] =
			frame_tf_.fromHorizontalToBaseFrame(pos_H, getRPY_W());
}


void ReducedBodyState::setFootPosition_H(const rbd::BodyVector3d& pos_H)
{
	for (FootIterator foot_it = pos_H.begin();
			foot_it != pos_H.end(); foot_it++)
		setFootPosition_H(foot_it);
}


void ReducedBodyState::setFootVelocity_W(FootIterator it)
{
	setFootVelocity_W(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_W(std::string name,
										 const Eigen::Vector3d& vel_W)
{
	// Computing the foot velocity relatives base expressed in the world frame
	// Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	Eigen::Vector3d vel_fb_W = computeFootRelativeVelocity_W(name, vel_W);

	// Transforming the foot velocity in the base frame
	foot_vel[name] = frame_tf_.fromWorldToBaseFrame(vel_fb_W, getRPY_W());
}


void ReducedBodyState::setFootVelocity_W(const rbd::BodyVector3d& vel_W)
{
	for (FootIterator foot_it = vel_W.begin();
			foot_it != vel_W.end(); foot_it++)
		setFootVelocity_W(foot_it);
}


void ReducedBodyState::setFootVelocity_B(FootIterator it)
{
	setFootVelocity_B(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_B(std::string name,
										 const Eigen::Vector3d& vel_B)
{
	foot_vel[name] = vel_B;
}


void ReducedBodyState::setFootVelocity_B(const rbd::BodyVector3d& vel_B)
{
	foot_vel = vel_B;
}


void ReducedBodyState::setFootVelocity_H(FootIterator it)
{
	setFootVelocity_H(it->first, it->second);
}


void ReducedBodyState::setFootVelocity_H(std::string name,
										 const Eigen::Vector3d& vel_H)
{
	// Computing the foot velocity expressed in the world frame
	Eigen::Vector3d vel_W =
			frame_tf_.fromHorizontalToWorldFrame(vel_H, getRPY_W());

	// Computing the foot velocity relatives base expressed in the world frame
	// Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	Eigen::Vector3d vel_fb_W = computeFootRelativeVelocity_W(name, vel_W);

	foot_vel[name] =
			frame_tf_.fromHorizontalToBaseFrame(vel_H, getRPY_W());
}


void ReducedBodyState::setFootVelocity_H(const rbd::BodyVector3d& vel_H)
{
	for (FootIterator foot_it = vel_H.begin();
			foot_it != vel_H.end(); foot_it++)
		setFootVelocity_H(foot_it);
}


void ReducedBodyState::setFootAcceleration_W(FootIterator it)
{
	setFootAcceleration_W(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_W(std::string name,
											 const Eigen::Vector3d& acc_W)
{
	foot_acc[name] =
			frame_tf_.fromWorldToBaseFrame(acc_W - getCoMAcceleration_W(),
										   getRPY_W());
}


void ReducedBodyState::setFootAcceleration_W(const rbd::BodyVector3d& acc_W)
{
	for (FootIterator foot_it = acc_W.begin();
			foot_it != acc_W.end(); foot_it++)
		setFootAcceleration_W(foot_it);
}


void ReducedBodyState::setFootAcceleration_B(FootIterator it)
{
	setFootAcceleration_B(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_B(std::string name,
											 const Eigen::Vector3d& acc_B)
{
	foot_acc[name] = acc_B;
}


void ReducedBodyState::setFootAcceleration_B(const rbd::BodyVector3d& acc_B)
{
	foot_acc = acc_B;
}


void ReducedBodyState::setFootAcceleration_H(FootIterator it)
{
	setFootAcceleration_H(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_H(std::string name,
											 const Eigen::Vector3d& acc_H)
{
	foot_acc[name] =
			frame_tf_.fromHorizontalToBaseFrame(acc_H, getRPY_W());
}


void ReducedBodyState::setFootAcceleration_H(const rbd::BodyVector3d& acc_H)
{
	for (FootIterator foot_it = acc_H.begin();
			foot_it != acc_H.end(); foot_it++)
		setFootAcceleration_H(foot_it);
}


Eigen::Vector3d ReducedBodyState::computeFootRelativeVelocity_W(std::string name,
																const Eigen::Vector3d& vel_W)
{
	// Computing the foot velocity relatives base expressed in the world frame
	// Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	return vel_W - getCoMVelocity_W() -
			getAngularVelocity_W().cross(getFootPosition_W(name));
}

} //@namespace dwl
