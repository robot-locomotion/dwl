#include <dwl/ReducedBodyState.h>


namespace dwl
{

ReducedBodyState::ReducedBodyState() : time(0.),
		com_pos(Eigen::Vector3d::Zero()),
		angular_pos(Eigen::Vector3d::Zero()),
		com_vel(Eigen::Vector3d::Zero()),
		angular_vel(Eigen::Vector3d::Zero()),
		com_acc(Eigen::Vector3d::Zero()),
		angular_acc(Eigen::Vector3d::Zero()),
		cop(Eigen::Vector3d::Zero())

{

}


ReducedBodyState::~ReducedBodyState()
{

}


const Eigen::Vector3d& ReducedBodyState::getCoMPosition_W() const
{
	return com_pos;
}


Eigen::Quaterniond ReducedBodyState::getOrientation_W() const
{
	return math::getQuaternion(getRPY_W());
}


const Eigen::Vector3d& ReducedBodyState::getRPY_W() const
{
	return angular_pos;
}


const Eigen::Vector3d& ReducedBodyState::getCoMVelocity_W() const
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


const Eigen::Vector3d& ReducedBodyState::getAngularVelocity_W() const
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


Eigen::Vector3d ReducedBodyState::getRPYVelocity() const
{
	// rpy^W_dot = EAR^-1 * omega^W
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(getRPY_W()).inverse();
	return EAR * getAngularVelocity_W();
}


const Eigen::Vector3d& ReducedBodyState::getCoMAcceleration_W() const
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


const Eigen::Vector3d& ReducedBodyState::getAngularAcceleration_W() const
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


Eigen::Vector3d ReducedBodyState::getRPYAcceleration() const
{
	// rpy^W_ddot = EAR^-1 * (omega^W_dot - EAR_dot * rpy^W_dot)
	Eigen::Matrix3d EAR =
			math::getInverseEulerAnglesRatesMatrix(getRPY_W()).inverse();
	Eigen::Matrix3d EARinv_dot =
			math::getInverseEulerAnglesRatesMatrix_dot(getRPY_W(), getRPYVelocity());
	return EAR * (getAngularAcceleration_W() - EARinv_dot * getRPYVelocity());
}


const Eigen::Vector3d& ReducedBodyState::getCoPPosition_W() const
{
	return cop;
}


Eigen::Vector3d ReducedBodyState::getFootPosition_W(FootIterator pos_it) const
{
	return getCoMPosition_W() +
			frame_tf_.fromBaseToWorldFrame(pos_it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootPosition_W(const std::string& name) const
{
	FootIterator foot_it = getFootPosition_B().find(name);
	return getFootPosition_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootPosition_W() const
{
	rbd::BodyVector3d foot_pos_W;
	for (FootIterator foot_it = getFootPosition_B().begin();
			foot_it != getFootPosition_B().end(); foot_it++) {
		std::string name = foot_it->first;
		foot_pos_W[name] = getFootPosition_W(foot_it);
	}

	return foot_pos_W;
}


const Eigen::Vector3d& ReducedBodyState::getFootPosition_B(FootIterator pos_it) const
{
	return pos_it->second;
}


const Eigen::Vector3d& ReducedBodyState::getFootPosition_B(const std::string& name) const
{
	FootIterator foot_it = foot_pos.find(name);
	return getFootPosition_B(foot_it);
}


const rbd::BodyVector3d& ReducedBodyState::getFootPosition_B() const
{
	return foot_pos;
}


Eigen::Vector3d ReducedBodyState::getFootPosition_H(FootIterator pos_it) const
{
	// Note that the horizontal and base frame have the same origin
	return frame_tf_.fromBaseToHorizontalFrame(pos_it->second, getRPY_W());
}


Eigen::Vector3d ReducedBodyState::getFootPosition_H(const std::string& name) const
{
	FootIterator foot_it = getFootPosition_B().find(name);
	return getFootPosition_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootPosition_H() const
{
	rbd::BodyVector3d foot_pos_H;
	for (FootIterator foot_it = getFootPosition_B().begin();
			foot_it != getFootPosition_B().end(); foot_it++) {
		std::string name = foot_it->first;
		foot_pos_H[name] = getFootPosition_H(foot_it);
	}

	return foot_pos_H;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_W(FootIterator vel_it) const
{
	// Computing the foot velocity w.r.t. the world frame.
	// Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(vel_it->first);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(vel_it);

	return getCoMVelocity_W() + vel_fb_W + getAngularVelocity_W().cross(pos_fb_W);
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_W(const std::string& name) const
{
	FootIterator foot_it = getFootVelocity_B().find(name);
	return getFootVelocity_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootVelocity_W() const
{
	rbd::BodyVector3d foot_vel_W;
	for (FootIterator foot_it = getFootVelocity_B().begin();
			foot_it != getFootVelocity_B().end(); foot_it++) {
		std::string name = foot_it->first;
		foot_vel_W[name] = getFootVelocity_W(foot_it);
	}

	return foot_vel_W;
}


const Eigen::Vector3d& ReducedBodyState::getFootVelocity_B(FootIterator vel_it) const
{
	return vel_it->second;
}


const Eigen::Vector3d& ReducedBodyState::getFootVelocity_B(const std::string& name) const
{
	FootIterator foot_it = getFootVelocity_B().find(name);
	return getFootVelocity_B(foot_it);
}


const rbd::BodyVector3d& ReducedBodyState::getFootVelocity_B() const
{
	return foot_vel;
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_H(FootIterator vel_it) const
{
	// Computing the foot velocity w.r.t. the world frame.
	// Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega^W_base x X^W_foot/base
	std::string name = vel_it->first;
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(vel_it);
	Eigen::Vector3d vel_W = getCoMVelocity_W() + vel_fb_W +
			getAngularVelocity_W().cross(pos_fb_W);

	// Computing the foot velocity w.r.t. the horizontal expressed in the world
	// frame. Here we use the equation:
	// Xd^W_foot = Xd^W_hor + Xd^W_foot/hor + omega^W_hor x X^W_foot/hor
	Eigen::Vector3d omega_hor_W(0., 0., getAngularVelocity_W()(rbd::Z));
	Eigen::Vector3d pos_fh_W =
			frame_tf_.fromHorizontalToWorldFrame(getFootPosition_H(name), getRPY_W());
	return vel_W - getCoMVelocity_W() - omega_hor_W.cross(pos_fh_W);
}


Eigen::Vector3d ReducedBodyState::getFootVelocity_H(const std::string& name) const
{
	FootIterator foot_it = getFootVelocity_B().find(name);
	return getFootVelocity_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootVelocity_H() const
{
	rbd::BodyVector3d foot_vel_H;
	for (FootIterator foot_it = getFootVelocity_B().begin();
			foot_it != getFootVelocity_B().end(); foot_it++) {
		std::string name = foot_it->first;
		foot_vel_H[name] = getFootVelocity_H(foot_it);
	}

	return foot_vel_H;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_W(FootIterator acc_it) const
{
	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega =
			math::skewSymmetricMatrixFromVector(getAngularVelocity_W());
	Eigen::Matrix3d C_omega_dot =
			math::skewSymmetricMatrixFromVector(getAngularAcceleration_W());

	// Computing the foot acceleration w.r.t. the world frame.
	// Here we use the equation:
	// Xdd^W_foot = Xdd^W_base + [C(wd^W) + C(w^W) * C(w^W)] X^W_foot/base
	// + 2 C(w^W) Xd^W_foot/base
	std::string name = acc_it->first;
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(name);
	return getCoMAcceleration_W() +
			(C_omega_dot + C_omega * C_omega) * pos_fb_W + 2 * C_omega * vel_fb_W;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_W(const std::string& name) const
{
	FootIterator foot_it = getFootAcceleration_B().find(name);
	return getFootAcceleration_W(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootAcceleration_W() const
{
	rbd::BodyVector3d foot_acc_W;
	for (FootIterator foot_it = getFootAcceleration_B().begin();
			foot_it != getFootAcceleration_B().end(); foot_it++) {
		std::string name = foot_it->first;
		foot_acc_W[name] = getFootAcceleration_W(foot_it);
	}

	return foot_acc_W;
}


const Eigen::Vector3d& ReducedBodyState::getFootAcceleration_B(FootIterator acc_it) const
{
	return acc_it->second;
}


const Eigen::Vector3d& ReducedBodyState::getFootAcceleration_B(const std::string& name) const
{
	FootIterator foot_it = getFootAcceleration_B().find(name);
	return getFootAcceleration_B(foot_it);
}


const rbd::BodyVector3d& ReducedBodyState::getFootAcceleration_B() const
{
	return foot_acc;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_H(FootIterator acc_it) const
{
	std::string name = acc_it->first;

	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega_b =
			math::skewSymmetricMatrixFromVector(getAngularVelocity_W());
	Eigen::Matrix3d C_omegad_b =
			math::skewSymmetricMatrixFromVector(getAngularAcceleration_W());
	Eigen::Vector3d omega_hor(0., 0., getAngularVelocity_W()(rbd::Z));
	Eigen::Vector3d omegad_hor(0., 0., getAngularAcceleration_W()(rbd::Z));
	Eigen::Matrix3d C_omega_h =
			math::skewSymmetricMatrixFromVector(omega_hor);
	Eigen::Matrix3d C_omegad_h =
			math::skewSymmetricMatrixFromVector(omegad_hor);

	// Computing the foot acceleration w.r.t. the world frame.
	// Here we use the equation:
	// Xdd^W_foot = Xdd^W_base + [C(wd^W_base) + C(w^W_base) * C(w^W_base)] X^W_foot/base
	// + 2 C(w^W_base) Xd^W_foot/base + Xdd^W_foot/base
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(name);
	Eigen::Vector3d acc_W = getCoMVelocity_W() +
			(C_omegad_b + C_omega_b * C_omega_b) * pos_fb_W +
			2 * C_omega_b * vel_fb_W;

	// Computing the foot acceleration w.r.t. the horizontal.
	// Here we use the equation:
	// Xdd^W_foot = Xdd^W_hor + [C(wd^W_hor) + C(w^W_hor) * C(w^W_hor)] X^W_foot/hor
	// + 2 C(w^W_hor) Xd^W_foot/hor + Xdd^W_foot/hor
	Eigen::Matrix3d W_rot_H = frame_tf_.getHorizontalToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fh_W = W_rot_H * getFootPosition_H(name);
	Eigen::Vector3d vel_fh_W = W_rot_H * getFootVelocity_H(name);
	return acc_W - getCoMAcceleration_W() -
			(C_omegad_h + C_omega_h * C_omega_h) * pos_fh_W +
			2 * C_omega_h * vel_fh_W;
}


Eigen::Vector3d ReducedBodyState::getFootAcceleration_H(const std::string& name) const
{
	FootIterator foot_it = getFootAcceleration_B().find(name);
	return getFootAcceleration_H(foot_it);
}


rbd::BodyVector3d ReducedBodyState::getFootAcceleration_H() const
{
	rbd::BodyVector3d foot_acc_H;
	for (FootIterator foot_it = getFootAcceleration_B().begin();
			foot_it != getFootAcceleration_B().end(); foot_it++) {
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


void ReducedBodyState::setRPYVelocity(const Eigen::Vector3d& rpy_rate)
{
	angular_vel = math::getInverseEulerAnglesRatesMatrix(getRPY_W()) * rpy_rate;
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


void ReducedBodyState::setRPYAcceleration(const Eigen::Vector3d& rpy_acc)
{
	// omega_ddot = EAR * rpy_ddot + EAR_dot * rpy_dot
	Eigen::Vector3d rpy_vel = getRPYVelocity();
	angular_acc =
			math::getInverseEulerAnglesRatesMatrix(getRPY_W()) * rpy_acc +
			math::getInverseEulerAnglesRatesMatrix_dot(getRPY_W(), rpy_vel) * rpy_vel;
}


void ReducedBodyState::setCoPPosition_W(const Eigen::Vector3d& cop_W)
{
	cop = cop_W;
}


void ReducedBodyState::setFootPosition_W(FootIterator it)
{
	setFootPosition_W(it->first, it->second);
}


void ReducedBodyState::setFootPosition_W(const std::string& name,
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


void ReducedBodyState::setFootPosition_B(const std::string& name,
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


void ReducedBodyState::setFootPosition_H(const std::string& name,
										 const Eigen::Vector3d& pos_H)
{
	// Note that the horizontal and base frames have the same origin
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


void ReducedBodyState::setFootVelocity_W(const std::string& name,
										 const Eigen::Vector3d& vel_W)
{
	// Computing the foot velocity w.r.t. the base but expressed in the world
	// frame. Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega_base x X^W_foot/base
	Eigen::Vector3d pos_fb_W =
			frame_tf_.fromBaseToWorldFrame(getFootPosition_B(name), getRPY_W());
	Eigen::Vector3d vel_fb_W = vel_W - getCoMVelocity_W() -
			getAngularVelocity_W().cross(pos_fb_W);

	foot_vel[name] = frame_tf_.fromWorldToBaseFrame(vel_fb_W, getRPY_W());
	// Expressing the foot velocity in the base frame
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


void ReducedBodyState::setFootVelocity_B(const std::string& name,
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


void ReducedBodyState::setFootVelocity_H(const std::string& name,
										 const Eigen::Vector3d& vel_H)
{
	// Computing the foot velocity w.r.t. the world frame.
	// Here we use the equation:
	// Xd^W_foot = Xd^W_hor + Xd^W_foot/hor + omega^W_hor x X^W_foot/hor
	Eigen::Matrix3d W_rot_H = frame_tf_.getHorizontalToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fh_W = W_rot_H * getFootPosition_H(name);
	Eigen::Vector3d vel_fh_W = W_rot_H * vel_H;
	Eigen::Vector3d omega_h_W(0., 0., getAngularVelocity_W()(rbd::Z));
	Eigen::Vector3d vel_W = getCoMVelocity_W() + vel_fh_W +	omega_h_W.cross(pos_fh_W);

	// Computing the foot velocity w.r.t. the base but expressed in the world
	// frame. Here we use the equation:
	// Xd^W_foot = Xd^W_base + Xd^W_foot/base + omega^W_base x X^W_foot/base
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = vel_W - getCoMVelocity_W() -
			getAngularVelocity_W().cross(pos_fb_W);

	// Expressing the foot velocity in the base frame
	foot_vel[name] = W_rot_B.transpose() * vel_fb_W;
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


void ReducedBodyState::setFootAcceleration_W(const std::string& name,
											 const Eigen::Vector3d& acc_W)
{
	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega =
			math::skewSymmetricMatrixFromVector(getAngularVelocity_W());
	Eigen::Matrix3d C_omega_dot =
			math::skewSymmetricMatrixFromVector(getAngularAcceleration_W());

	// Computing the foot acceleration w.r.t. the base but expressed in the
	// world frame. Here we use the equation:
	// Xdd^W_foot = Xdd^W_base + [C(wd^W) + C(w^W) * C(w^W)] X^W_foot/base
	// + 2 C(w^W) Xd^W_foot/base + Xdd^W_foot/base
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(name);
	Eigen::Vector3d acc_fb_W = acc_W - getCoMAcceleration_W() -
			(C_omega_dot + C_omega * C_omega) * pos_fb_W -
			2 * C_omega * vel_fb_W;

	// Expressing the foot acceleration in the base frame
	foot_acc[name] = W_rot_B.transpose() * acc_fb_W;
}


void ReducedBodyState::setFootAcceleration_W(const rbd::BodyVector3d& acc_W)
{
	for (FootIterator acc_it = acc_W.begin();
			acc_it != acc_W.end(); acc_it++) {
		setFootAcceleration_W(acc_it);
	}
}


void ReducedBodyState::setFootAcceleration_B(FootIterator it)
{
	setFootAcceleration_B(it->first, it->second);
}


void ReducedBodyState::setFootAcceleration_B(const std::string& name,
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


void ReducedBodyState::setFootAcceleration_H(const std::string& name,
											 const Eigen::Vector3d& acc_H)
{
	// Computing the skew symmetric matrixes
	Eigen::Matrix3d C_omega_b =
			math::skewSymmetricMatrixFromVector(getAngularVelocity_W());
	Eigen::Matrix3d C_omegad_b =
			math::skewSymmetricMatrixFromVector(getAngularAcceleration_W());
	Eigen::Vector3d omega_h(0., 0., getAngularVelocity_W()(rbd::Z));
	Eigen::Vector3d omegad_h(0., 0., getAngularAcceleration_W()(rbd::Z));
	Eigen::Matrix3d C_omega_h =
			math::skewSymmetricMatrixFromVector(omega_h);
	Eigen::Matrix3d C_omegad_h =
			math::skewSymmetricMatrixFromVector(omegad_h);

	// Computing the foot acceleration w.r.t. the world frame.
	// Here we use the equation:
	// Xdd^W_foot = Xdd^W_hor + [C(wd^W_hor) + C(w^W_hor) * C(w^W_hor)] X^W_foot/hor
	// + 2 C(w^W_hor) Xd^W_foot/hor + Xdd^W_foot/hor
	Eigen::Matrix3d W_rot_H = frame_tf_.getHorizontalToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fh_W = W_rot_H * getFootPosition_H(name);
	Eigen::Vector3d vel_fh_W = W_rot_H * getFootVelocity_H(name);
	Eigen::Vector3d acc_fh_W = W_rot_H * acc_H;
	Eigen::Vector3d acc_W =
			getCoMAcceleration_W() +
			(C_omegad_h + C_omega_h * C_omega_h) * pos_fh_W +
			2 * C_omega_h * vel_fh_W + acc_fh_W;

	// Computing the foot acceleration w.r.t. the base but expressed in the
	// world frame. Here we use the equation:
	// Xdd^W_foot = Xdd^W_base + [C(wd^W) + C(w^W) * C(w^W)] X^W_foot/base
	// + 2 C(w^W) Xd^W_foot/base + Xdd^W_foot/base
	Eigen::Matrix3d W_rot_B = frame_tf_.getBaseToWorldRotation(getRPY_W());
	Eigen::Vector3d pos_fb_W = W_rot_B * getFootPosition_B(name);
	Eigen::Vector3d vel_fb_W = W_rot_B * getFootVelocity_B(name);
	Eigen::Vector3d acc_fb_W =
			acc_W - getCoMAcceleration_W() -
			(C_omegad_b + C_omega_b * C_omega_b) * pos_fb_W -
			2 * C_omega_b * vel_fb_W;


	// Expressing the foot acceleration in the base frame
	foot_acc[name] = W_rot_B.transpose() * acc_fb_W;
}


void ReducedBodyState::setFootAcceleration_H(const rbd::BodyVector3d& acc_H)
{
	for (FootIterator acc_it = acc_H.begin();
			acc_it != acc_H.end(); acc_it++) {
		setFootAcceleration_H(acc_it);
	}
}

} //@namespace dwl
