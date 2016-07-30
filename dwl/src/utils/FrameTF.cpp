#include <dwl/utils/FrameTF.h>
#include <iostream>

namespace dwl
{

namespace math
{

FrameTF::FrameTF()
{

}


FrameTF::~FrameTF()
{

}


Eigen::Vector3d FrameTF::fromWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											  const Eigen::Vector3d& rpy) const
{
	return math::getQuaternion(rpy).inverse().toRotationMatrix() * vec_W;
}


Eigen::Vector3d FrameTF::fromWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											  const Eigen::Quaterniond& q) const
{
	return q.inverse().toRotationMatrix() * vec_W;
}


Eigen::Vector3d FrameTF::fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
													const Eigen::Vector3d& rpy) const
{
	return getRotWorldToHF(rpy) * vec_W;
}


Eigen::Vector3d FrameTF::fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
													const Eigen::Quaterniond& q) const
{
	return getRotWorldToHF(math::getRPY(q)) * vec_W;
}


Eigen::Vector3d FrameTF::fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											  const Eigen::Vector3d& rpy) const
{
	return math::getQuaternion(rpy).toRotationMatrix() * vec_B;
}


Eigen::Vector3d FrameTF::fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											  const Eigen::Quaterniond& q) const
{
	return q.toRotationMatrix() * vec_B;
}


Eigen::Vector3d FrameTF::fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												   const Eigen::Vector3d& rpy) const
{
	return getRotBaseToHF(rpy) * vec_B;
}


Eigen::Vector3d FrameTF::fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												   const Eigen::Quaterniond& q) const
{
	Eigen::Vector3d rpy = math::getRPY(q);
	return getRotBaseToHF(rpy) * vec_B;
}


Eigen::Vector3d FrameTF::fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
													const Eigen::Vector3d& rpy) const
{
	return getRotWorldToHF(rpy).inverse() * vec_H;
}


Eigen::Vector3d FrameTF::fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
													const Eigen::Quaterniond& q) const
{
	Eigen::Vector3d rpy = math::getRPY(q);
	return getRotWorldToHF(rpy).inverse() * vec_H;
}


Eigen::Vector3d FrameTF::mapHorizontalToWorldFrame(const Eigen::Vector3d vec_H,
Eigen::Vector3d FrameTF::mapHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Vector3d& rpy) const
{
	return fromWorldToHorizontalFrame(vec_H, rpy);
}


Eigen::Vector3d FrameTF::mapHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Quaterniond& q) const
{
	return fromWorldToHorizontalFrame(vec_H, q);
}

} //@namespace math
} //@namespace dwl
