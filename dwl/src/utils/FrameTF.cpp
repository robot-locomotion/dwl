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


Eigen::Vector3d FrameTF::fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Vector3d& rpy) const
{
	return getRotBaseToHF(rpy).inverse() * vec_H;
}


Eigen::Vector3d FrameTF::fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Quaterniond& q) const
{
	Eigen::Vector3d rpy = math::getRPY(q);
	return getRotBaseToHF(rpy).inverse() * vec_H;
}


Eigen::Vector3d FrameTF::mapWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											 const Eigen::Vector3d& rpy) const
{
	return fromBaseToWorldFrame(vec_W, rpy);
}


Eigen::Vector3d FrameTF::mapWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											 const Eigen::Quaterniond& q) const
{
	return fromBaseToWorldFrame(vec_W, q);
}


Eigen::Vector3d FrameTF::mapBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											 const Eigen::Vector3d& rpy) const
{
	return fromWorldToBaseFrame(vec_B, rpy);
}


Eigen::Vector3d FrameTF::mapBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											 const Eigen::Quaterniond& q) const
{
	return fromWorldToBaseFrame(vec_B, q);
}


Eigen::Vector3d FrameTF::mapWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												   const Eigen::Vector3d& rpy) const
{
	return fromHorizontalToWorldFrame(vec_W, rpy);
}


Eigen::Vector3d FrameTF::mapWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
												   const Eigen::Quaterniond& q) const
{
	return fromHorizontalToWorldFrame(vec_W, q);
}


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


Eigen::Vector3d FrameTF::mapBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												  const Eigen::Vector3d& rpy) const
{
	return fromHorizontalToBaseFrame(vec_B, rpy);
}


Eigen::Vector3d FrameTF::mapBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												  const Eigen::Quaterniond& q) const
{
	return fromHorizontalToBaseFrame(vec_B, q);
}


Eigen::Vector3d FrameTF::mapHorizontalBaseFrame(const Eigen::Vector3d& vec_H,
												const Eigen::Vector3d& rpy) const
{
	return fromBaseToHorizontalFrame(vec_H, rpy);
}


Eigen::Vector3d FrameTF::mapHorizontalBaseFrame(const Eigen::Vector3d& vec_H,
												const Eigen::Quaterniond& rpy) const
{
	return fromBaseToHorizontalFrame(vec_H, rpy);
}

} //@namespace math
} //@namespace dwl
