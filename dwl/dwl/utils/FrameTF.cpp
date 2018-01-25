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
	return getWorldToBaseRotation(rpy) * vec_W;
}


Eigen::Vector3d FrameTF::fromWorldToBaseFrame(const Eigen::Vector3d& vec_W,
											  const Eigen::Quaterniond& q) const
{
	return getWorldToBaseRotation(q) * vec_W;
}


Eigen::Matrix3d FrameTF::getWorldToBaseRotation(const Eigen::Vector3d& rpy) const
{
	return math::getQuaternion(rpy).inverse().toRotationMatrix();
}


Eigen::Matrix3d FrameTF::getWorldToBaseRotation(const Eigen::Quaterniond& q) const
{
	return q.inverse().toRotationMatrix();
}


Eigen::Vector3d FrameTF::fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
													const Eigen::Vector3d& rpy) const
{
	return getWorldToHorizontalRotation(rpy) * vec_W;
}


Eigen::Vector3d FrameTF::fromWorldToHorizontalFrame(const Eigen::Vector3d& vec_W,
													const Eigen::Quaterniond& q) const
{
	return getWorldToHorizontalRotation(q) * vec_W;
}


Eigen::Matrix3d FrameTF::getWorldToHorizontalRotation(const Eigen::Vector3d& rpy) const
{
	// Note that the rotation matrix is an orthogonal matrix, that is the
	// inverse can computed as transpose. This improve the computation time
	return getRotHorizontalToWorld(rpy).transpose();
}


Eigen::Matrix3d FrameTF::getWorldToHorizontalRotation(const Eigen::Quaterniond& q) const
{
	// Note that the rotation matrix is an orthogonal matrix, that is the
	// inverse can computed as transpose. This improve the computation time
	return getRotHorizontalToWorld(math::getRPY(q)).transpose();
}


Eigen::Vector3d FrameTF::fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											  const Eigen::Vector3d& rpy) const
{
	return getBaseToWorldRotation(rpy) * vec_B;
}


Eigen::Vector3d FrameTF::fromBaseToWorldFrame(const Eigen::Vector3d& vec_B,
											  const Eigen::Quaterniond& q) const
{
	return getBaseToWorldRotation(q) * vec_B;
}


Eigen::Matrix3d FrameTF::getBaseToWorldRotation(const Eigen::Vector3d& rpy) const
{
	return math::getQuaternion(rpy).toRotationMatrix();
}


Eigen::Matrix3d FrameTF::getBaseToWorldRotation(const Eigen::Quaterniond& q) const
{
	return q.toRotationMatrix();
}


Eigen::Vector3d FrameTF::fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												   const Eigen::Vector3d& rpy) const
{
	return getBaseToHorizontalRotation(rpy) * vec_B;
}


Eigen::Vector3d FrameTF::fromBaseToHorizontalFrame(const Eigen::Vector3d& vec_B,
												   const Eigen::Quaterniond& q) const
{
	return getBaseToHorizontalRotation(q) * vec_B;
}


Eigen::Matrix3d FrameTF::getBaseToHorizontalRotation(const Eigen::Vector3d& rpy) const
{
	return getRotBaseToHorizontal(rpy);
}


Eigen::Matrix3d FrameTF::getBaseToHorizontalRotation(const Eigen::Quaterniond& q) const
{
	return getRotBaseToHorizontal(math::getRPY(q));
}


Eigen::Vector3d FrameTF::fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
													const Eigen::Vector3d& rpy) const
{
	return getHorizontalToWorldRotation(rpy) * vec_H;
}


Eigen::Vector3d FrameTF::fromHorizontalToWorldFrame(const Eigen::Vector3d& vec_H,
													const Eigen::Quaterniond& q) const
{
	return getHorizontalToWorldRotation(q) * vec_H;
}


Eigen::Matrix3d FrameTF::getHorizontalToWorldRotation(const Eigen::Vector3d& rpy) const
{
	return getRotHorizontalToWorld(rpy);
}


Eigen::Matrix3d FrameTF::getHorizontalToWorldRotation(const Eigen::Quaterniond& q) const
{
	return getRotHorizontalToWorld(math::getRPY(q));
}


Eigen::Vector3d FrameTF::fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Vector3d& rpy) const
{
	return getHorizontalToBaseRotation(rpy) * vec_H;
}


Eigen::Vector3d FrameTF::fromHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												   const Eigen::Quaterniond& q) const
{
	return getHorizontalToBaseRotation(q) * vec_H;
}


Eigen::Matrix3d FrameTF::getHorizontalToBaseRotation(const Eigen::Vector3d& rpy) const
{
	// Note that the rotation matrix is an orthogonal matrix, that is the
	// inverse can computed as transpose. This improve the computation time
	return getRotBaseToHorizontal(rpy).transpose();
}

Eigen::Matrix3d FrameTF::getHorizontalToBaseRotation(const Eigen::Quaterniond& q) const
{
	// Note that the rotation matrix is an orthogonal matrix, that is the
	// inverse can computed as transpose. This improve the computation time
	return getRotBaseToHorizontal(math::getRPY(q)).transpose();
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


Eigen::Vector3d FrameTF::mapHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Vector3d& rpy) const
{
	return fromBaseToHorizontalFrame(vec_H, rpy);
}


Eigen::Vector3d FrameTF::mapHorizontalToBaseFrame(const Eigen::Vector3d& vec_H,
												  const Eigen::Quaterniond& rpy) const
{
	return fromBaseToHorizontalFrame(vec_H, rpy);
}

} //@namespace math
} //@namespace dwl
