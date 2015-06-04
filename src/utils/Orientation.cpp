#include <utils/Orientation.h>


namespace dwl
{

namespace math
{

Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation_mtx)
{
	Eigen::Vector3d rpy;
	rpy[0] = atan2f((float) rotation_mtx(2,1), (float) rotation_mtx(2,2));
	rpy[1] = asinf((float) -rotation_mtx(2,0));
	rpy[2] = atan2f((float) rotation_mtx(1,0), (float) rotation_mtx(0,0));

	return rpy;
}


Eigen::Vector3d getRPY(const Eigen::Quaterniond& quaternion)
{
	Eigen::Matrix3d rotation_mtx = quaternion.toRotationMatrix();

	return getRPY(rotation_mtx);
}


Eigen::Quaterniond getQuaternion(const Eigen::Matrix3d& rotation_mtx)
{
	Eigen::Quaterniond quaternion(rotation_mtx);
	return quaternion;
}


Eigen::Quaterniond getQuaternion(const Eigen::Vector3d& rpy)
{
	double roll = rpy[0];
	double pitch = rpy[1];
	double yaw = rpy[2];

	double w = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) +
			sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double x = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) -
			cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double y = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) +
			sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
	double z = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) -
			sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);

	Eigen::Quaterniond quaternion(w, x, y, z);
	return quaternion;
}


Eigen::Matrix3d getRotationMatrix(const Eigen::Quaterniond& quaternion)
{
	return quaternion.toRotationMatrix();
}


Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& rpy)
{
	Eigen::Quaterniond quaternion = getQuaternion(rpy);

	return getRotationMatrix(quaternion);
}


double getRoll(const Eigen::Vector3d& rpy)
{
	return rpy[0];
}


double getPitch(const Eigen::Vector3d& rpy)
{
	return rpy[1];
}


double getYaw(const Eigen::Vector3d& rpy)
{
	return rpy[2];
}

}
}
