#include <dwl/utils/Orientation.h>


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
	return rpy(0);
}


double getPitch(const Eigen::Vector3d& rpy)
{
	return rpy(1);
}


double getYaw(const Eigen::Vector3d& rpy)
{
	return rpy(2);
}


Eigen::Matrix3d getInverseEulerAnglesRatesMatrix(const Eigen::Vector3d& rpy)
{
	double pitch = getPitch(rpy);
	double yaw = getYaw(rpy);

	// Computing the inverse of the Euler angle rates (EAR) matrix
	// Note that the EAR matrix is computed using the following equation:
	// E_ijk(φ,θ,ψ) := [R_k(ψ)^T R_j(θ)^T e_i, R_k(ψ)^T e_j, e_k]
	Eigen::Matrix3d InverseEAR;
	InverseEAR << cos(pitch) * cos(yaw), -sin(yaw), 0.,
				  cos(pitch) * sin(yaw),  cos(yaw), 0.,
							-sin(pitch), 		0., 1.;

	return InverseEAR;
}


Eigen::Matrix3d getInverseEulerAnglesRatesMatrix(const Eigen::Matrix3d& rotation_mtx)
{
	// Getting the rpy vector
	Eigen::Vector3d rpy = getRPY(rotation_mtx);

	// Getting the inverse of the Euler angle rates matrix
	return getInverseEulerAnglesRatesMatrix(rpy);
}


Eigen::Matrix3d getInverseEulerAnglesRatesMatrix(const Eigen::Quaterniond& quaternion)
{
	// Getting the rpy vector
	Eigen::Vector3d rpy = getRPY(quaternion);

	// Getting the inverse of the Euler angle rates matrix
	return getInverseEulerAnglesRatesMatrix(rpy);
}


Eigen::Matrix3d getEulerAnglesRatesMatrix(const Eigen::Vector3d& rpy)
{
	Eigen::Matrix3d EAR;
	double roll = getRoll(rpy);
	double pitch = getPitch(rpy);

	EAR <<  1.,         0.,            -sin(pitch),
			0.,  cos(roll), cos(pitch) * sin(roll),
			0., -sin(roll), cos(pitch) * cos(roll);

	return EAR;
}


Eigen::Matrix3d getEulerAnglesRatesMatrix(const Eigen::Matrix3d& rotation_mtx)
{
	// Getting the rpy vector
	Eigen::Vector3d rpy = getRPY(rotation_mtx);

	// Getting the Euler angle rates matrix
	return getEulerAnglesRatesMatrix(rpy);
}


Eigen::Matrix3d getEulerAnglesRatesMatrix(const Eigen::Quaterniond& quaternion)
{
	// Getting the rpy vector
	Eigen::Vector3d rpy = getRPY(quaternion);

	// Getting the Euler angle rates matrix
	return getEulerAnglesRatesMatrix(rpy);
}


Eigen::Matrix3d getInverseEulerAnglesRatesMatrix_dot(const Eigen::Vector3d& rpy,
													 const Eigen::Vector3d& rpyd)
{
	Eigen::Matrix3d EARInv_dot;
	double pitch = getPitch(rpy);
	double yaw = getYaw(rpy);
	double pitchd = getPitch(rpyd);
	double yawd = getYaw(rpyd);

	EARInv_dot <<  -cos(yaw) * sin(pitch) * pitchd - cos(pitch) * sin(yaw) * yawd, -cos(yaw) * yawd, 0.,
					cos(yaw) * cos(pitch) * yawd - sin(yaw) * sin(pitch) * pitchd, -sin(yaw) * yawd, 0.,
															 -cos(pitch) * pitchd,  			 0., 0.;

	return EARInv_dot;
}

}
}
