#include <utils/Orientation.h>


namespace dwl
{

Orientation::Orientation(Eigen::Matrix3d rotation_matrix)
{
	double roll = atan2f((float) rotation_matrix(2,1), (float) rotation_matrix(2,2));
	double pitch = asinf((float) -rotation_matrix(2,0));
	double yaw = atan2f((float) rotation_matrix(1,0), (float) rotation_matrix(0,0));

	quaternion_ = RPYToQuaternion(roll, pitch, yaw);
}


Orientation::Orientation(Eigen::Quaterniond quaternion)
{
	quaternion_ = quaternion;
}


Orientation::Orientation(double roll, double pitch, double yaw)
{
	quaternion_ = RPYToQuaternion(roll, pitch, yaw);
}


Orientation::Orientation(double x, double y, double z, double w)
{
	Eigen::Quaterniond quaternion(w, x, y, z);
	quaternion_ = quaternion;
}


Orientation::~Orientation()
{

}


void Orientation::getRPY(double& roll, double& pitch, double& yaw)
{
	Eigen::Matrix3d rotation_matrix = quaternion_.toRotationMatrix();

	roll = atan2f((float) rotation_matrix(2,1), (float) rotation_matrix(2,2));
	pitch = asinf((float) -rotation_matrix(2,0));
	yaw = atan2f((float) rotation_matrix(1,0), (float) rotation_matrix(0,0));
}


void Orientation::getQuaternion(Eigen::Quaterniond& quaternion)
{
	quaternion = quaternion_;
}


void Orientation::getRotationMatrix(Eigen::Matrix3d& rotation_matrix)
{
	rotation_matrix = quaternion_.toRotationMatrix();
}


Eigen::Quaterniond Orientation::RPYToQuaternion(double roll, double pitch, double yaw)
{
	double w = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double x = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) - cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
	double y = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
	double z = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) - sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);
	Eigen::Quaterniond quaternion(w, x, y, z);

	return quaternion;
}

}
