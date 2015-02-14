#ifndef DWL_Rotation_H
#define DWL_Rotation_H

#include <Eigen/Dense>


namespace dwl
{

/**
 * @class Orientation
 * @brief Class for converting the orientation between roll, pitch and yaw angles, rotation matrix and quaternions
 */
class Orientation
{
	public:
		/**
		 * @brief Constructor function
		 * @param Eigen::Matrix3d Rotation matrix
		 */
		Orientation(Eigen::Matrix3d rotation_matrix);

		/**
		 * @brief Constructor function
		 * @param Eigen::Quaterniond Quaternion
		 */
		Orientation(Eigen::Quaterniond q);

		/**
		 * @brief Constructor function
		 * @param double Roll angle
		 * @param double Pitch angle
		 * @param double Yaw angle
		 */
		Orientation(double roll, double pitch, double yaw);

		/**
		 * @brief Constructor function
		 * @param double& X component of the vector
		 * @param double& Y component of the vector
		 * @param double& Z component of the vector
		 * @param double& W Scalar
		 */
		Orientation(double x, double y, double z, double w);

		/** @brief Destructor function */
		~Orientation();

		/**
		 * @brief Gets the roll, pitch and yaw angles
		 * @param double& Roll angle
		 * @param double& Pitch angle
		 * @param double& Yaw angle
		 */
		void getRPY(double& roll, double& pitch, double& yaw);

		/**
		 * @brief Gets quaternion, i.e. the scalar (w) and vector (x,y,z) components
		 * @param double& X component of the vector
		 * @param double& Y component of the vector
		 * @param double& Z component of the vector
		 * @param double& W Scalar
		 */
		void getQuaternion(double& x, double& y, double& z, double& w);

		/**
		 * @brief Gets the quaternion
		 * @param Eigen::Quaterniond& Quaternion
		 */
		void getQuaternion(Eigen::Quaterniond& q);

		/**
		 * @brief Gets the rotation matrix
		 * @param Eigen::Matrix3d& Rotation matrix
		 */
		void getRotationMatrix(Eigen::Matrix3d& rotation_matrix);


	private:
		/**
		 * @brief Returns the quaternion from roll, pitch and yaw angles
		 * @param double Roll angle
		 * @param double Pitch angle
		 * @param double Yaw angle
		 */
		Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw);

		/* * @brief Quaternion */
		Eigen::Quaterniond quaternion_;
};

}

#endif
