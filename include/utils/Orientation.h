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
		 * @param Eigen::Quaterniond q Quaternion
		 */
		Orientation(Eigen::Quaterniond q);

		/**
		 * @brief Constructor function
		 * @param double roll Roll angle
		 * @param double pitch Pitch angle
		 * @param double yaw Yaw angle
		 */
		Orientation(double roll, double pitch, double yaw);

		/**
		 * @brief Constructor function
		 * @param double& x X component of the vector
		 * @param double& y Y component of the vector
		 * @param double& z Z component of the vector
		 * @param double& w Scalar
		 */
		Orientation(double x, double y, double z, double w);

		/** @brief Destructor function */
		~Orientation();

		/**
		 * @brief Gets the roll, pitch and yaw angles
		 * @param double& roll Roll angle
		 * @param double& pitch Pitch angle
		 * @param double& yaw Yaw angle
		 */
		void getRPY(double& roll, double& pitch, double& yaw);

		/**
		 * @brief Gets quaternion, i.e. the scalar (w) and vector (x,y,z) components
		 * @param double& x X component of the vector
		 * @param double& y Y component of the vector
		 * @param double& z Z component of the vector
		 * @param double& w Scalar
		 */
		void getQuaternion(double& x, double& y, double& z, double& w);

		/**
		 * @brief Gets the quaternion
		 * @param Eigen::Quaterniond& q Quaternion
		 */
		void getQuaternion(Eigen::Quaterniond& q);

		/**
		 * @brief Gets the rotation matrix
		 * @param Eigen::Matrix3d& rotation_matrix
		 */
		void getRotationMatrix(Eigen::Matrix3d& rotation_matrix);


	private:
		/**
		 * @brief Returns the quaternion from roll, pitch and yaw angles
		 * @param double roll Roll angle
		 * @param pitch Pitch angle
		 * @param yaw Yaw angle
		 */
		Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw);

		/* * @brief Quaternion */
		Eigen::Quaterniond quaternion_;

};

}

#endif
