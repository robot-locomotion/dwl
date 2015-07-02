#ifndef DWL_Geometry_H
#define DWL_Geometry_H

#include <utils/macros.h>
#include <Eigen/Dense>
#include <vector>


namespace dwl
{

enum AngleRepresentation {ZeroTo2Pi, MinusPiToPi};

namespace math
{

/**
 * @brief Struct that defines the area
 */
struct Area
{
	Area() : min_x(0.), max_x(0.), min_y(0.), max_y(0.), min_z(0.), max_z(0.) {}
	Area(double x_min, double x_max,
		 double y_min, double y_max,
		 double z_min, double z_max) : min_x(x_min), max_x(x_max), min_y(y_min), max_y(y_max),
				 min_z(z_min), max_z(z_max) {}
	double min_x;
	double max_x;
	double min_y;
	double max_y;
	double min_z;
	double max_z;
};

/**
 * @brief Normalizes the angles according to an angle representation
 * @param double& Angle
 * @param AngleRepresentation Angle representation could be ZeroTo2Pi or MinusPiToPi
 */
void normalizeAngle(double& angle, AngleRepresentation angle_notation);

/**
 * @brief Computes the inradii of a triangle
 * @param double& Inscribed radii of the triangle
 * @param double Size a
 * @param double Size b
 * @param double Size c
 */
void inRadiiTriangle(double& inradii, double size_a, double size_b, double size_c);

/**
 * @brief Computes the plane parameters (normal vector of the plane)
 * @param Eigen::Vector3d& Parameters or normal vector of the plane
 * @param std::vector<Eigen::Vector3f> Points of the cloud
 */
void computePlaneParameters(Eigen::Vector3d& normal, std::vector<Eigen::Vector3f> points);

/**
 * @brief Computes the mean and covariance of the cloud of 3D points
 * @param std::vector<Eigen::Vector3f> Cloud of 3D points
 * @param Eigen::Matrix3d& Matrix of covariance
 * @param Eigen::Vector3d& Mean vector
 * @return The size of the cloud
 */
unsigned int computeMeanAndCovarianceMatrix(std::vector<Eigen::Vector3f> cloud,
											Eigen::Matrix3d& covariance_matrix, Eigen::Vector3d& mean);

/**
 * @brief Solves the plane parameters
 * @param Eigen::Vector3d& Normal vector of the plane
 * @param double& Curvature of the plane
 * @param Eigen::Matrix3d Matrix of covariance
 */
void solvePlaneParameters(Eigen::Vector3d& normal_vector, double& curvature, const Eigen::Matrix3d covariance_matrix);

/**
 * @brief Computes the roots of a third order equation
 * @param Eigen::Matrix3d& Matrix
 * @param Eigen::Vector3d& Roots
 */
void computeRoots(const Eigen::Matrix3d& m, Eigen::Vector3d& roots);

/**
 * @brief Complementary function of the computeRoots method
 * @param const Eigen::Matrix3d::Scalar& b
 * @param const Eigen::Matrix3d::Scalar& c
 * @param Eigen::Vector3d& roots
 */
void computeRoots2(const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c, Eigen::Vector3d& roots);

} //@namespace utils
} //@namespace dwl


#endif
