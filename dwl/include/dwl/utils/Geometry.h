#ifndef DWL__MATH__GEOMETRY__H
#define DWL__MATH__GEOMETRY__H

#include <dwl/utils/Macros.h>
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


/** p*x + q*y + r = 0 */
struct LineCoeff2d
{
	double p;
	double q;
	double r;
};

enum Coords3d {X = 0, Y, Z};

/**
 * @brief Normalizes the angles according to an angle representation
 * @param double& Angle
 * @param AngleRepresentation Angle representation could be ZeroTo2Pi or MinusPiToPi
 */
void normalizeAngle(double& angle,
					AngleRepresentation angle_notation);

/**
 * @brief Computes the inradii of a triangle
 * @param double& Inscribed radii of the triangle
 * @param double Size a
 * @param double Size b
 * @param double Size c
 */
void inRadiiTriangle(double& inradii,
					 double size_a,
					 double size_b,
					 double size_c);

/**
 * @brief Computes the plane parameters (normal vector of the plane)
 * @param Eigen::Vector3d& Parameters or normal vector of the plane
 * @param const std::vector<Eigen::Vector3f> Points of the cloud
 */
void computePlaneParameters(Eigen::Vector3d& normal,
							const std::vector<Eigen::Vector3f>& points);

/**
 * @brief Computes the mean and covariance of the cloud of 3D points
 * @param Eigen::Vector3d& Mean vector
 * @param Eigen::Matrix3d& Matrix of covariance
 * @param const std::vector<Eigen::Vector3f> Cloud of 3D points
 * @return The size of the cloud
 */
unsigned int computeMeanAndCovarianceMatrix(Eigen::Vector3d& mean,
											Eigen::Matrix3d& covariance_matrix,
											const std::vector<Eigen::Vector3f>& cloud);

/**
 * @brief Solves the plane parameters
 * @param Eigen::Vector3d& Normal vector of the plane
 * @param double& Curvature of the plane
 * @param Eigen::Matrix3d Matrix of covariance
 */
void solvePlaneParameters(Eigen::Vector3d& normal_vector,
						  double& curvature,
						  const Eigen::Matrix3d covariance_matrix);

/**
 * @brief Computes the roots of a third order equation
 * @param Eigen::Vector3d& Roots
 * @param Eigen::Matrix3d& Matrix
 */
void computeRoots(Eigen::Vector3d& roots,
				  const Eigen::Matrix3d& m);

/**
 * @brief Complementary function of the computeRoots method
 * @param Eigen::Vector3d& roots
 * @param const Eigen::Matrix3d::Scalar& b
 * @param const Eigen::Matrix3d::Scalar& c
 */
void computeRoots2(Eigen::Vector3d& roots,
				   const Eigen::Matrix3d::Scalar& b,
				   const Eigen::Matrix3d::Scalar& c);

/**
 * @brief Checks if p2 is on the right side of line from p0 to p1
 * @return: >0 for p2 right of the line through p0 and p1
 * 			=0 for p2 on the line
 * 			<0 for p2 left of the line
 */
double isRight(const Eigen::Vector3d p0,
			   const Eigen::Vector3d p1,
			   const Eigen::Vector3d p2);

/**
 * @brief Performs a clockwise radial sort of the input points starting
 * with the bottom left point
 * @param[out] p Unsorted points or footholds
 * Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points
 * Fails when 3 points are on same line and one could be removed
 */
void clockwiseSort(std::vector<Eigen::Vector3d>& p);

/**
 * @brief Performs a  counter clockwise radial sort of the input points
 * starting with the bottom left point
 * @param[out] p Unsorted points or footholds
 * Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points
 * Fails when 3 points are on same line and one could be removed
 */
void counterClockwiseSort(std::vector<Eigen::Vector3d>& p);

/**
 * @brief Gets the line coefficient, i.e. p*x + q*y + r = 0
 * @param const Eigen::Vector3d First vertex
 * @param const Eigen::Vector3d Second vertex
 * @param bool Normalized coefficients
 * @return The line coefficients
 */
LineCoeff2d lineCoeff(const Eigen::Vector3d& pt0,
					  const Eigen::Vector3d& pt1,
					  bool normalize = true);

} //@namespace math
} //@namespace dwl


#endif
