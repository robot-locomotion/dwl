#ifndef DWL__RBD__RIGID_BODY_DYNAMICS__H
#define DWL__RBD__RIGID_BODY_DYNAMICS__H

#include <dwl/utils/Math.h>


namespace dwl
{

namespace rbd
{


enum Component {Linear, Angular, Full};

typedef std::map<std::string,unsigned int> BodyID;

/**
 * @brief Vector coordinates
 * Constants to index either 6d or 3d coordinate vectors.
 */
enum Coords3d {X = 0, Y, Z};
std::string coord3dToName(enum Coords3d coord);
enum Coords6d {LX_V = 0, LY_V, LZ_V, AX_V, AY_V, AZ_V};
std::string coord6dToName(enum Coords6d coord);
enum Coords7d {LX_Q = 0, LY_Q, LZ_Q, AX_Q, AY_Q, AZ_Q, AW_Q};
std::string coord7dToName(enum Coords7d coord);

/**
 * @brief The 3-coordinate vector with the angular components of a
 * given configuration vector of SE(3)
 */
Eigen::Vector3d angularPart(Eigen::Vector6d& vector);

/**
 * @brief The 4-coordinate vector with the angular components of a
 * given tangent vector of SE(3)
 */
Eigen::Vector4d angularPart(Eigen::Vector7d& vector);

/**
 * @brief The 3-coordinate vector with the linear components of a
 * given configuration vector of SE(3).
 */
Eigen::Vector3d linearPart(Eigen::Vector6d& vector);

/**
 * @brief The 3-coordinate vector with the linear components of a
 * given configuration vector of SE(3).
 */
Eigen::Vector3d linearPart(Eigen::Vector7d& vector);


/** @brief The translation vector of a homogeneous transform */
Eigen::Vector3d translationVector(Eigen::Matrix4d& hom_transform);

/** @brief The rotation matrix of a homogeneous transform */
Eigen::Matrix3d rotationMatrix(Eigen::MatrixBase<Eigen::Matrix4d>& hom_transform);

} //@namespace rbd
} //@namespace dwl

#endif
