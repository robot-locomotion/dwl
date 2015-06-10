#ifndef DWL_RigidBodyDynamics_H
#define DWL_RigidBodyDynamics_H

#include <rbdl/rbdl.h>
#include <utils/Math.h>


namespace dwl
{

namespace rbd
{

enum Component {Linear, Angular, Full};

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Block<Vector6d,3,1> Part3d;///< a 3D subvector of a 6D vector
typedef std::map<std::string,bool> EndEffectorSelector;
typedef std::map<std::string,unsigned int> EndEffectorID;
typedef std::map<std::string,Eigen::Vector3d> EndEffectorPosition;
typedef std::map<std::string,Vector6d> EndEffectorForce;

/**
 * @brief The 3-coordinate vector with the angular components (angular velocity or torque) of the given
 * 6d vector
 */
Part3d angularPart(Vector6d& vector);

/**
 * @brief The 3-coordinate vector with the linear components (linear
 * velocity or force) of the given 6D vector.
 */
Part3d linearPart(Vector6d& vector);

/**
 * @brief Vector coordinates
 * Constants to index either 6d or 3d coordinate vectors.
 */
enum Coords3d {X = 0, Y, Z};
enum Coords6d {AX = 0, AY, AZ, LX, LY, LZ };

/** @brief Returns true if it's a floating-base robot */
bool isFloatingBaseRobot(const RigidBodyDynamics::Model& model);

/**
 * @brief Converts the base and joint states to a generalized joint state
 * @param const Vector6d& Base state
 * @param const Eigen::VectorXd& Joint state
 * @return Eigen::VectorXd Generalized joint state
 */
Eigen::VectorXd toGeneralizedJointState(const RigidBodyDynamics::Model& model,
										   const Vector6d& base_state,
										   const Eigen::VectorXd& joint_state);

/**
 * @brief Converts the generalized joint state to base and joint states
 * @param Vector6d& Base state
 * @param Eigen::VectorXd& Joint state
 * @return const Eigen::VectorXd Generalized joint state
 */
void fromGeneralizedJointState(const RigidBodyDynamics::Model& model,
								   Vector6d& base_state,
								   Eigen::VectorXd& joint_state,
								   const Eigen::VectorXd& generalized_state);

Vector6d convertVelocityToSpatialVelocity(Vector6d& velocity,
											  const Eigen::Vector3d& point);
Vector6d convertForceToSpatialForce(Vector6d& force,
									    const Eigen::Vector3d& point);

} //@namespace rbd
} //@namespace dwl

#endif
