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

/**
 * @brief Converts coordinated veloticy to spatial velocity
 * @param rbd::Vector6d& Velocity
 * @param const Eigen::Vector3d& Point
 * @return rbd::Vector6d Spatial velocity
 */
Vector6d convertVelocityToSpatialVelocity(Vector6d& velocity,
											  const Eigen::Vector3d& point);

/**
 * @brief Converts coordinated force to spatial force
 * @param rbd::Vector6d& Force
 * @param const Eigen::Vector3d& Point
 * @return rbd::Vector6d Spatial force
 */
Vector6d convertForceToSpatialForce(Vector6d& force,
									    const Eigen::Vector3d& point);



/**
 * @brief Computes the Jacobian in certain point of a specific body
 * @param const RigidBodyDynamics::Model& Model of the rigid-body system
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint position
 * @param unsigned int Body id
 * @param const RigidBodyDynamics::Math::Vector3d& 3d Position of the point
 * @param RigidBodyDynamics::Math::MatrixNd& Jacobian
 * @param bool Update kinematic state
 */
void computePointJacobian(RigidBodyDynamics::Model& model,
						  const RigidBodyDynamics::Math::VectorNd &Q,
						  unsigned int body_id,
						  const RigidBodyDynamics::Math::Vector3d& point_position,
						  RigidBodyDynamics::Math::MatrixNd& jacobian,
						  bool update_kinematics);

/**
 * @brief Computes the velocity in certain point of a specific body
 * @param RigidBodyDynamics::Model& Model of the rigid-body system
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint position
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint velocity
 * @param unsigned int Body id
 * @param const Eigen::Vector3d& 3d Position of the point
 * @param bool Update kinematic state
 */
rbd::Vector6d computePointVelocity(RigidBodyDynamics::Model& model,
								   const RigidBodyDynamics::Math::VectorNd& Q,
								   const RigidBodyDynamics::Math::VectorNd& QDot,
								   unsigned int body_id,
								   const RigidBodyDynamics::Math::Vector3d point_position,
								   bool update_kinematics);

/**
 * @brief Computes the acceleration in certain point of a specific body
 * @param RigidBodyDynamics::Model& Model of the rigid-body system
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint position
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint velocity
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint acceleration
 * @param unsigned int Body id
 * @param const Eigen::Vector3d& 3d Position of the point
 * @param bool Update kinematic state
 */
rbd::Vector6d computePointAcceleration(RigidBodyDynamics::Model& model,
									   const RigidBodyDynamics::Math::VectorNd& Q,
									   const RigidBodyDynamics::Math::VectorNd& QDot,
									   const RigidBodyDynamics::Math::VectorNd& QDDot,
									   unsigned int body_id,
									   const Eigen::Vector3d point_position,
									   bool update_kinematics);


void FloatingBaseInverseDynamics(RigidBodyDynamics::Model& model,
								 const RigidBodyDynamics::Math::VectorNd &Q,
								 const RigidBodyDynamics::Math::VectorNd &QDot,
								 const RigidBodyDynamics::Math::VectorNd &QDDot,
								 RigidBodyDynamics::Math::SpatialVector& base_acc,
								 RigidBodyDynamics::Math::VectorNd &Tau,
								 std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext = NULL);


} //@namespace rbd
} //@namespace dwl

#endif
