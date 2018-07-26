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


/**
 * @brief Converts an applied velocity acting at a certain point to spatial velocity
 * @param Vector6d& Velocity
 * @param const Eigen::Vector3d& Point
 * @return Vector6d Spatial velocity
 */
Vector6d convertPointVelocityToSpatialVelocity(Vector6d& velocity,
										  	   const Eigen::Vector3d& point);

/**
 * @brief Converts an applied force acting at a certain point to spatial force
 * @param Vector6d& Force
 * @param const Eigen::Vector3d& Point
 * @return Vector6d Spatial force
 */
Vector6d convertPointForceToSpatialForce(Vector6d& force,
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

/**
 * @brief Computes the floating-base inverse dynamics
 * @param RigidBodyDynamcis::Model& Model of the rigid-body system
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint position
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint velocity
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint acceleration
 * @param RigidBodyDynamics::Math::VectorNd& Joint forces
 * @param std::vector<RigidBodyDynamcis::Math::SpatialVector>* Applied external forces
 */
void FloatingBaseInverseDynamics(RigidBodyDynamics::Model& model,
								 const RigidBodyDynamics::Math::VectorNd &Q,
								 const RigidBodyDynamics::Math::VectorNd &QDot,
								 const RigidBodyDynamics::Math::VectorNd &QDDot,
								 RigidBodyDynamics::Math::SpatialVector& base_acc,
								 RigidBodyDynamics::Math::VectorNd &Tau,
								 std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext = NULL);

/**
 * @brief Computes the floating-base inverse dynamics
 * @param RigidBodyDynamcis::Model& Model of the rigid-body system
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint position
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint velocity
 * @param const RigidBodyDynamics::Math::VectorNd& Generalized joint acceleration
 * @param RigidBodyDynamics::Math::VectorNd& Joint forces
 * @param std::vector<RigidBodyDynamcis::Math::SpatialVector>* Applied external forces
 */
void FloatingBaseInverseDynamics(RigidBodyDynamics::Model& model,
								 unsigned int base_dof,
								 const RigidBodyDynamics::Math::VectorNd &Q,
								 const RigidBodyDynamics::Math::VectorNd &QDot,
								 const RigidBodyDynamics::Math::VectorNd &QDDot,
								 RigidBodyDynamics::Math::SpatialVector& base_acc,
								 RigidBodyDynamics::Math::VectorNd &Tau,
								 std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext = NULL);// TODO experimental

} //@namespace rbd
} //@namespace dwl

#endif
