#ifndef DWL__RBD__RIGID_BODY_DYNAMICS__H
#define DWL__RBD__RIGID_BODY_DYNAMICS__H

#include <rbdl/rbdl.h>
#include <utils/Math.h>


namespace dwl
{

namespace rbd
{


enum Component {Linear, Angular, Full};

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Block<Vector6d,3,1> Part3d;///< a 3D sub-vector of a 6D vector
typedef Eigen::Block<Eigen::Matrix4d,3,1> TranslationPart;
typedef Eigen::Block<Eigen::Matrix4d,3,3> RotationPart;
typedef std::vector<std::string> BodySelector;
typedef std::map<std::string,unsigned int> BodyID;
typedef std::map<std::string,Eigen::Vector3d> BodyPosition;
typedef std::map<std::string,Eigen::VectorXd> BodyVector;
typedef std::map<std::string,Vector6d> BodyWrench;

/**
 * @brief Vector coordinates
 * Constants to index either 6d or 3d coordinate vectors.
 */
enum Coords3d {X = 0, Y, Z};
enum Coords6d {AX = 0, AY, AZ, LX, LY, LZ};

/**
 * @brief The 3-coordinate vector with the angular components (angular velocity or torque) of the
 * given spatial vector
 */
Part3d angularPart(Vector6d& vector);

/**
 * @brief The 3-coordinate vector with the linear components (linear
 * velocity or force) of the given spatial vector.
 */
Part3d linearPart(Vector6d& vector);

/** @brief The translation vector of a homogeneous transform */
TranslationPart translationVector(Eigen::Matrix4d& hom_transform);

/** @brief The rotation matrix of a homogeneous transform */
RotationPart rotationMatrix(Eigen::MatrixBase<Eigen::Matrix4d>& hom_transform);

/**
 * @brief Gets list of bodies (movable and fixed) of the rigid-body system
 * @param Body& Body ids
 * @param const RigidBodyDynamics::Model& Model of the rigid-body system
 */
void getListOfBodies(BodyID& list_body_id,
					 const RigidBodyDynamics::Model& model);

/** @brief Print the model information */
void printModelInfo(const RigidBodyDynamics::Model& model);

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

} //@namespace rbd
} //@namespace dwl

#endif
