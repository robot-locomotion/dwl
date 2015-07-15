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
 * @brief The 3-coordinate vector with the angular components (angular velocity or torque) of the given
 * spatial vector
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
 * @brief Vector coordinates
 * Constants to index either 6d or 3d coordinate vectors.
 */
enum Coords3d {X = 0, Y, Z};
enum Coords6d {AX = 0, AY, AZ, LX, LY, LZ };
enum TypeOfSystem {FixedBase, FloatingBase, ConstrainedFloatingBase, VirtualFloatingBase};

/** @brief Defines a floating base joint status */
struct FloatingBaseJoint {
	FloatingBaseJoint(bool status) : active(status), id(0) {}
	bool active;
	unsigned id;
};

/** @brief Defines a reduced floating-base system */
struct ReducedFloatingBase {
	ReducedFloatingBase(bool full_floating_base = false) : AX(full_floating_base), AY(full_floating_base),
			AZ(full_floating_base), LX(full_floating_base), LY(full_floating_base), LZ(full_floating_base) {}
	bool isFullyFree() {
		if (!AX.active && !AY.active && !AZ.active && !LX.active && !LY.active && !LZ.active)
			return true;
		else
			return false;
	}
	unsigned int getFloatingBaseDOF() {
		return AX.active + AY.active + AZ.active + LX.active + LY.active + LZ.active;
	}
	unsigned int getJoint(unsigned int id) {
		if (AX.active && AX.id == id)
			return rbd::AX;
		else if (AY.active && AY.id == id)
			return rbd::AY;
		else if (AZ.active && AZ.id == id)
			return rbd::AZ;
		else if (LX.active && LX.id == id)
			return rbd::LX;
		else if (LY.active && LY.id == id)
			return rbd::LY;
		else if (LZ.active && LZ.id == id)
			return rbd::LZ;
		else {
			printf(RED "ERROR: the %i id doesn't bellow to floating-base joint\n" COLOR_RESET, id);
			return 0;
		}
	}
	FloatingBaseJoint AX;
	FloatingBaseJoint AY;
	FloatingBaseJoint AZ;
	FloatingBaseJoint LX;
	FloatingBaseJoint LY;
	FloatingBaseJoint LZ;
};

/** @brief Returns true if it's a floating-base robot */
bool isFloatingBaseRobot(const RigidBodyDynamics::Model& model);

/** @brief Returns true if it's a constrained floating-base robot */
bool isConstrainedFloatingBaseRobot(struct ReducedFloatingBase* reduced_base);

/** @brief Returns true if it's a virtual floating-base robot */
bool isVirtualFloatingBaseRobot(struct ReducedFloatingBase* reduced_base);

/** @brief Returns the number of dof of the floating base */
unsigned int getFloatingBaseDOF(const RigidBodyDynamics::Model& model,
								struct ReducedFloatingBase* reduced_base = NULL);

/**
 * @brief Gets the type of rigid body dynamic system, i.e fixed-base, floating-base, constrained floating-base or
 * virtual floating-base
 * @param TypeOfSystem Type of rigid body dynamic system
 * @param const RigidBodyDynamics::Model& Model of the rigid-body system
 * @param struct ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
 * base with physical constraints
 */
void getTypeOfDynamicSystem(TypeOfSystem& type_of_system,
							const RigidBodyDynamics::Model& model,
							struct ReducedFloatingBase* reduced_base = NULL);

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
 * @brief Converts the base and joint states to a generalized joint state
 * @param const Vector6d& Base state
 * @param const Eigen::VectorXd& Joint state
 * @param enum TypeOfSystem Defines the type of system, e.g. fixed or floating- base system
 * @param struct ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
 * base with physical constraints
 * @return Eigen::VectorXd Generalized joint state
 */
Eigen::VectorXd toGeneralizedJointState(const Vector6d& base_state,
										const Eigen::VectorXd& joint_state,
										enum TypeOfSystem type_of_system,
										struct ReducedFloatingBase* reduced_base = NULL);

/**
 * @brief Converts the generalized joint state to base and joint states
 * @param Vector6d& Base state
 * @param Eigen::VectorXd& Joint state
 * @param const Eigen::VectorXd Generalized joint state
 * @param enum TypeOfSystem Defines the type of system, e.g. fixed or floating- base system
 * @param struct ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
 * base with physical constraints
 */
void fromGeneralizedJointState(Vector6d& base_state,
							   Eigen::VectorXd& joint_state,
							   const Eigen::VectorXd& generalized_state,
							   enum TypeOfSystem type_of_system,
							   struct ReducedFloatingBase* reduced_base = NULL);

/**
 * @brief Sets the joint state given a branch values
 * @param Eigen::VectorXd& Joint state vector
 * @param cons Eigen::VectorXd& Branch state
 * @param unsigned int Body id
 * @param RigidBodyDynamics::Model& Rigid-body dynamic model
 * @param ReducedFloatingBase* Reduced floating-base model
 */
void setBranchState(Eigen::VectorXd& new_joint_state,
					const Eigen::VectorXd& branch_state,
					unsigned int body_id,
					RigidBodyDynamics::Model& model,
					struct ReducedFloatingBase* reduced_base = NULL);

Eigen::VectorXd getBranchState(Eigen::VectorXd& joint_state,
							   unsigned int body_id,
							   RigidBodyDynamics::Model& model,
							   struct ReducedFloatingBase* reduced_base = NULL);

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
