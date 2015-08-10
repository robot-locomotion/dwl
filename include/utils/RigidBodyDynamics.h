#ifndef DWL__RBD__RIGID_BODY_DYNAMICS__H
#define DWL__RBD__RIGID_BODY_DYNAMICS__H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <utils/URDF.h>
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
 * @brief Vector coordinates
 * Constants to index either 6d or 3d coordinate vectors.
 */
enum Coords3d {X = 0, Y, Z};
enum Coords6d {AX = 0, AY, AZ, LX, LY, LZ };
enum TypeOfSystem {FixedBase, FloatingBase, ConstrainedFloatingBase, VirtualFloatingBase};

/** @brief Defines a floating-base joint information */
struct FloatingBaseJoint {
	FloatingBaseJoint(bool status) : active(status), id(0) {}
	FloatingBaseJoint(bool status,
					  unsigned int _id,
					  std::string _name) : active(status), id(_id), name(_name) {}
	bool active;
	unsigned int id;
	std::string name;
};

/** @brief Defines an actuated joint information */
struct Joint {
	Joint(unsigned int _id,
		  std::string _name) : id(_id), name(_name) {}
	unsigned int id;
	std::string name;
};

/** @brief Defines a floating-base system */
struct FloatingBaseSystem {
	FloatingBaseSystem(bool full = false, unsigned int _num_joints = 0) : num_system_joints(0),
			num_floating_joints(6 * full), num_joints(_num_joints),
			AX(full), AY(full), AZ(full), LX(full), LY(full), LZ(full),
			AX_constraint(false), AY_constraint(false), AZ_constraint(false),
			LX_constraint(false), LY_constraint(false), LZ_constraint(false),
			type_of_system(rbd::FixedBase), num_end_effectors(0) {}

	/** @brief Resets the system information from URDF description */
	void reset(std::string urdf_model)
	{
		// Getting the RBDL model from URDF model
		RigidBodyDynamics::Addons::URDFReadFromString(urdf_model.c_str(), &rbd_model, false);

		// Getting information about the floating-base joints
		urdf_model::JointID floating_joint_names;
		urdf_model::getJointNames(floating_joint_names, urdf_model, urdf_model::floating);
		num_floating_joints = floating_joint_names.size();

		urdf_model::JointID floating_joint_motions;
		if (num_floating_joints > 0) {
			urdf_model::getFloatingBaseJointMotion(floating_joint_motions, urdf_model);
			for (urdf_model::JointID::iterator jnt_it = floating_joint_motions.begin();
					jnt_it != floating_joint_motions.end(); jnt_it++) {
				std::string joint_name = jnt_it->first;
				unsigned int joint_motion = jnt_it->second;
				unsigned int joint_id = floating_joint_names.find(joint_name)->second;

				// Setting the floating joint information
				rbd::FloatingBaseJoint joint(true, joint_id, joint_name);
				setFloatingBaseJoint(joint, joint_motion);
			}
		}

		// Getting the information about the actuated joints
		urdf_model::JointID free_joint_names;
		urdf_model::getJointNames(free_joint_names, urdf_model, urdf_model::free);
		unsigned int num_free_joints = free_joint_names.size();
		num_joints = num_free_joints - num_floating_joints;
		for (urdf_model::JointID::iterator jnt_it = free_joint_names.begin();
				jnt_it != free_joint_names.end(); jnt_it++) {
			std::string joint_name = jnt_it->first;
			unsigned int joint_id = jnt_it->second;

			// Checking if it's a virtual floating-base joint
			if (num_floating_joints > 0) {
				if (floating_joint_names.find(joint_name) == floating_joint_names.end()) {
					// Setting the actuated joint information
					Joint joint(joint_id, joint_name);
					setJoint(joint);
				}
			}
		}

		// Getting the floating-base system information
		num_system_joints = num_floating_joints + num_joints;
		if (isFullyFloatingBase()) {
			if (hasFloatingBaseConstraints())
				type_of_system = ConstrainedFloatingBase;
			else
				type_of_system = FloatingBase;
		} else if (num_floating_joints > 0)
			type_of_system = VirtualFloatingBase;
		else
			type_of_system = FixedBase;

		// Getting the end-effectors information
		urdf_model::getEndEffectors(end_effectors, urdf_model);
		num_end_effectors = end_effectors.size();
	}

	/** @brief Sets the floating-base joint information */
	void setFloatingBaseJoint(const FloatingBaseJoint& joint, unsigned int joint_id)
	{
		if (joint_id == urdf_model::AX)
			AX = joint;
		else if (joint_id == urdf_model::AY)
			AY = joint;
		else if (joint_id == urdf_model::AZ)
			AZ = joint;
		else if (joint_id == urdf_model::LX)
			LX = joint;
		else if (joint_id == urdf_model::LY)
			LY = joint;
		else if (joint_id == urdf_model::LZ)
			LZ = joint;
		else if (joint_id == urdf_model::FULL) {
			AX = joint;
			AY = joint;
			AZ = joint;
			LX = joint;
			LY = joint;
			LZ = joint;
		} else {
			printf(RED "FATAL: the floating-joint id (%i) is not consistent with the standard\n"
					COLOR_RESET, joint_id);
			exit(EXIT_FAILURE);
		}
	}

	/** @brief Sets the actuated joint information */
	void setJoint(const Joint& joint)
	{
		joints[joint.name] = joint.id;
	}

	/** @brief Sets the floating-base constraint information */
	void setFloatingBaseConstraint(unsigned int joint_id)
	{
		if (joint_id == urdf_model::AX)
			AX_constraint = true;
		else if (joint_id == urdf_model::AY)
			AY_constraint = true;
		else if (joint_id == urdf_model::AZ)
			AZ_constraint = true;
		else if (joint_id == urdf_model::LX)
			LX_constraint = true;
		else if (joint_id == urdf_model::LY)
			LY_constraint = true;
		else if (joint_id == urdf_model::LZ)
			LZ_constraint = true;
		else if (joint_id == urdf_model::FULL) {
			AX_constraint = true;
			AY_constraint = true;
			AZ_constraint = true;
			LX_constraint = true;
			LY_constraint = true;
			LZ_constraint = true;
			printf(YELLOW "WARNING: it is fully constrained floating-base system\n" COLOR_RESET);
		} else {
			printf(RED "FATAL: the floating-joint id (%i) is not consistent with the standard\n"
					COLOR_RESET, joint_id);
			exit(EXIT_FAILURE);
		}
	}

	/** @brief Sets the type of floating-base system */
	void setTypeOfDynamicSystem(enum TypeOfSystem _type_of_system)
	{
		type_of_system = _type_of_system;
	}

	/** @brief Sets the actuated joint DoF */
	void setJointDoF(unsigned int _num_joints)
	{
		num_joints = _num_joints;
	}

	/** @brief Gets the floating-base system DoF */
	const unsigned int& getSystemDoF()
	{
		return num_system_joints;
	}

	/** @brief Gets the floating-base DoF */
	const unsigned int& getFloatingBaseDoF()
	{
		return num_floating_joints;
	}

	/** @brief Gets the actuated joint DoF */
	const unsigned int& getJointDoF()
	{
		return num_joints;
	}

	/** @brief Gets the floating-base joint */
	const FloatingBaseJoint& getFloatingBaseJoint(Coords6d joint)
	{
		if (joint == rbd::AX)
			return AX;
		else if (joint == rbd::AY)
			return AY;
		else if (joint == rbd::AZ)
			return AZ;
		else if (joint == rbd::LX)
			return LX;
		else if (joint == rbd::LY)
			return LY;
		else
			return LZ;
	}

	/** @brief Gets the floating-base joint given an Id */
	unsigned int getFloatingBaseJoint(unsigned int id)
	{
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

	/** @brief Gets actuated joint information */
	const urdf_model::JointID& getJoints()
	{
		return joints;
	}

	/** @brief Gets the type of floating-base system */
	enum TypeOfSystem getTypeOfDynamicSystem()
	{
		return type_of_system;
	}

	/** @brief Gets the number of end-effectors */
	const unsigned int& getNumberOfEndEffectors()
	{
		return num_end_effectors;
	}

	/** @brief Gets the end-effectors names */
	void getEndEffectors(urdf_model::LinkSelector& _end_effectors)
	{
		_end_effectors = end_effectors;
	}

	bool isFullyFloatingBase()
	{
		if (AX.active && AY.active && AZ.active && LX.active && LY.active && LZ.active)
			return true;
		else
			return false;
	}


	bool isVirtualFloatingBaseRobot()
	{
		if (type_of_system == VirtualFloatingBase)
			return true;
		else
			return false;
	}

	bool isConstrainedFloatingBaseRobot()
	{
		if (type_of_system == ConstrainedFloatingBase)
			return true;
		else
			return false;
	}

	bool hasFloatingBaseConstraints()
	{
		if (AX_constraint || AY_constraint || AZ_constraint ||
				LX_constraint || LY_constraint || LZ_constraint)
			return true;
		else
			return false;
	}

	/** @brief Rigid-body dynamic model */
	RigidBodyDynamics::Model rbd_model;

	/** @brief Number of DoFs */
	unsigned int num_system_joints;
	unsigned int num_floating_joints;
	unsigned int num_joints;

	/** @brief System joints */
	FloatingBaseJoint AX;
	FloatingBaseJoint AY;
	FloatingBaseJoint AZ;
	FloatingBaseJoint LX;
	FloatingBaseJoint LY;
	FloatingBaseJoint LZ;
	urdf_model::JointID joints;

	/** @brief Floating-base constraints */
	bool AX_constraint;
	bool AY_constraint;
	bool AZ_constraint;
	bool LX_constraint;
	bool LY_constraint;
	bool LZ_constraint;

	/** @brief Type of system */
	enum TypeOfSystem type_of_system;

	/** @brief End-effector information */
	urdf_model::LinkSelector end_effectors;
	unsigned int num_end_effectors;
};

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
 * @param struct FloatingBaseSystem& Defines the general properties of a floating-base
 * system
 * @return Eigen::VectorXd Generalized joint state
 */
Eigen::VectorXd toGeneralizedJointState(const Vector6d& base_state,
										const Eigen::VectorXd& joint_state,
										struct FloatingBaseSystem& system);

/**
 * @brief Converts the generalized joint state to base and joint states
 * @param Vector6d& Base state
 * @param Eigen::VectorXd& Joint state
 * @param const Eigen::VectorXd Generalized joint state
 * @param struct FloatingBaseSystem& Defines the general properties of a floating-base
 * system
 */
void fromGeneralizedJointState(Vector6d& base_state,
							   Eigen::VectorXd& joint_state,
							   const Eigen::VectorXd& generalized_state,
							   struct FloatingBaseSystem& system);

/**
 * @brief Sets the joint state given a branch values
 * @param Eigen::VectorXd& Joint state vector
 * @param cons Eigen::VectorXd& Branch state
 * @param unsigned int Body id
 * @param struct FloatingBaseSystem& Defines the general properties of a floating-base
 * system
 */
void setBranchState(Eigen::VectorXd& new_joint_state,
					const Eigen::VectorXd& branch_state,
					unsigned int body_id,
					struct FloatingBaseSystem& system);

/**
 * @brief Gets the branch values given a joint state
 * @param Eigen::VectorXd& Joint state vector
 * @param cons Eigen::VectorXd& Branch state
 * @param unsigned int Body id
 * @param struct FloatingBaseSystem& Defines the general properties of a floating-base
 * system
 */
Eigen::VectorXd getBranchState(Eigen::VectorXd& joint_state,
							   unsigned int body_id,
							   struct FloatingBaseSystem& system);

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
