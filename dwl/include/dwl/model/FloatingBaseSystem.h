#ifndef DWL__MODEL__FLOATING_BASE_SYSTEM__H
#define DWL__MODEL__FLOATING_BASE_SYSTEM__H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl/utils/URDF.h>
#include <dwl/utils/Math.h>
#include <dwl/utils/YamlWrapper.h>
#include <fstream>


namespace dwl
{

namespace model
{

enum TypeOfSystem {FixedBase, FloatingBase, ConstrainedFloatingBase, VirtualFloatingBase};

/** @brief Defines a floating-base joint information */
struct FloatingBaseJoint {
	FloatingBaseJoint(bool status) : active(status), constrained(false), id(0) {}
	FloatingBaseJoint(bool status,
					  unsigned int _id,
					  std::string _name) : active(status), constrained(false), id(_id), name(_name) {}
	bool active;
	bool constrained;
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

/** @brief Defines the type of end-effectors */
enum TypeOfEndEffector {ALL, FOOT};

/**
 * @class FloatingBaseSystem
 * @brief FloatingBaseSystem class read the floating-base system information from an URDF file.
 * Additionally, it has methods that allows us easily to manipulate joint states which depends on
 * the floating-base system kinematic-tree
 */
class FloatingBaseSystem
{
	public:
		/** @brief Constructor function */
		FloatingBaseSystem(bool full = false, unsigned int _num_joints = 0);

		/** @brief Destructor function */
		~FloatingBaseSystem();

		/**
		 * @brief Resets the system information from an URDF file
		 * @param const std::string& URDF filename
		 * @param const std::string& Semantic system description filename
		 */
		void resetFromURDFFile(const std::string& urdf_file,
							   const std::string& system_file = std::string());

		/**
		 * @brief Resets the system information from URDF model
		 * @param const std::string& URDF model
		 * @param const std::string& Semantic system description filename
		 */
		void resetFromURDFModel(const std::string& urdf_model,
								const std::string& system_file = std::string());

		/**
		 * @brief Resets the system semantic description from yaml file
		 * @param std::string Semantic system description filename
		 */
		void resetSystemDescription(std::string filename);

		/**
		 * @brief Sets the 6d floating-base joint information
		 * @param const FloatingBaseJoint& Floating-base joint information
		 **/
		void setFloatingBaseJoint(const FloatingBaseJoint& joint);

		/**
		 * @brief Sets the floating-base joint information
		 * @param const FloatingBaseJoint& Floating-base joint information
		 * @param rbd::Coords6d Joint coordinate
		 **/
		void setFloatingBaseJoint(const FloatingBaseJoint& joint,
								  rbd::Coords6d joint_coord);

		/**
		 * @brief Sets the actuated joint information
		 * @param const Joint& Joint information
		 */
		void setJoint(const Joint& joint);

		/**
		 * @brief Sets the floating-base constraint information
		 * @param rbd::Coords6d Joint coordinate
		 */
		void setFloatingBaseConstraint(rbd::Coords6d joint_id);

		/**
		 * @brief Sets the type of floating-base system
		 * @param enum TypeOfSystem Type of floating-base system
		 */
		void setTypeOfDynamicSystem(enum TypeOfSystem _type_of_system);


		/**
		 * @brief Sets the system DoF
		 * @param unsigned int Number of DoF
		 */
		void setSystemDoF(unsigned int _num_dof);

		/**
		 * @brief Sets the actuated joint DoF
		 * @param unsigned int Number of joints
		 */
		void setJointDoF(unsigned int _num_joints);

		/**
		 * @brief Gets the rigid body dynamic model
		 * @return const RigidBodyDynamics::Model& Rigid body dynamics model
		 */
		RigidBodyDynamics::Model& getRBDModel();

		/**
		 * @brief Gets the total mass of the rigid body system
		 * @return double The total mass of the rigid body system
		 */
		double getTotalMass();

		/**
		 * @brief Gets the body mass
		 * @return const double& The mass of the body
		 */
		const double& getBodyMass(std::string body_name) const;

		/** @brief Gets the gravity vector of the rigid body system */
		const Eigen::Vector3d& getGravityVector() const;

		/** @brief Gets the gravity acceleration of the rigid body system */
		const double& getGravityAcceleration() const;

		/** @brief Gets the gravity direction of the rigid body system */
		const Eigen::Vector3d& getGravityDirection() const;

		/**
		 * @brief Gets the Center of Mass (CoM) of the floating-base system
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @return Eigen::Vector3d& The CoM of the floating-base system
		 */
		Eigen::Vector3d& getSystemCoM(const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Gets the Center of Mass (CoM) rate of the floating-base system
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @return Eigen::Vector3d& The CoM rate of the floating-base system
		 */
		Eigen::Vector3d& getSystemCoMRate(const rbd::Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const rbd::Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel);

		/**
		 * @brief Gets the Center of Mass (CoM) of floating-base
		 * @return double The CoM of the floating-base
		 */
		const Eigen::Vector3d& getFloatingBaseCoM();

		/**
		 * @brief Gets the Center of Mass (CoM) of a specific body
		 * @param std::string Body name
		 * @return double The CoM of the body
		 */
		const Eigen::Vector3d& getBodyCoM(std::string body_name);

		/**
		 * @brief Gets the floating-base system DoF
		 * @return const unsigned int& Number of DoF of the floating-base system
		 */
		const unsigned int& getSystemDoF();

		/**
		 * @brief Gets the floating-base DoF
		 * @return const unsigned int& Number of floating-base DoF
		 */
		const unsigned int& getFloatingBaseDoF() const;

		/**
		 * @brief Gets the actuated joint DoF
		 * @return const unsigned int& Number of joint DoF
		 */
		const unsigned int& getJointDoF() const;

		/**
		 * @brief Gets the floating-base joint
		 * @param rbd::Coords6d Floating-base joint coordinate
		 * return const FloatingBaseJoint& Floating-base joint information
		 */
		const FloatingBaseJoint& getFloatingBaseJoint(rbd::Coords6d joint) const;

		/**
		 * @brief Gets the floating-base joint coordinate given an Id
		 * @param unsigned int Floating-base joint Id
		 * @return unsigned int Floating-base joint coordinate
		 */
		unsigned int getFloatingBaseJointCoordinate(unsigned int id);

		/**
		 * @brief Gets the name of the floating-base body
		 * @return Returns the name of the floating-base body
		 */
		const std::string& getFloatingBaseName();

		/**
		 * @brief Gets the joint id given the name
		 * @param std::string Joint name
		 * @return Returns the joint id
		 */
		const unsigned int& getJointId(std::string joint_name) const;

		/**
		 * @brief Gets actuated joint information
		 * @return const urdf_model::JointID& Joint names and Ids
		 */
		const urdf_model::JointID& getJoints();

		/**
		 * @brief Gets actuated joint limits
		 * @return const urdf_model::JointLimits& Joint names and limits
		 */
		const urdf_model::JointLimits& getJointLimits();

		/**
		 * @brief Gets the floating-base joint names list
		 * @return const rbd::BodySelector& Joint names list
		 */
		const rbd::BodySelector& getFloatingJointNames();

		/**
		 * @brief Gets the joint names list
		 * @return const rbd::BodySelector& Joint names list
		 */
		const rbd::BodySelector& getJointNames();

		/**
		 * @brief Gets the body name of the floating-base
		 * @return std::string Floating-base body
		 */
		std::string getFloatingBaseBody();

		/**
		 * @brief Gets the type of floating-base system
		 * @return enum TypeOfSystem Type of floating-base system
		 */
		const enum TypeOfSystem& getTypeOfDynamicSystem() const;

		/**
		 * @brief Gets the number of end-effectors
		 * @param enum TypeOfEndEffector Type of end-effector
		 * @return const unsigned int& Number of end-effectors
		 */
		const unsigned int& getNumberOfEndEffectors(enum TypeOfEndEffector type = ALL);

		/**
		 * @brief Gets the end-effector id given the name
		 * @param std::string End-effector name
		 * @return Returns the end-effector id
		 */
		unsigned int& getEndEffectorId(std::string contact_name);

		/**
		 * @brief Gets the end-effectors names
		 * @return const urdf_model::LinkID& Names and ids of the end-effectors
		 */
		const urdf_model::LinkID& getEndEffectors(enum TypeOfEndEffector type = ALL);

		/**
		 * @brief Gets the end-effector names list
		 * @param enum TypeOfEndEffector Type of end-effector
		 * @return const rbd::BodySelector& End-effector names list
		 */
		const rbd::BodySelector& getEndEffectorNames(enum TypeOfEndEffector type = ALL);

		/** @brief Returns true if the system has fully floating-base */
		bool isFullyFloatingBase();

		/** @brief Returns true if the system has a virtual floating-base */
		bool isVirtualFloatingBaseRobot();

		/** @brief Returns true if the system has a physical constraint with a fully floating-base */
		bool isConstrainedFloatingBaseRobot();

		/** @brief Returns true if there are a physical constraint in the floating-base */
		bool hasFloatingBaseConstraints();

		/**
		 * @brief Converts the base and joint states to a generalized joint state
		 * @param const Vector6d& Base state
		 * @param const Eigen::VectorXd& Joint state
		 * @return Eigen::VectorXd& Generalized joint state
		 */
		Eigen::VectorXd& toGeneralizedJointState(const rbd::Vector6d& base_state,
												 const Eigen::VectorXd& joint_state);

		/**
		 * @brief Converts the generalized joint state to base and joint states
		 * @param Vector6d& Base state
		 * @param Eigen::VectorXd& Joint state
		 * @param const Eigen::VectorXd Generalized joint state
		 */
		void fromGeneralizedJointState(rbd::Vector6d& base_state,
									   Eigen::VectorXd& joint_state,
									   const Eigen::VectorXd& generalized_state);

		/**
		 * @brief Sets the joint state given a branch values
		 * @param Eigen::VectorXd& Joint state vector
		 * @param cons Eigen::VectorXd& Branch state
		 * @param std::string Body name
		 */
		void setBranchState(Eigen::VectorXd& new_joint_state,
							const Eigen::VectorXd& branch_state,
							std::string body_name);

		/**
		 * @brief Gets the branch values given a joint state
		 * @param Eigen::VectorXd& Joint state vector
		 * @param cons Eigen::VectorXd& Branch state
		 * @param std::string Body name
		 */
		Eigen::VectorXd getBranchState(Eigen::VectorXd& joint_state,
									   std::string body_name);

		/**
		 * @brief Gets the position index and number of DOF of certain branch
		 * @param unsigned int& Position index of the body branch
		 * @param unsigned int& Degrees of freedom of the body branch
		 * @param std::string Name of the body branch (end-effector name)
		 */
		void getBranch(unsigned int& pos_idx,
					   unsigned int& num_dof,
					   std::string body_name);

		/**
		 * @brief Gets the default posture defined in the system file
		 * @return Eigen::VectorXd Default joint position
		 */
		Eigen::VectorXd getDefaultPosture();


	private:
		/** @brief Rigid-body dynamic model */
		RigidBodyDynamics::Model rbd_model_;
		RigidBodyDynamics::Math::Vector3d com_system_;
		RigidBodyDynamics::Math::Vector3d comd_system_;

		/** @brief System name */
		std::string system_name_;

		/** @brief Number of DoFs */
		unsigned int num_system_joints_;
		unsigned int num_floating_joints_;
		unsigned int num_joints_;

		/** @brief System joints */
		FloatingBaseJoint floating_ax_;
		FloatingBaseJoint floating_ay_;
		FloatingBaseJoint floating_az_;
		FloatingBaseJoint floating_lx_;
		FloatingBaseJoint floating_ly_;
		FloatingBaseJoint floating_lz_;
		rbd::BodySelector floating_joint_names_;
		urdf_model::JointID joints_;
		urdf_model::JointLimits joint_limits_;
		rbd::BodySelector joint_names_;
		Eigen::VectorXd default_joint_pos_;
		Eigen::VectorXd full_state_;
		rbd::Vector6d base_state_;
		Eigen::VectorXd joint_state_;

		/** @brief System bodies */
		std::string floating_body_name_;

		/** @brief Type of system */
		enum TypeOfSystem type_of_system_;

		/** @brief End-effector information */
		urdf_model::LinkID end_effectors_;
		unsigned int num_end_effectors_;
		rbd::BodySelector end_effector_names_;

		urdf_model::LinkID feet_;
		unsigned int num_feet_;
		rbd::BodySelector foot_names_;

		/** @brief Gravity information */
		double grav_acc_;
		Eigen::Vector3d grav_dir_;
};

} //@namespace
} //@namespace dwl

#endif
