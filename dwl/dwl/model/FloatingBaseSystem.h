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
		 * @param[in] urdf URDF filename
		 * @param[in] yarf Semantic system description filename
		 */
		void resetFromURDFFile(const std::string& urdf,
							   const std::string& yarf = std::string());

		/**
		 * @brief Resets the system information from URDF model
		 * @param[in] urdf URDF model
		 * @param[in] yarf Semantic system description filename
		 */
		void resetFromURDFModel(const std::string& urdf,
								const std::string& yarf = std::string());

		/**
		 * @brief Resets the system semantic description from yaml file
		 * @param[in] filename Semantic system description filename
		 */
		void resetSystemDescription(const std::string& filename);

		/**
		 * @brief Sets the 6d floating-base joint information
		 * @param[in] joint Floating-base joint information
		 **/
		void setFloatingBaseJoint(const FloatingBaseJoint& joint);

		/**
		 * @brief Sets the floating-base joint information
		 * @param[in] joint Floating-base joint information
		 * @param[in] joint_coord Joint coordinate
		 **/
		void setFloatingBaseJoint(const FloatingBaseJoint& joint,
								  rbd::Coords6d joint_coord);

		/**
		 * @brief Sets the actuated joint information
		 * @param[in] joint Joint information
		 */
		void setJoint(const Joint& joint);

		/**
		 * @brief Sets the floating-base constraint information
		 * @param[in] id Joint coordinate
		 */
		void setFloatingBaseConstraint(rbd::Coords6d id);

		/**
		 * @brief Sets the type of floating-base system
		 * @param[in] type Type of floating-base system
		 */
		void setTypeOfDynamicSystem(enum TypeOfSystem type);


		/**
		 * @brief Sets the system DoF
		 * @param[in] ndof Number of DoF
		 */
		void setSystemDoF(unsigned int ndof);

		/**
		 * @brief Sets the actuated joint DoF
		 * @param[in] njoints int Number of joints
		 */
		void setJointDoF(unsigned int njoints);

		/**
		 * @brief Gets the URDF model
		 * @return The URDF model
		 */
		const std::string& getURDFModel() const;

		/**
		 * @brief Gets the YARF model
		 * @return The YARF model
		 */
		const std::string& getYARFModel() const;

		/**
		 * @brief Gets the rigid body dynamic model
		 * @return The rigid body dynamics model
		 */
		RigidBodyDynamics::Model& getRBDModel();

		/**
		 * @brief Gets the total mass of the rigid body system
		 * @return The total mass of the rigid body system
		 */
		double getTotalMass();

		/**
		 * @brief Gets the body mass
		 * @param[in] name The body name
		 * @return The mass of the body
		 */
		const double& getBodyMass(const std::string& name) const;

		/** @brief Gets the gravity vector of the rigid body system
		 * @return The gravity vector
		 */
		const Eigen::Vector3d& getGravityVector() const;

		/** @brief Gets the gravity acceleration of the rigid body system 
		 * @return The gravity acceleration
		*/
		const double& getGravityAcceleration() const;

		/** @brief Gets the gravity direction of the rigid body system
		 * @return The gravity direction
		 */
		const Eigen::Vector3d& getGravityDirection() const;

		/**
		 * @brief Gets the Center of Mass (CoM) of the floating-base system
		 * @param[in] base_pos Base position
		 * @param[in] joint_pos Joint position
		 * @return The CoM of the floating-base system
		 */
		const Eigen::Vector3d& getSystemCoM(const rbd::Vector6d& base_pos,
											const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Gets the Center of Mass (CoM) rate of the floating-base system
		 * @param[in] base_pos Base position
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity
		 * @param[in] joint_vel Joint velocity
		 * @return The CoM rate of the floating-base system
		 */
		const Eigen::Vector3d& getSystemCoMRate(const rbd::Vector6d& base_pos,
												const Eigen::VectorXd& joint_pos,
												const rbd::Vector6d& base_vel,
												const Eigen::VectorXd& joint_vel);

		/**
		 * @brief Gets the Center of Mass (CoM) of floating-base
		 * @return The CoM of the floating-base
		 */
		const Eigen::Vector3d& getFloatingBaseCoM() const;

		/**
		 * @brief Gets the Center of Mass (CoM) of a specific body
		 * @param[in] name Body name
		 * @return The CoM of the body
		 */
		const Eigen::Vector3d& getBodyCoM(const std::string& name) const;

		/**
		 * @brief Gets the floating-base system DoF
		 * @return Number of DoF of the floating-base system
		 */
		const unsigned int& getSystemDoF() const;

		/**
		 * @brief Gets the floating-base DoF
		 * @return Number of floating-base DoF
		 */
		const unsigned int& getFloatingBaseDoF() const;

		/**
		 * @brief Gets the actuated joint DoF
		 * @return Number of joint DoF
		 */
		const unsigned int& getJointDoF() const;

		/**
		 * @brief Gets the floating-base joint
		 * @param[in] joint Floating-base joint coordinate
		 * return Floating-base joint information
		 */
		const FloatingBaseJoint& getFloatingBaseJoint(rbd::Coords6d joint) const;

		/**
		 * @brief Gets the floating-base joint coordinate given an Id
		 * @param[in] id Floating-base joint Id
		 * @return Floating-base joint coordinate
		 */
		unsigned int getFloatingBaseJointCoordinate(unsigned int id);

		/**
		 * @brief Gets the name of the floating-base body
		 * @return Returns the name of the floating-base body
		 */
		const std::string& getFloatingBaseName() const;

		/**
		 * @brief Checks if the joint exist
		 * @return True if the joint exist, false otherwise
		 */
		bool existJoint(const std::string& joint_name) const;

		/**
		 * @brief Gets the joint id given the name
		 * @param[in] name Joint name
		 * @return Returns the joint id
		 */
		const unsigned int& getJointId(const std::string& name) const;

		/**
		 * @brief Gets actuated joint information
		 * @return Joint names and Ids
		 */
		const urdf_model::JointID& getJoints() const;

		/**
		 * @brief Gets actuated joint limits
		 * @return Joint names and limits
		 */
		const urdf_model::JointLimits& getJointLimits() const;

		/**
		 * @brief Gets the joint limits given its name
		 * @param[in] name Joint name
		 * @return The joint limits
		 */
		const urdf::JointLimits& getJointLimit(const std::string& name) const;

		/** @brief Gets the lower joint limit
		 * @param[in] name Joint name
		 * @return The lower joint position limit
		 */
		const double& getLowerLimit(const std::string& name) const;

		/** @brief Gets the lower joint limit
		 * @param[in] joint Joint limits
		 * @return The lower joint position limit
		 */
		const double& getLowerLimit(const urdf::JointLimits& joint) const;

		/** @brief Gets the upper joint limit
		 * @param[in] name Joint name
		 * @return The upper joint position limit
		 */
		const double& getUpperLimit(const std::string& name) const;

		/** @brief Gets the upper joint limit
		 * @param[in] joint Joint limits
		 * @return The upper joint position limit
		 */
		const double& getUpperLimit(const urdf::JointLimits& joint) const;

		/** @brief Gets the velocity joint limit
		 * @param[in] name Joint name
		 * @return The velocity joint position limit
		 */
		const double& getVelocityLimit(const std::string& name) const;

		/** @brief Gets the velocity joint limit
		 * @param[in] name Joint limits
		 * @return The velocity joint position limit
		 */
		const double& getVelocityLimit(const urdf::JointLimits& joint) const;

		/** @brief Gets the effort joint limit
		 * @param[in] name Joint name
		 * @return The effort joint position limit
		 */
		const double& getEffortLimit(const std::string& name) const;

		/** @brief Gets the effort joint limit
		 * @param[in] name Joint limits
		 * @return The effort joint position limit
		 */
		const double& getEffortLimit(const urdf::JointLimits& joint) const;

		/**
		 * @brief Gets the floating-base joint names list
		 * @return Joint names list
		 */
		const rbd::BodySelector& getFloatingJointNames() const;

		/**
		 * @brief Gets the joint name given its id
		 * @return id Joint id
		 * @return Joint name
		 */
		const std::string& getJointName(const unsigned int& id) const;

		/**
		 * @brief Gets the joint names list
		 * @return Joint names list
		 */
		const rbd::BodySelector& getJointNames() const;

		/**
		 * @brief Gets the body name of the floating-base
		 * @return Floating-base body
		 */
		const std::string& getFloatingBaseBody() const;

		/**
		 * @brief Gets the type of floating-base system
		 * @return Type of floating-base system
		 */
		const enum TypeOfSystem& getTypeOfDynamicSystem() const;

		/**
		 * @brief Gets the number of end-effectors
		 * @param[in] type Type of end-effector
		 * @return Number of end-effectors
		 */
		const unsigned int& getNumberOfEndEffectors(enum TypeOfEndEffector type = ALL) const;

		/**
		 * @brief Gets the end-effector id given the name
		 * @param[in] name End-effector name
		 * @return Returns the end-effector id
		 */
		const unsigned int& getEndEffectorId(const std::string& name) const;

		/**
		 * @brief Gets the end-effectors names
		 * @return Names and ids of the end-effectors
		 */
		const urdf_model::LinkID& getEndEffectors(enum TypeOfEndEffector type = ALL) const;

		/**
		 * @brief Gets the end-effector names list
		 * @param[in] type Type of end-effector
		 * @return End-effector names list
		 */
		const rbd::BodySelector& getEndEffectorNames(enum TypeOfEndEffector type = ALL) const;

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
		 * @param[in] base_state Base state
		 * @param[in] joint_state Joint state
		 * @return Generalized joint state
		 */
		const Eigen::VectorXd& toGeneralizedJointState(const rbd::Vector6d& base_state,
													   const Eigen::VectorXd& joint_state);

		/**
		 * @brief Converts the generalized joint state to base and joint states
		 * @param[out] base_state Base state
		 * @param[in] joint_state Joint state
		 * @param[in] gen_state Generalized joint state
		 */
		void fromGeneralizedJointState(rbd::Vector6d& base_state,
									   Eigen::VectorXd& joint_state,
									   const Eigen::VectorXd& gen_state);

		/**
		 * @brief Sets the joint state given a branch values
		 * @param[out] new_joint_state Joint state vector
		 * @param[in] branch_state Branch state
		 * @param[in] name Body name
		 */
		void setBranchState(Eigen::VectorXd& new_joint_state,
							const Eigen::VectorXd& branch_state,
							std::string name);

		/**
		 * @brief Gets the branch values given a joint state
		 * @param[in] joint_name Joint state vector
		 * @param[in] body_name Body name
		 * @return The branch states
		 */
		Eigen::VectorXd getBranchState(const Eigen::VectorXd& joint_state,
									   const std::string& body_name);

		/**
		 * @brief Gets the position index and number of DOF of certain branch
		 * @param[in] pos_idx Position index of the body branch
		 * @param[in] num_dof Degrees of freedom of the body branch
		 * @param[in] name Name of the body branch (end-effector name)
		 */
		void getBranch(unsigned int& pos_idx,
					   unsigned int& num_dof,
					   const std::string& name);

		/**
		 * @brief Gets the default posture defined in the system file
		 * @return Default joint position
		 */
		const Eigen::VectorXd& getDefaultPosture() const;


	private:
		/** @brief Compared string function */
		bool compareString(std::string a, std::string b);

		/** @brief Robot models (urdf and yarf) */
		std::string urdf_;
		std::string yarf_;

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
