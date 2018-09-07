#ifndef DWL__MODEL__FLOATING_BASE_SYSTEM__H
#define DWL__MODEL__FLOATING_BASE_SYSTEM__H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <dwl/utils/URDF.h>
#include <dwl/utils/YamlWrapper.h>


namespace dwl
{

namespace model
{

/** @brief Defines the type of end-effectors */
enum TypeOfEndEffector {ALL, FOOT};

typedef std::map<std::string, unsigned int> ElementId;
typedef std::vector<std::string> ElementList;
enum RootJoint {FREE_FLYER = 0, PLANAR, NO};

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
		FloatingBaseSystem();

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
		 * @brief Gets the pinocchio model
		 * @return The pinocchio model
		 */
		se3::Model& getModel();

		/**
		 * @brief Gets the pinocchio data
		 * @return The pinocchio data
		 */
		se3::Data& getData();

		/**
		 * @brief Gets the configuration manifold dimension of the system
		 * @return Dimension of the configuration manifold
		 */
		unsigned int getConfigurationDim();

		/**
		 * @brief Gets the tangent space dimension of the system
		 * @return Dimension of the tangent space
		 */
		unsigned int getTangentDim();

		/**
		 * @brief Gets the body name of the floating-base
		 * @return Floating-base body
		 */
		const std::string& getFloatingBaseName() const;

		/**
		 * @brief Gets the actuated joint DoF
		 * @return Number of joint DoF
		 */
		const unsigned int& getJointDoF() const;

		/**
		 * @brief Gets the joint id given the name
		 * @param[in] name Joint name
		 * @return Returns the joint id
		 */
		const unsigned int& getJointId(const std::string& name) const;

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
		const ElementList& getJointNames() const;

		/**
		 * @brief Gets actuated joint information
		 * @return Joint names and Ids
		 */
		const ElementId& getJoints() const;

		/**
		 * @brief Gets actuated lower joint limits
		 * @return Lower joint limits
		 */
		Eigen::VectorXd getLowerLimits();

		/** @brief Gets the lower joint limit
		 * @param[in] name Joint name
		 * @return The lower joint position limit
		 */
		double getLowerLimit(const std::string& name);

		/**
		 * @brief Gets actuated lower joint limits
		 * @return Lower joint limits
		 */
		Eigen::VectorXd getUpperLimits();

		/** @brief Gets the upper joint limit
		 * @param[in] name Joint name
		 * @return The upper joint position limit
		 */
		double getUpperLimit(const std::string& name);

		/**
		 * @brief Gets actuated velocity joint limits
		 * @return Velocity joint limits
		 */
		Eigen::VectorXd getVelocityLimits();

		/** @brief Gets the velocity joint limit
		 * @param[in] name Joint name
		 * @return The velocity joint position limit
		 */
		double getVelocityLimit(const std::string& name);

		/**
		 * @brief Gets actuated effort joint limits
		 * @return Effort joint limits
		 */
		Eigen::VectorXd getEffortLimits();

		/** @brief Gets the effort joint limit
		 * @param[in] name Joint name
		 * @return The effort joint position limit
		 */
		double getEffortLimit(const std::string& name);

		/**
		 * @brief Gets the default posture defined in the system file
		 * @return Default joint position
		 */
		const Eigen::VectorXd& getDefaultPosture() const;

		/**
		 * @brief Checks if the joint exist
		 * @param[in] name Joint name
		 * @return True if the joint exist, false otherwise
		 */
		bool existJoint(const std::string& name) const;

		/**
		 * @brief Gets the body id given the name
		 * @param[in] name Body name
		 * @return Returns the body id
		 */
		const unsigned int& getBodyId(const std::string& name) const;

		/**
		 * @brief Gets the body name given its id
		 * @return id Body id
		 * @return Body name
		 */
		const std::string& getBodyName(const unsigned int& id) const;

		/**
		 * @brief Gets body information
		 * @return Body names and Ids
		 */
		const ElementId& getBodies() const;

		/**
		 * @brief Gets the total mass of the rigid body system
		 * @return The total mass of the rigid body system
		 */
		double getTotalMass();

		/**
		 * @brief Gets the floating-base mass
		 * @return The mass of the body
		 */
		double getFloatingBaseMass();

		/**
		 * @brief Gets the body mass
		 * @param[in] name The body name
		 * @return The mass of the body
		 */
		double getBodyMass(const std::string& name);

		/**
		 * @brief Gets the Center of Mass (CoM) of floating-base
		 * @return The CoM of the floating-base
		 */
		Eigen::Vector3d getFloatingBaseCoM();

		/**
		 * @brief Gets the Center of Mass (CoM) of a specific body
		 * @param[in] name Body name
		 * @return The CoM of the body
		 */
		Eigen::Vector3d getBodyCoM(const std::string& name);

		/**
		 * @brief Gets the inertia matrix of floating-base
		 * @return The inertia matrix of the floating-base
		 */
		Eigen::Matrix3d getFloatingBaseInertia();

		/**
		 * @brief Gets the inertia matrix of a specific body
		 * @param[in] name Body name
		 * @return The inertia matrix of the body
		 */
		Eigen::Matrix3d getBodyInertia(const std::string& name);

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
		const ElementId& getEndEffectors(enum TypeOfEndEffector type = ALL) const;

		/**
		 * @brief Gets the end-effector names list
		 * @param[in] type Type of end-effector
		 * @return End-effector names list
		 */
		const ElementList& getEndEffectorNames(enum TypeOfEndEffector type = ALL) const;

		/** @brief Gets the gravity vector of the rigid body system
		 * @return The gravity vector
		 */
		Eigen::Vector3d getGravityVector();

		/** @brief Gets the gravity acceleration of the rigid body system 
		 * @return The gravity acceleration
		*/
		const double& getGravityAcceleration() const;

		/** @brief Gets the gravity direction of the rigid body system
		 * @return The gravity direction
		 */
		const Eigen::Vector3d& getGravityDirection() const;

		/** @brief Returns true if the system has a floating base */
		bool isFloatingBase();

		/** @brief Returns true if the system is has a fixed base */
		bool isFixedBase();

		/** @brief Returns true if the system has a physical constraint
		 *  in the floating base */
		bool isConstrainedFloatingBase();

		/**
		 * @brief Converts the base and joint configurations to a
		 * generalized configuration state
		 * @param[in] base_state Base configuration
		 * @param[in] joint_state Joint configuration
		 * @return Generalized configuration state
		 */
		const Eigen::VectorXd& toConfigurationState(dwl::SE3& base_state,
													const Eigen::VectorXd& joint_state);

		/**
		 * @brief Converts the base and joint tangent states to a
		 * generalized tangent state
		 * @param[in] base_state Base tangent state
		 * @param[in] joint_state Joint tangent state
		 * @return Generalized tangent state
		 */
		const Eigen::VectorXd& toTangentState(const dwl::Motion& base_state,
											  const Eigen::VectorXd& joint_state);

		/**
		 * @brief Converts the generalized configuration state to base and joint states
		 * @param[out] base_state Base configuration
		 * @param[in] joint_state Joint configuration
		 * @param[in] gen_state Generalized configuration state
		 */
		void fromConfigurationState(dwl::SE3& base_state,
									Eigen::VectorXd& joint_state,
									const Eigen::VectorXd& gen_state);

		/**
		 * @brief Converts the generalized tangent state to base and joint states
		 * @param[out] base_state Base tangent vector
		 * @param[in] joint_state Joint tangent vector
		 * @param[in] gen_state Generalized tangent state
		 */
		void fromTangentState(dwl::Motion& base_state,
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
		 * @param[in] joint_state Joint state vector
		 * @param[in] name Body name
		 * @return The branch states
		 */
		Eigen::VectorXd getBranchState(const Eigen::VectorXd& joint_state,
									   const std::string& name);

		/**
		 * @brief Gets the position index and number of DOF of certain branch
		 * @param[in] pos_idx Position index of the body branch
		 * @param[in] num_dof Degrees of freedom of the body branch
		 * @param[in] name Name of the body branch (end-effector name)
		 */
		void getBranch(unsigned int& pos_idx,
					   unsigned int& num_dof,
					   const std::string& name);

		/** Constant variable for undefined elements (joints or bodies) **/
		const unsigned int UNDEFINED_ID = std::numeric_limits<unsigned int>::max();

		/**
		 * @brief Get the type of root Joint
		 * @return Root joint type
		 */
		enum RootJoint getRootJoint();


	private:
		/** @brief Compared string function */
		bool compareString(std::string a, std::string b);

		/** @brief Robot models (urdf and yarf) */
		std::string urdf_;
		std::string yarf_;

		/** @brief Pinocchio model and data */
		se3::Model model_;
		se3::Data *data_ptr_;

		/** @brief Joint information */
		RootJoint root_joint_;
		unsigned int n_base_joints_;
		unsigned int n_joints_;
		ElementId joints_;
		ElementList joint_names_;
		std::string floating_joint_name_;

		/** @brief Body information */
		ElementId bodies_;
		ElementList body_names_;
		std::string floating_body_name_;

		/** @brief End-effector information */
		ElementId end_effectors_;
		ElementId feet_;
		ElementList end_effector_names_;		
		ElementList foot_names_;
		unsigned int num_end_effectors_;
		unsigned int num_feet_;

		/** @brief Vectors **/
		Eigen::VectorXd q_;
		Eigen::VectorXd v_;
		Eigen::VectorXd default_joint_pos_;

		/** @brief Gravity information */
		double grav_acc_;
		Eigen::Vector3d grav_dir_;
};

} //@namespace
} //@namespace dwl

#endif
