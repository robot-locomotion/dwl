#ifndef DWL__MODEL__WHOLE_BODY_KINEMATICS__H
#define DWL__MODEL__WHOLE_BODY_KINEMATICS__H

#include <model/FloatingBaseSystem.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class WholeBodyKinematics
 * @brief WholeBodyKinematics class implements the kinematics methods for a floating-base robot
 */
class WholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		WholeBodyKinematics();

		/** @brief Destructor function */
		~WholeBodyKinematics();

		/**
		 * @brief Build the model rigid-body system from an URDF file
		 * @param std::string URDF filename
		 * @param Print model information
		 */
		void modelFromURDFFile(std::string filename,
							   bool info = false);

		/**
		 * @brief Build the model rigid-body system from an URDF model (xml)
		 * @param std::string URDF model
		 * @param Print model information
		 */
		void modelFromURDFModel(std::string urdf_model,
								bool info = false);

		/**
		 * @brief Computes the forward kinematics for a predefined set of bodies
		 * @param rbd::BodyVector& Operational position of bodies
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as:
		 * linear, angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeForwardKinematics(rbd::BodyVector& op_pos,
									  const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const rbd::BodySelector& body_set,
									  enum rbd::Component component = rbd::Full,
									  enum TypeOfOrientation type = RollPitchYaw);

		/**
		 * @brief Computes the inverse kinematics for a predefined set of bodies positions. This
		 * inverse kinematics algorithm uses an operational position which consists of the desired
		 * 3d position for the base and each body
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @param const rbd::BodyPosition& Operational position of bodies
		 * @param double Step tolerance
		 * @param double Lambda value for singularities
		 * @param unsigned int Maximum number of iterations
		 */
		void computeInverseKinematics(rbd::Vector6d& base_pos,
									  Eigen::VectorXd& joint_pos,
									  const rbd::Vector6d& base_pos_init,
									  const Eigen::VectorXd& joint_pos_init,
									  const rbd::BodyPosition& op_pos,
									  double step_tol = 1.0e-12,
									  double lambda = 0.01,
									  unsigned int max_iter = 50);

		/**
		 * @brief Computes the whole-body jacobian for a predefined set of bodies. A whole-body
		 * jacobian is defined as end-effector (body) jacobian with respect to the inertial frame
		 * of the robot. Additionally, the whole-body jacobian represents a stack of floating-base
		 * effector jacobians in which there are base and end-effector (body) contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as:
		 * linear, angular and full
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Gets the floating-base contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Floating-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not fully floating-base,
		 * i.e. a floating-base with physical constraints
		 */
		void getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
									 const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Gets the fixed-base jacobian contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not fully floating-base,
		 * i.e. a floating-base with physical constraints
		 */
		void getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
								  const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Computes the operational velocity from the joint space for a predefined set of
		 * bodies of the robot
		 * @param rbd::BodyVector& Operational velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as:
		 * linear, angular and full
		 */
		void computeVelocity(rbd::BodyVector& op_vel,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration from the joint space for a predefined set of
		 * bodies of the robot
		 * @param rbd::BodyVector& Operational acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as:
		 * linear, angular and full
		 */
		void computeAcceleration(rbd::BodyVector& op_acc,
								 const rbd::Vector6d& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const rbd::Vector6d& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const rbd::Vector6d& base_acc,
								 const Eigen::VectorXd& joint_acc,
								 const rbd::BodySelector& body_set,
								 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for a
		 * predefined set of bodies of the robot, i.e. Jac_d * q_d
		 * @param rbd::BodyVector& Operational acceleration contribution from joint velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as:
		 * linear, angular and full
		 */
		void computeJdotQdot(rbd::BodyVector& jacd_qd,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);

		/** @brief Gets the floating-base system information */
		FloatingBaseSystem& getFloatingBaseSystem();

		/**
		 * @brief Gets the number of active end-effectors
		 * @param cons rbd::EndEffectorSelector& End-effector set
		 */
		int getNumberOfActiveEndEffectors(const rbd::BodySelector& effector_set);


	private:
		/** @brief Fixed body ids */
		rbd::BodyID body_id_;

		/** @brief A floating-base system definition */
		FloatingBaseSystem system_;
};

} //@namespace model
} //@namespace dwl

#endif
