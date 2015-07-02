#ifndef DWL_RobCoGenWholeBodyKinematics_H
#define DWL_RobCoGenWholeBodyKinematics_H

#include <utils/utils.h>
#include <utils/RigidBodyDynamics.h>

#include <Eigen/Dense>
#include <map>
#include <iit/rbd/utils.h>


namespace dwl
{

namespace model
{

using namespace iit;

/**
 * @brief RobCoGenWholeBodyKinematics class implements the kinematics methods for a floating-base robot using
 * RobCoGen library
 */
class RobCoGenWholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		RobCoGenWholeBodyKinematics();

		/** @brief Destructor function */
		virtual ~RobCoGenWholeBodyKinematics();

		/**
		 * @brief This abstract method updates the state of the robot, which as a convention is
		 * [xb^T; q^T]^T where xb is the position and orientation of the robot base and q is joint
		 * configuration of the rigid body
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		virtual void updateState(const rbd::Vector6d& base_pos,
								 const Eigen::VectorXd& joint_pos) = 0;

		/**
		 * @brief Computes the forward kinematics for a predefined set of end-effectors
		 * @param Eigen::VectorXd& Operation position of bodies
		 * @param const Eigen::VectorXd& Joint position
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeForwardKinematics(Eigen::VectorXd& op_pos,
									  const Eigen::VectorXd& joint_pos,
									  rbd::BodySelector body_set,
									  enum rbd::Component component = rbd::Full);

		/**
		 * @brief This method computes the whole-body jacobian for a predefined set of end-effectors
		 * of the robot. The whole-body jacobian represents a stack of floating-base effector jacobians
		 * in which there are base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeJacobian(Eigen::MatrixXd& jacobian,
									 const rbd::Vector6d& base_pos,
									 const Eigen::VectorXd& joint_pos,
									 rbd::BodySelector body_set,
									 enum rbd::Component component = rbd::Full);

		/**
		 * @brief This method computes the floating-base jacobian contribution for a predefined set of
		 * end-effectors of the robot.
		 * @param Eigen::MatrixXd& Free-base jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
												 const rbd::Vector6d& base_pos,
												 const Eigen::VectorXd& joint_pos,
												 rbd::BodySelector bodies_set,
												 enum rbd::Component component = rbd::Full);

		/**
		 * @brief This method computes the fixed-base jacobian contribution for a predefined set of
		 * bodies of the robot.
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::VectorXd& Joint position
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											  const Eigen::VectorXd& joint_pos,
											  rbd::BodySelector body_set,
											  enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational velocity from the joint space for a predefined set of
		 * bodies of the robot, i.e. x_d = Jac * q_d
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
									  const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const rbd::Vector6d& base_vel,
									  const Eigen::VectorXd& joint_vel,
									  rbd::BodySelector effector_set,
									  enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint acceleration for a
		 * predefined set of bodies of the robot, i.e. Jac * q_dd
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
															 const rbd::Vector6d& base_pos,
															 const Eigen::VectorXd& joint_pos,
															 const rbd::Vector6d& base_acc,
															 const Eigen::VectorXd& joint_acc,
															 rbd::BodySelector body_set,
															 enum rbd::Component component = rbd::Full);

		/** @brief Gets the end-effector list */
		rbd::BodyID& getBodyList();

		Eigen::Matrix3d getBaseRotationMatrix();
		Eigen::Matrix4d getHomogeneousTransform(std::string effector_name);

		typedef std::map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic> > EndEffectorJacobian;
		typedef std::map<std::string, Eigen::Matrix<double, 4, 4> > EndEffectorHomTransform;


	protected:
		/** @brief Body ids */
		rbd::BodyID body_id_;

		/** @brief Jacobian matrix for the current state of the robot */
		EndEffectorJacobian jacobians_;

		/** @brief End-effector homogeneous transforms for the current state of the robot */
		EndEffectorHomTransform homogeneous_tf_;

		/** @brief Number of joints of the robot */
		int num_joints_;

		/** @brief Floating-base rotation matrix for the current state of the robot */
		Eigen::Matrix3d floating_base_rot_;
};

} //@namespace model
} //@namespace dwl

#endif
