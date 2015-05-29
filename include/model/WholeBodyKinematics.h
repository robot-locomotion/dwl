#ifndef DWL_WholeBodyKinematics_H
#define DWL_WholeBodyKinematics_H

#include <Eigen/Dense>
#include <map>
#include <utils/Orientation.h>
#include <utils/utils.h>
#include <utils/Math.h>
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>


namespace dwl
{

namespace model
{

using namespace iit;

enum Component {Linear, Angular, Full};

/**
 * @brief WholeBodyKinematics class implements the kinematics methods for a floating-base robot
 */
class WholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		WholeBodyKinematics();

		/** @brief Destructor function */
		virtual ~WholeBodyKinematics();

		/**
		 * @brief This abstract method updates the state of the robot, which as a convention is
		 * [xb^T; q^T]^T where xb is the position and orientation of the robot base and q is joint
		 * configuration of the rigid body
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		virtual void updateState(const iit::rbd::Vector6D& base_pos,
								 const Eigen::VectorXd& joint_pos) = 0;


		/**
		 * @brief Computes the forward kinematics for all end-effectors of the robot
		 * @param Eigen::VectorXd& Operation position of end-effectors of the robot
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeEffectorFK(Eigen::VectorXd& op_pos,
							   const Eigen::VectorXd& joint_pos,
							   enum Component component = Full);

		/**
		 * @brief Computes the forward kinematics for a predefined set of end-effectors
		 * @param Eigen::VectorXd& Operation position of end-effectors
		 * @param const Eigen::VectorXd& Joint position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeEffectorFK(Eigen::VectorXd& op_pos,
							   const Eigen::VectorXd& joint_pos,
							   EndEffectorSelector effector_set,
							   enum Component component = Full);

		/**
		 * @brief Computes the inverse kinematics for all end-effectors of the robot
		 * @param Eigen::VectorXd& Operation position of end-effectors of the robot
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeEffectorIK(Eigen::VectorXd& joint_pos,
									   Eigen::VectorXd& joint_vel,
									   const Eigen::VectorXd& op_pos,
									   const Eigen::VectorXd& op_vel) = 0;

		/**
		 * @brief This method computes the whole-body jacobian for all end-effector of the robot.
		 * The whole-body jacobian represents a stack of floating-base effector jacobians in which
		 * there are base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
											  const iit::rbd::Vector6D& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  enum Component component = Full);

		/**
		 * @brief This method computes the whole-body jacobian for a predefined set of end-effectors
		 * of the robot. The whole-body jacobian represents a stack of floating-base effector jacobians
		 * in which there are base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
											  const iit::rbd::Vector6D& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  EndEffectorSelector effector_set,
											  enum Component component = Full);

		/**
		 * @brief This method computes the free-base jacobian contribution for all the end-effectors of
		 * the robot.
		 * @param Eigen::MatrixXd& Free-base jacobian
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFreeBaseJacobian(Eigen::MatrixXd& jacobian,
										 	 const iit::rbd::Vector6D& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 enum Component component = Full);

		/**
		 * @brief This method computes the free-base jacobian contribution for a predefined set of
		 * end-effectors of the robot.
		 * @param Eigen::MatrixXd& Free-base jacobian
		 * @param const iit::rbd::Vector6D& Base position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFreeBaseJacobian(Eigen::MatrixXd& jacobian,
											 const iit::rbd::Vector6D& base_pos,
											 const Eigen::VectorXd& joint_pos,
											 EndEffectorSelector effector_set,
											 enum Component component = Full);

		/**
		 * @brief This method computes the fixed-base jacobian contribution for all the end-effectors of
		 * the robot.
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											  const Eigen::VectorXd& joint_pos,
											  enum Component component = Full);

		/**
		 * @brief This method computes the fixed-base jacobian contribution for a predefined set of
		 * end-effectors of the robot.
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::VectorXd& Joint position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void computeFixedBaseJacobian(Eigen::MatrixXd& jacobian,
											  const Eigen::VectorXd& joint_pos,
											  EndEffectorSelector effector_set,
											  enum Component component = Full);

		/**
		 * @brief Computes the operational velocity from the joint space for all end-effectors of the
		 * robot, i.e. x_dot = Jac * q_dot
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
									  const iit::rbd::Vector6D& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const iit::rbd::Vector6D& base_vel,
									  const Eigen::VectorXd& joint_vel,
									  enum Component component = Full);

		/**
		 * @brief Computes the operational velocity from the joint space for a predefined set of
		 * end-effectors of the robot, i.e. x_d = Jac * q_d
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opVelocityFromJointSpace(Eigen::VectorXd& op_vel,
									  const iit::rbd::Vector6D& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const iit::rbd::Vector6D& base_vel,
									  const Eigen::VectorXd& joint_vel,
									  EndEffectorSelector effector_set,
									  enum Component component = Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint acceleration for all
		 * end-effectors of the robot, i.e. Jac * q_dd
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint acceleration
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
												   	   	     const iit::rbd::Vector6D& base_pos,
															 const Eigen::VectorXd& joint_pos,
															 const iit::rbd::Vector6D& base_acc,
															 const Eigen::VectorXd& joint_acc,
															 enum Component component = Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint acceleration for a
		 * predefined set of end-effector of the robot, i.e. Jac * q_dd
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint acceleration
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void opAccelerationContributionFromJointAcceleration(Eigen::VectorXd& jac_qdd,
												   	   	     const iit::rbd::Vector6D& base_pos,
															 const Eigen::VectorXd& joint_pos,
															 const iit::rbd::Vector6D& base_acc,
															 const Eigen::VectorXd& joint_acc,
															 EndEffectorSelector effector_set,
															 enum Component component = Full);

		/** @brief Gets the end-effector list */
		EndEffectorID& getEndEffectorList();

		Eigen::Matrix3d getBaseRotationMatrix();
		Eigen::Matrix4d getHomogeneousTransform(std::string effector_name);

		typedef std::map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic> > EndEffectorJacobian;
		typedef std::map<std::string, Eigen::Matrix<double, 4, 4> > EndEffectorHomTransform;


	protected:
		/** @brief End-effector ids */
		EndEffectorID effector_id_;

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
