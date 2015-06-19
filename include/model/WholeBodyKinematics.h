#ifndef DWL_WholeBodyKinematics_H
#define DWL_WholeBodyKinematics_H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <utils/RigidBodyDynamics.h>
#include <utils/Math.h>
#include <utils/utils.h>


namespace dwl
{

namespace model
{

/**
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
		 * @param std::string URDF file
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
		 * base with physical constraints
		 * @param Print model information
		 */
		void modelFromURDF(std::string file, struct rbd::ReducedFloatingBase* reduce_base = NULL, bool info = false);

		/**
		 * @brief Computes the forward kinematics for all end-effectors of the robot
		 * @param Eigen::VectorXd& Operation position of end-effectors of the robot
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeForwardKinematics(Eigen::VectorXd& op_pos,
									  const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  enum rbd::Component component = rbd::Full,
									  enum TypeOfOrientation type = RollPitchYaw);

		/**
		 * @brief Computes the forward kinematics for a predefined set of end-effectors
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeForwardKinematics(Eigen::VectorXd& op_pos,
									  const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  rbd::EndEffectorSelector effector_set,
									  enum rbd::Component component = rbd::Full,
									  enum TypeOfOrientation type = RollPitchYaw);

		/**
		 * @brief Computes the inverse kinematics for all set of end-effectors. This inverse kinematics
		 * algorithm uses an operational position which consists of the desired 3d position for the base and
		 * each end-effector
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param double Step tolerance
		 * @param double Lambda value for singularities
		 * @param unsigned int Maximum number of iterations
		 */
		void computeInverseKinematics(rbd::Vector6d& base_pos,
									  Eigen::VectorXd& joint_pos,
									  const rbd::Vector6d& base_pos_init,
									  const Eigen::VectorXd& joint_pos_init,
									  rbd::EndEffectorPosition op_pos,
									  double step_tol = 1.0e-12,
									  double lambda = 0.01,
									  unsigned int max_iter = 50);
		/**
		 * @brief Computes the inverse kinematics for a predefined set of end-effectors. This inverse kinematics
		 * algorithm uses an operational position which consists of the desired 3d position for the base and
		 * each end-effector
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param double Step tolerance
		 * @param double Lambda value for singularities
		 * @param unsigned int Maximum number of iterations
		 */
		void computeInverseKinematics(rbd::Vector6d& base_pos,
									  Eigen::VectorXd& joint_pos,
									  const rbd::Vector6d& base_pos_init,
									  const Eigen::VectorXd& joint_pos_init,
									  rbd::EndEffectorPosition op_pos,
									  rbd::EndEffectorSelector effector_set,
									  double step_tol = 1.0e-12,
									  double lambda = 0.01,
									  unsigned int max_iter = 50);

		/**
		 * @brief Computes the whole-body jacobian for all end-effector of the robot. A whole-body jacobian is
		 * defined as end-effector jacobian with respect to the inertial frame. Additionally, the whole-body
		 * jacobian represents a stack of floating-base effector jacobians in which there are base and
		 * effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the whole-body jacobian for a predefined set of end-effectors. A whole-body jacobian
		 * is defined as end-effector jacobian with respect to the inertial frame of the robot. Additionally,
		 * the whole-body jacobian represents a stack of floating-base effector jacobians in which there are
		 * base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 rbd::EndEffectorSelector effector_set,
							 enum rbd::Component component = rbd::Full);


		/**
		 * @brief Gets the floating-base contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Floating-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
		 * base with physical constraints
		 */
		void getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
									 const Eigen::MatrixXd& full_jacobian);


		/**
		 * @brief Gets the fixed-base jacobian contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not fully floating-base, i.e. a floating-
		 * base with physical constraints
		 */
		void getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
								  const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Computes the operational velocity from the joint space for all end-effectors of the
		 * robot
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeVelocity(Eigen::VectorXd& op_vel,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 enum rbd::Component component = rbd::Full);
		/**
		 * @brief Computes the operational velocity from the joint space for a predefined set of
		 * end-effectors of the robot
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeVelocity(Eigen::VectorXd& op_vel,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 rbd::EndEffectorSelector effector_set,
							 enum rbd::Component component = rbd::Full);
		/**
		 * @brief Computes the operational acceleration from the joint space for all end-effectors of the
		 * robot
		 * @param Eigen::VectorXd& Operational acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeAcceleration(Eigen::VectorXd& op_acc,
								 const rbd::Vector6d& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const rbd::Vector6d& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const rbd::Vector6d& base_acc,
								 const Eigen::VectorXd& joint_acc,
								 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration from the joint space for a predefined set of
		 * end-effectors of the robot
		 * @param Eigen::VectorXd& Operational acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeAcceleration(Eigen::VectorXd& op_acc,
								 const rbd::Vector6d& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const rbd::Vector6d& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const rbd::Vector6d& base_acc,
								 const Eigen::VectorXd& joint_acc,
								 rbd::EndEffectorSelector effector_set,
								 enum rbd::Component component = rbd::Full);
		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for all
		 * end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeJdotQdot(Eigen::VectorXd& jacd_qd,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for a
		 * predefined set of end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param rbd::EndEffectorSelector A predefined set of end-effectors
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeJdotQdot(Eigen::VectorXd& jacd_qd,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 rbd::EndEffectorSelector effector_set,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Gets the number of active end-effectors
		 * @param EndEffectorSelector End-effector set
		 */
		int getNumberOfActiveEndEffectors(rbd::EndEffectorSelector effector_set);

		/**
		 * @brief Actives all the end-effectors of the robot
		 * @param EndEffectorSelector& End-effector set
		 */
		void activeAllEndEffector(rbd::EndEffectorSelector& effector_set);


	private:
		/** @brief Model of the rigid-body system */
		RigidBodyDynamics::Model robot_model_;

		/* @brief Fixed body ids */
		rbd::EndEffectorID body_id_;

		/* @brief Full set of active end-effectors */
		rbd::EndEffectorSelector full_effector_set_;

		rbd::ReducedFloatingBase* reduced_base_;
};

} //@namespace model
} //@namespace dwl

#endif
