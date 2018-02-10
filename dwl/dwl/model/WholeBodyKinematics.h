#ifndef DWL__MODEL__WHOLE_BODY_KINEMATICS__H
#define DWL__MODEL__WHOLE_BODY_KINEMATICS__H

#include <dwl/model/FloatingBaseSystem.h>
#include <dwl/utils/utils.h>


namespace dwl
{

namespace model
{

/**
 * @class WholeBodyKinematics
 * @brief WholeBodyKinematics class implements the kinematics methods for a
 * floating-base robot
 */
class WholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		WholeBodyKinematics();

		/** @brief Destructor function */
		~WholeBodyKinematics();

		/**
		 * @brief Set the floating-base model of the rigid-body system
		 * @param[in] fbs Floating-base model
		 */
		void reset(FloatingBaseSystem& fbs);

		/**
		 * @brief Sets the Ik solver properties
		 * @param double Step tolerance
		 * @param double Lambda value for singularities
		 * @param unsigned int Maximum number of iterations
		 */
		void setIKSolver(double step_tol,
						 double lambda,
						 unsigned int max_iter);

		/**
		 * @brief Computes the forward kinematics for a predefined set of bodies
		 * @param rbd::BodyVector& Operational position of bodies
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important
		 * kind of jacobian such as: linear, angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeForwardKinematics(rbd::BodyVectorXd& op_pos,
									  const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const rbd::BodySelector& body_set,
									  enum rbd::Component component = rbd::Full,
									  enum TypeOfOrientation type = RollPitchYaw);
		const rbd::BodyVectorXd& computePosition(const rbd::Vector6d& base_pos,
												 const Eigen::VectorXd& joint_pos,
												 const rbd::BodySelector& body_set,
												 enum rbd::Component component = rbd::Full,
												 enum TypeOfOrientation type = RollPitchYaw);


		/**
		 * @brief Computes the inverse kinematics for a predefined set of
		 * bodies positions.
		 * This inverse kinematics algorithm uses an operational position which
		 * consists of the desired 3d position for the base and each body
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodyPosition& Operational position of bodies
		 * @param const rbd::Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @return True on success, false otherwise
		 */
		bool computeInverseKinematics(rbd::Vector6d& base_pos,
									  Eigen::VectorXd& joint_pos,
									  const rbd::BodyVector3d& op_pos);
		bool computeInverseKinematics(rbd::Vector6d& base_pos,
									  Eigen::VectorXd& joint_pos,
									  const rbd::BodyVector3d& op_pos,
									  const rbd::Vector6d& base_pos_init,
									  const Eigen::VectorXd& joint_pos_init);

		/**
		 * @brief Computes the joint position from a predefined set of
		 * body positions w.r.t the base.
		 * This inverse kinematics algorithm uses an operational position which
		 * consists of the desired 3d position for each body
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodyPosition& Operational position of bodies
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @return True on success, false otherwise
		 */
		bool computeJointPosition(Eigen::VectorXd& joint_pos,
								  const rbd::BodyVector3d& op_pos);
		bool computeJointPosition(Eigen::VectorXd& joint_pos,
								  const rbd::BodyVector3d& op_pos,
								  const Eigen::VectorXd& joint_pos_init);

		/**
		 * @brief Computes the joint velocity for a predefined set of body
		 * velocities (q_d = J^-1 * x_d)
		 * @param Eigen::VectorXd& Joint velocities
		 * @param const Eigen::VectorXd& Joint positions
		 * @param const rbd::BodyVector& Operational velocities of bodies
		 * @param const rbd::BodySelector& A predefined set of bodies
		 */
		void computeJointVelocity(Eigen::VectorXd& joint_vel,
								  const Eigen::VectorXd& joint_pos,
								  const rbd::BodyVectorXd& op_vel,
								  const rbd::BodySelector& body_set);

		/**
		 * @brief Computes the joint acceleration for a predefined set of
		 * body (q_dd = J^-1 * [x_dd - J_d * q_d])
		 * @param Eigen::VectorXd& Joint accelerations
		 * @param const Eigen::VectorXd& joint positions
		 * @param const Eigen::VectorXd& joint velocities
		 * @param const rbd::BodyVector& Operational accelerations of bodies
		 * @param const rbd::BodySelector& A predefined set of bodies
		 */
		void computeJointAcceleration(Eigen::VectorXd& joint_acc,
									  const Eigen::VectorXd& joint_pos,
									  const Eigen::VectorXd& joint_vel,
									  const rbd::BodyVectorXd& op_acc,
									  const rbd::BodySelector& body_set);

		/**
		 * @brief Computes the whole-body jacobian for a predefined set of
		 * bodies. A whole-body jacobian is defined as end-effector (body)
		 * jacobian with respect to the inertial frame of the robot.
		 * Additionally, the whole-body jacobian represents a stack of
		 * floating-base effector jacobians in which there are base and
		 * end-effector (body) contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind
		 * of jacobian such as: linear, angular and full
		 */
		void computeJacobian(Eigen::MatrixXd& jacobian,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the fixed jacobian, without the floating-base
		 * component, for a certain body.
		 * @param Eigen::MatrixXd& Fixed jacobian
		 * @param const Eigen::VectorXd& Joint position
		 * @param const std::string& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind
		 * of jacobian such as: linear, angular and full
		 */
		void computeFixedJacobian(Eigen::MatrixXd& jacobian,
							 	  const Eigen::VectorXd& joint_pos,
							 	  const std::string& body_name,
							 	  enum rbd::Component component = rbd::Full);

		/**
		 * @brief Gets the floating-base contribution of a given whole-body
		 * jacobian
		 * @param Eigen::MatrixXd& Floating-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not
		 * fully floating-base, i.e. a floating-base with physical constraints
		 */
		void getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
									 const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Gets the fixed-base jacobian contribution of a given
		 * whole-body jacobian
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 * @param struct rbd::ReducedFloatingBase* Defined only when it's not
		 * fully floating-base, i.e. a floating-base with physical constraints
		 */
		void getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
								  const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Computes the operational velocity from the joint space for a
		 * predefined set of bodies of the robot
		 * @param rbd::BodyVector& Operational velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind
		 * of jacobian such as: linear, angular and full
		 */
		void computeVelocity(rbd::BodyVectorXd& op_vel,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);
		const rbd::BodyVectorXd& computeVelocity(const rbd::Vector6d& base_pos,
												 const Eigen::VectorXd& joint_pos,
												 const rbd::Vector6d& base_vel,
												 const Eigen::VectorXd& joint_vel,
												 const rbd::BodySelector& body_set,
												 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration from the joint space
		 * for a predefined set of bodies of the robot
		 * @param rbd::BodyVector& Operational acceleration
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind
		 * of jacobian such as: linear, angular and full
		 */
		void computeAcceleration(rbd::BodyVectorXd& op_acc,
								 const rbd::Vector6d& base_pos,
								 const Eigen::VectorXd& joint_pos,
								 const rbd::Vector6d& base_vel,
								 const Eigen::VectorXd& joint_vel,
								 const rbd::Vector6d& base_acc,
								 const Eigen::VectorXd& joint_acc,
								 const rbd::BodySelector& body_set,
								 enum rbd::Component component = rbd::Full);
		const rbd::BodyVectorXd& computeAcceleration(const rbd::Vector6d& base_pos,
								 	 	 	 	 	 const Eigen::VectorXd& joint_pos,
													 const rbd::Vector6d& base_vel,
													 const Eigen::VectorXd& joint_vel,
													 const rbd::Vector6d& base_acc,
													 const Eigen::VectorXd& joint_acc,
													 const rbd::BodySelector& body_set,
													 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration contribution from the
		 * joint velocity for a predefined set of bodies of the robot, i.e.
		 * Jac_d * q_d
		 * @param rbd::BodyVector& Operational acceleration contribution from
		 * joint velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::BodySelector& A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind
		 * of jacobian such as: linear, angular and full
		 */
		void computeJdotQdot(rbd::BodyVectorXd& jacd_qd,
							 const rbd::Vector6d& base_pos,
							 const Eigen::VectorXd& joint_pos,
							 const rbd::Vector6d& base_vel,
							 const Eigen::VectorXd& joint_vel,
							 const rbd::BodySelector& body_set,
							 enum rbd::Component component = rbd::Full);
		const rbd::BodyVectorXd& computeJdotQdot(const rbd::Vector6d& base_pos,
												 const Eigen::VectorXd& joint_pos,
												 const rbd::Vector6d& base_vel,
												 const Eigen::VectorXd& joint_vel,
												 const rbd::BodySelector& body_set,
												 enum rbd::Component component = rbd::Full);

		/** @brief Gets the floating-base system information */
		std::shared_ptr<FloatingBaseSystem> getFloatingBaseSystem();

		/**
		 * @brief Gets the number of active end-effectors
		 * @param cons rbd::EndEffectorSelector& End-effector set
		 */
		int getNumberOfActiveEndEffectors(const rbd::BodySelector& effector_set);


	private:
		/** @brief Fixed body ids */
		rbd::BodyID body_id_;

		/** @brief A floating-base system definition */
		std::shared_ptr<FloatingBaseSystem> fbs_;

		/** @brief Middle joint position */
		Eigen::VectorXd joint_pos_middle_;

		rbd::BodyVectorXd body_pos_;
		rbd::BodyVectorXd body_vel_;
		rbd::BodyVectorXd body_acc_;
		rbd::BodyVectorXd jdot_qdot_;

		/** @brief IK solver */
		double step_tol_;
		double lambda_;
		unsigned int max_iter_;
};

} //@namespace model
} //@namespace dwl

#endif
