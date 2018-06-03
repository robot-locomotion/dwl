#ifndef DWL__MODEL__WHOLE_BODY_KINEMATICS__H
#define DWL__MODEL__WHOLE_BODY_KINEMATICS__H


#include <dwl/model/FloatingBaseSystem.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
// #include <pinocchio/algorithm/finite-differences.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <dwl/utils/utils.h>



namespace dwl
{

namespace model
{

/**
 * @brief WholeBodyKinematics class
 * This class implements various helpful methods for computing whole-body kinematics
 * quantities such as: CoM states, stack-of-Jacobians, frame and joint positions,
 * velocities and accelerations. Before using this class, you need to provide the
 * floating-base system description by passing this object in the reset function.
 * For improving code efficient, the kinematic quantities can be computed by updating
 * the joint states or no. In fact, it's possible to update the joint states only ones,
 * and not in each desired computation.
 * 
 * An important remark is that DWL defines SE(3) groups through 7d vector in which the first
 * 4 rows containts the quaternion and the next 3 the position (i.e. [q_wxyz, position]).
 * Additionally, the tangent of the SE(3) group is described as 6d vector ordered by
 * angular and linear components (i.e. [angular, linear]).
 * @author Carlos Mastalli
 * @copyright BSD 3-Clause License
 */
class WholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		WholeBodyKinematics();

		/** @brief Destructor function */
		~WholeBodyKinematics();

		/**
		 * @brief Sets the floating-base model of the rigid-body system
		 * 
		 * @param[in] fbs Floating-base model
		 */
		void reset(FloatingBaseSystem& fbs);

		/**
		 * @brief Sets the Ik solver properties.
		 * 
		 * @param[in] step_tol Step tolerance
		 * @param[in] max_iter Maximum number of iterations
		 */
		void setIKSolver(double step_tol,
						 unsigned int max_iter);

		/**
		 * @brief Updates the kinematics needed for computing the frame
		 * states (e.g. for calling getFramePosition).
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 */
		void updateKinematics(const Eigen::Vector7d& base_pos,
							  const Eigen::VectorXd& joint_pos);
		void updateKinematics(const Eigen::Vector7d& base_pos,
							  const Eigen::VectorXd& joint_pos,
							  const Eigen::Vector6d& base_vel,
							  const Eigen::VectorXd& joint_vel);
		void updateKinematics(const Eigen::Vector7d& base_pos,
							  const Eigen::VectorXd& joint_pos,
							  const Eigen::Vector6d& base_vel,
							  const Eigen::VectorXd& joint_vel,
							  const Eigen::Vector6d& base_acc,
							  const Eigen::VectorXd& joint_acc);

		/**
		 * @brief Updated the Jacobians and frame kinematics needed for
		 * computed the frame Jacobians (e.g. for calling getJointVelocity).
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 */
		void updateJacobians(const Eigen::Vector7d& base_pos,
							 const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the whole-body Center of Mass (CoM)
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @return Whole-body CoM
		 */
		const Eigen::Vector3d& computeCoM(const Eigen::Vector7d& base_pos,
										  const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the whole-body Center of Mass (CoM) and its velocity
		 * 
		 * @param[out] com CoM position
		 * @param[out] com_d CoM velocity
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @return CoM rate of the floating-base system
		 */
		void computeCoMRate(Eigen::Vector3d& com,
							Eigen::Vector3d& com_d,
							const Eigen::Vector7d& base_pos,
							const Eigen::VectorXd& joint_pos,
							const Eigen::Vector6d& base_vel,
							const Eigen::VectorXd& joint_vel);

		/**
		 * @brief Computes the operational position (i.e. SE3 group)
		 * for a predefined set of frames
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] frames Names of frames
		 * @return Configuration for each frame
		 */
		Eigen::Vector7dMap
		computePosition(const Eigen::Vector7d& base_pos,
						const Eigen::VectorXd& joint_pos,
						const ElementList& frames);

		/**
		 * @brief Gets the frame configuration.
		 * 
		 * @param[out] name Frame name
		 * @return Frame configuration [quaternion, position]
		 * 
		 * @warning You have to run first updateKinematics.
		 */
		const Eigen::Vector7d&
		getFramePosition(const std::string& name);

		/**
		 * @brief Computes the operational velocity (i.e. tangent of
		 * configuration space) for a predefined set of frames
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] frames Names of frames
		 * @return Operational velocity for each frame
		 */
		Eigen::Vector6dMap
		computeVelocity(const Eigen::Vector7d& base_pos,
						const Eigen::VectorXd& joint_pos,
						const Eigen::Vector6d& base_vel,
						const Eigen::VectorXd& joint_vel,
						const ElementList& frames);

		/**
		 * @brief Gets the frame velocity.
		 * 
		 * @param[out] name Frame name
		 * @return Frame velocity [angular, linear]
		 * 
		 * @warning You have to run first updateKinematics with the joint velocities.
		 */
		const Eigen::Vector6d&
		getFrameVelocity(const std::string& name);

		/**
		 * @brief Computes the operational acceleration for a predefined set
		 * of frames
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] base_acc Base acceleration [angular, linear]
		 * @param[in] joint_acc Joint acceleration
		 * @param[in] frames Names of frames
		 * @return Operational acceleration for each frame
		 */
		Eigen::Vector6dMap
		computeAcceleration(const Eigen::Vector7d& base_pos,
							const Eigen::VectorXd& joint_pos,
							const Eigen::Vector6d& base_vel,
							const Eigen::VectorXd& joint_vel,
							const Eigen::Vector6d& base_acc,
							const Eigen::VectorXd& joint_acc,
							const ElementList& frames);

		/**
		 * @brief Gets the frame acceleration.
		 * 
		 * @param[out] name Frame name
		 * @return Frame acceleration [angular, linear]
		 * 
		 * @warning You have to run first updateKinematics with the joint accelerations.
		 */
		const Eigen::Vector6d&
		getFrameAcceleration(const std::string& name);


		/**
		 * @brief Computes the operational acceleration contribution from the
		 * joint velocity for a predefined set of frames, i.e. J_d * q_d
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] base_vel Base velocity [angular, linear]
		 * @param[in] joint_vel Joint velocity
		 * @param[in] frames Names of frames
		 * @return Jdot * qdot term for each frame
		 */
		Eigen::Vector6dMap
		computeJdQd(const Eigen::Vector7d& base_pos,
					const Eigen::VectorXd& joint_pos,
					const Eigen::Vector6d& base_vel,
					const Eigen::VectorXd& joint_vel,
					const ElementList& frames);

		/**
		 * @brief Gets the Jdot * qdot term.
		 * 
		 * @param[out] name Frame name
		 * @return Jdot * qdot term [angular, linear]
		 * 
		 * @warning You have to run first updateKinematics with the joint velocities.
		 */
		const Eigen::Vector6d&
		getFrameJdQd(const std::string& name);

		//TODO: implement the whole-body IK
		// /**
		//  * @brief Computes the whole-body inverse kinematics for a predefined set of
		//  * frame configurations, e.g. base and feet configurations, etc.
		//  * @param[out] base_pos Base position
		//  * @param[out] joint_pos Joint position
		//  * @param[in] op_pos Configuration of bodies
		//  * @param[in] base_pos_init Warm-start base position to the solver
		//  * @param[in] joint_pos_init Warm-start joint position to the solver
		//  * @return True on success, false otherwise
		//  */
		// bool computeInverseKinematics(Eigen::Vector7d& base_pos,
		// 							  Eigen::VectorXd& joint_pos,
		// 							  const Eigen::Vector7dMap op_pos);
		// bool computeInverseKinematics(Eigen::Vector7d& base_pos,
		// 							  Eigen::VectorXd& joint_pos,
		// 							  const Eigen::Vector7dMap& op_pos,
		// 							  const Eigen::Vector7d& base_pos_init,
		// 							  const Eigen::VectorXd& joint_pos_init);

		/**
		 * @brief Computes the joint position from a predefined set of frame
		 * configurations expressed in the base frame. The IK problem is solved
		 * through a Gauss-Newton descent method. You can set the solver parameters
		 * with setIKSolver function.
		 * 
		 * This IK algorithm uses the entired desired configuration (i.e. SE3 group) or
		 * only the desired 3d position for a general cases or 3-DoF branches, respetively.
		 * 
		 * @param[out] joint_pos Joint position
		 * @param[in] frame_pos Frame configuration expressed in the base frame
		 * @param[in] joint_pos0 Warm-start joint position
		 * @return True on success, false otherwise
		 */
		bool computeJointPosition(Eigen::VectorXd& joint_pos,
								  const Eigen::Vector7dMap& frame_pos);
		bool computeJointPosition(Eigen::VectorXd& joint_pos,
								  const Eigen::Vector7dMap& frame_pos,
								  const Eigen::VectorXd& joint_pos0);

		/**
		 * @brief Computes the joint velocity for a predefined set of frame
		 * velocities (q_d = J^-1 * x_d) expressed in the base frame.
		 * 
		 * @param[out] joint_vel Joint velocities
		 * @param[in] joint_pos Joint positions
		 * @param[in] frame_vel Frame velocities expressed in the base frame
		 */
		void computeJointVelocity(Eigen::VectorXd& joint_vel,
								  const Eigen::VectorXd& joint_pos,
								  const Eigen::Vector6dMap& frame_vel);

		/**
		 * @brief Gets the joint velocity.
		 * 
		 * @param[out] joint_vel Joint velocity
		 * @return Frame velocity [angular, linear]
		 * 
		 * @warning You have to run first updateJacobians.
		 */
		void getJointVelocity(Eigen::VectorXd& joint_vel,
							  const Eigen::Vector6dMap& frame_vel);

		/**
		 * @brief Computes the joint acceleration for a predefined set of frame
		 * accelerations (q_dd = J^-1 * [x_dd - J_d * q_d]) expressed in the base frame.
		 * 
		 * @param[out] joint_acc Joint accelerations
		 * @param[in] joint_pos Joint positions
		 * @param[in] joint_vel Joint velocities
		 * @param[in] frame_acc Frame accelerations expressed in the base frame
		 */
		void computeJointAcceleration(Eigen::VectorXd& joint_acc,
									  const Eigen::VectorXd& joint_pos,
									  const Eigen::VectorXd& joint_vel,
									  const Eigen::Vector6dMap& frame_acc);

		/**
		 * @brief Gets the joint acceleration.
		 * 
		 * @param[out] joint_acc Joint acceleration
		 * @return Frame acceleration [angular, linear]
		 * 
		 * @warning You have to run first updateKinematics and updateJacobians.
		 */
		void getJointAcceleration(Eigen::VectorXd& joint_acc,
								  const Eigen::Vector6dMap& frame_acc);

		/**
		 * @brief Computes the whole-body Jacobians for a predefined set of
		 * frames. The whole-body Jacobians are defined as frame Jacobian w.r.t
		 * the inertial frame of the robot (i.e. world frame). The whole-body
		 * Jacobian represents a stack of frame Jacobians indexe by its frame name.
		 * 
		 * @param[in] base_pos Base configuration [quaternion, position]
		 * @param[in] joint_pos Joint position
		 * @param[in] frames Frame names
		 * @return Whole-body Jacobian for each frame
		 */
		Eigen::Matrix6xMap
		computeJacobian(const Eigen::Vector7d& base_pos,
						const Eigen::VectorXd& joint_pos,
						const ElementList& frames);

		/**
		 * @brief Gets the frame Jacobian expressed in the world frame.
		 * 
		 * @param[in] name Frame name
		 * @return Frame Jacobian [angular, linear]
		 * 
		 * @warning You have to run first updateJacobians.
		 */
		Eigen::Matrix6x
		getFrameJacobian(const std::string& name);

		/**
		 * @brief Gets the floating-base contribution of a given whole-body
		 * Jacobian. This belongs to the unactuated part of the robot. For
		 * fixed-base system returns a 6x6 matrix with zeros.		 * 
		 * 
		 * @param[out] Floating-base jacobian
		 * @param[in] Whole-body jacobian
		 */
		void getFloatingBaseJacobian(Eigen::Matrix6d& jacobian,
									 const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Gets the fixed-base jacobian contribution of a given
		 * whole-body Jacobian. This belongs to the actuated part of the robot.
		 * 
		 * @param[out] jac Fixed-base jacobian
		 * @param[in] full_jac Whole-body jacobian
		 */
		void getFixedBaseJacobian(Eigen::Matrix6x& jac,
								  const Eigen::Matrix6x& full_jac);


	private:
		/** @brief A floating-base system definition */
		std::shared_ptr<FloatingBaseSystem> fbs_;

		/** @brief Middle joint position */
		Eigen::VectorXd joint_pos_middle_;

		Eigen::Vector7d config_vec_;
		Eigen::Vector6d tangent_vec_;

		/** @brief IK solver */
		double step_tol_;
		unsigned int max_iter_;
};

} //@namespace model
} //@namespace dwl

#endif

