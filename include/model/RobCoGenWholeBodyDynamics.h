#ifndef DWL_RobCoGenWholeBodyDynamics_H
#define DWL_RobCoGenWholeBodyDynamics_H

#include <model/RobCoGenWholeBodyKinematics.h>
#include <utils/RigidBodyDynamics.h>


namespace dwl
{

namespace model
{

/**
 * @brief RobCoGenWholeBodyDynamics class implements the dynamics methods for a floating-base robot using
 * RobCoGen library
 */
class RobCoGenWholeBodyDynamics
{
	public:
		/** @bried Constructor function */
		RobCoGenWholeBodyDynamics();

		/** @brief Destructor function */
		virtual ~RobCoGenWholeBodyDynamics();

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
		 * @brief Set the whole-body kinematics model
		 * @param WholeBodyKinematics* Pointer to the kinematics model
		 */
		void setKinematicModel(RobCoGenWholeBodyKinematics* kin_model);

		/**
		 * @brief An abstract method for computing whole-body inverse dynamics
		 * @param rbd::Vector6d& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Gravity vector
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodyForce External force applied to a certain body of the robot
		 */
		virtual void computeInverseDynamics(rbd::Vector6d& base_wrench,
											Eigen::VectorXd& joint_forces,
											const rbd::Vector6d& g,
											const rbd::Vector6d& base_pos,
											const Eigen::VectorXd& joint_pos,
											const rbd::Vector6d& base_vel,
											const Eigen::VectorXd& joint_vel,
											const rbd::Vector6d& base_acc,
											const Eigen::VectorXd& joint_acc,
											const rbd::BodyForce& ext_force = rbd::BodyForce()) = 0;

		/**
		 * @brief An abstract method for propagating the states for whole-body inverse dynamics
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 */
		virtual void propagateInverseDynamics(const rbd::Vector6d& base_pos,
											  const Eigen::VectorXd& joint_pos,
											  const rbd::Vector6d& base_vel,
											  const Eigen::VectorXd& joint_vel,
											  const rbd::Vector6d& base_acc,
											  const Eigen::VectorXd& joint_acc) = 0;

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
		virtual void opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																 const rbd::Vector6d& base_pos,
																 const Eigen::VectorXd& joint_pos,
																 const rbd::Vector6d& base_vel,
																 const Eigen::VectorXd& joint_vel,
																 enum rbd::Component component = rbd::Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for a
		 * predefined set of bodies of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param rbd::BodySelector A predefined set of bodies
		 * @param enum rbd::Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																 const rbd::Vector6d& base_pos,
																 const Eigen::VectorXd& joint_pos,
																 const rbd::Vector6d& base_vel,
																 const Eigen::VectorXd& joint_vel,
																 rbd::BodySelector body_set,
																 enum rbd::Component component = rbd::Full);

		typedef std::map<std::string, Eigen::Matrix<double, 6, 6> > BodySpatialTransform;
		typedef std::map<std::string, Eigen::Matrix<double, 6, 1> > BodySpatialVector;


	protected:
		/** @brief Whole-body kinematic model */
		model::RobCoGenWholeBodyKinematics* kin_model_;

		/** @brief True if the kinematics was initialized */
		bool initialized_kinematics_;

		/** @brief Motion transform of the closest link to the end-effector */
		BodySpatialTransform closest_link_motion_tf_;

		/** @brief Velocity of the closest link to the end-effector */
		BodySpatialVector closest_link_velocity_;

		/** @brief Acceleration of the closest link to the end-effector */
		BodySpatialVector closest_link_acceleration_;
};

} //@namespace model
} //@namespace dwl

#endif
