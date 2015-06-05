#ifndef DWL_RobCoGenWholeBodyDynamics_H
#define DWL_RobCoGenWholeBodyDynamics_H

#include <model/RobCoGenWholeBodyKinematics.h>
#include <iit/rbd/rbd.h>


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
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		virtual void updateState(const iit::rbd::Vector6D& base_pos,
								 	const Eigen::VectorXd& joint_pos) = 0;

		/**
		 * @brief Set the whole-body kinematics model
		 * @param WholeBodyKinematics* Pointer to the kinematics model
		 */
		void setKinematicModel(RobCoGenWholeBodyKinematics* kin_model);

		/**
		 * @brief An abstract method for computing whole-body inverse dynamics
		 * @param iit::rbd::Vector6D& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const iit::rbd::Vector6D& Gravity vector
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const iit::rbd::Vector6D& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 */
		virtual void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench,
														   Eigen::VectorXd& joint_forces,
														   const iit::rbd::Vector6D& g,
														   const iit::rbd::Vector6D& base_pos,
														   const Eigen::VectorXd& joint_pos,
														   const iit::rbd::Vector6D& base_vel,
														   const Eigen::VectorXd& joint_vel,
														   const iit::rbd::Vector6D& base_acc,
														   const Eigen::VectorXd& joint_acc) = 0;
		//													 const ExtForces& fext = zeroExtForces) = 0;

		/**
		 * @brief An abstract method for propagating the states for whole-body inverse dynamics
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const iit::rbd::Vector6D& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 */
		virtual void propagateWholeBodyInverseDynamics(const iit::rbd::Vector6D& base_pos,
													   	   	 const Eigen::VectorXd& joint_pos,
													   	   	 const iit::rbd::Vector6D& base_vel,
													   	   	 const Eigen::VectorXd& joint_vel,
													   	   	 const iit::rbd::Vector6D& base_acc,
													   	   	 const Eigen::VectorXd& joint_acc) = 0;

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for all
		 * end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																 	 	const iit::rbd::Vector6D& base_pos,
																 	 	const Eigen::VectorXd& joint_pos,
																 	 	const iit::rbd::Vector6D& base_vel,
																 	 	const Eigen::VectorXd& joint_vel,
																 	 	enum Component component = Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for a
		 * predefined set of end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		virtual void opAccelerationContributionFromJointVelocity(Eigen::VectorXd& jacd_qd,
																 	 	const iit::rbd::Vector6D& base_pos,
																 	 	const Eigen::VectorXd& joint_pos,
																 	 	const iit::rbd::Vector6D& base_vel,
																 	 	const Eigen::VectorXd& joint_vel,
																 	 	EndEffectorSelector effector_set,
																 	 	enum Component component = Full);

		typedef std::map<std::string, Eigen::Matrix<double, 6, 6> > EndEffectorSpatialTransform;
		typedef std::map<std::string, Eigen::Matrix<double, 6, 1> > EndEffectorSpatialVector;


	protected:
		/** @brief Whole-body kinematic model */
		model::RobCoGenWholeBodyKinematics* kin_model_;

		/** @brief True if the kinematics was initialized */
		bool initialized_kinematics_;

		/** @brief Motion transform of the closest link to the end-effector */
		EndEffectorSpatialTransform closest_link_motion_tf_;

		/** @brief Velocity of the closest link to the end-effector */
		EndEffectorSpatialVector closest_link_velocity_;

		/** @brief Acceleration of the closest link to the end-effector */
		EndEffectorSpatialVector closest_link_acceleration_;
};

} //@namespace model
} //@namespace dwl

#endif
