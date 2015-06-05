#ifndef DWL_WholeBodyKinematics_H
#define DWL_WholeBodyKinematics_H

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <map>
#include <utils/utils.h>
#include <utils/Math.h>



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
		virtual ~WholeBodyKinematics();

		/**
		 * @brief Build the model rigid-body system from an URDF file
		 * @param std::string URDF file
		 * @param Print model information
		 */
		void modelFromURDF(std::string file, bool info = false);

		/**
		 * @brief Adds an end-effector to the list
		 * @param std::string End-effector name
		 */
		void addEndEffector(std::string name);

		/**
		 * @brief Computes the forward kinematics for all end-effectors of the robot
		 * @param Eigen::VectorXd& Operation position of end-effectors of the robot
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeWholeBodyFK(Eigen::VectorXd& op_pos,
								   const Vector6d& base_pos,
								   const Eigen::VectorXd& joint_pos,
								   enum Component component = Full,
								   enum TypeOfOrientation type = RollPitchYaw);

		/**
		 * @brief Computes the forward kinematics for a predefined set of end-effectors
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 * @param enum TypeOfOrientation Desired type of orientation
		 */
		void computeWholeBodyFK(Eigen::VectorXd& op_pos,
								   const Vector6d& base_pos,
								   const Eigen::VectorXd& joint_pos,
								   EndEffectorSelector effector_set,
								   enum Component component = Full,
								   enum TypeOfOrientation type = RollPitchYaw);

		/**
		 * @brief Computes the inverse kinematics for all set of end-effectors. This inverse kinematics
		 * algorithm uses an operational position which consists of the desired 3d position for the base and
		 * each end-effector
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param EndEffectorSelector A predefined set of end-effectors
		 */
		void computeWholeBodyIK(Vector6d& base_pos,
								   Eigen::VectorXd& joint_pos,
								   const Vector6d& base_pos_init,
								   const Eigen::VectorXd& joint_pos_init,
								   const Eigen::VectorXd& op_pos);
		/**
		 * @brief Computes the inverse kinematics for a predefined set of end-effectors. This inverse kinematics
		 * algorithm uses an operational position which consists of the desired 3d position for the base and
		 * each end-effector
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Initial base position for the iteration
		 * @param const Eigen::VectorXd& Initial joint position for the iteration
		 * @param Eigen::VectorXd& Operational position of end-effectors
		 * @param EndEffectorSelector A predefined set of end-effectors
		 */
		void computeWholeBodyIK(Vector6d& base_pos,
								   Eigen::VectorXd& joint_pos,
								   const Vector6d& base_pos_init,
								   const Eigen::VectorXd& joint_pos_init,
								   const Eigen::VectorXd& op_pos,
								   EndEffectorSelector effector_set);

		/**
		 * @brief Computes the whole-body jacobian for all end-effector of the robot. A whole-body jacobian is
		 * defined as end-effector jacobian with respect to the inertial frame. Additionally, the whole-body
		 * jacobian represents a stack of floating-base effector jacobians in which there are base and
		 * effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
										  const Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  enum Component component = Full);

		/**
		 * @brief Computes the whole-body jacobian for a predefined set of end-effectors. A whole-body jacobian
		 * is defined as end-effector jacobian with respect to the inertial frame of the robot. Additionally,
		 * the whole-body jacobian represents a stack of floating-base effector jacobians in which there are
		 * base and effector contributions
		 * @param Eigen::MatrixXd& Whole-body jacobian
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyJacobian(Eigen::MatrixXd& jacobian,
										  const Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  EndEffectorSelector effector_set,
										  enum Component component = Full);


		/**
		 * @brief Gets the floating-base contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Floating-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 */
		void getFloatingBaseJacobian(Eigen::MatrixXd& jacobian,
										 const Eigen::MatrixXd& full_jacobian);


		/**
		 * @brief Gets the fixed-base jacobian contribution of a given whole-body jacobian
		 * @param Eigen::MatrixXd& Fixed-base jacobian
		 * @param const Eigen::MatrixXd& Whole-body jacobian
		 */
		void getFixedBaseJacobian(Eigen::MatrixXd& jacobian,
									 const Eigen::MatrixXd& full_jacobian);

		/**
		 * @brief Computes the operational velocity from the joint space for all end-effectors of the
		 * robot
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyVelocity(Eigen::VectorXd& op_vel,
										  const Vector6d& base_pos,
										  const Eigen::VectorXd& joint_pos,
										  const Vector6d& base_vel,
										  const Eigen::VectorXd& joint_vel,
										  enum Component component = Full);
		/**
		 * @brief Computes the operational velocity from the joint space for a predefined set of
		 * end-effectors of the robot
		 * @param Eigen::VectorXd& Operational velocity
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyVelocity(Eigen::VectorXd& op_vel,
									  	  const Vector6d& base_pos,
									  	  const Eigen::VectorXd& joint_pos,
									  	  const Vector6d& base_vel,
									  	  const Eigen::VectorXd& joint_vel,
									  	  EndEffectorSelector effector_set,
									  	  enum Component component = Full);
		/**
		 * @brief Computes the operational acceleration from the joint space for all end-effectors of the
		 * robot
		 * @param Eigen::VectorXd& Operational acceleration
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyAcceleration(Eigen::VectorXd& op_acc,
										  	  const Vector6d& base_pos,
										  	  const Eigen::VectorXd& joint_pos,
										  	  const Vector6d& base_vel,
										  	  const Eigen::VectorXd& joint_vel,
										  	  const Vector6d& base_acc,
										  	  const Eigen::VectorXd& joint_acc,
										  	  enum Component component = Full);

		/**
		 * @brief Computes the operational acceleration from the joint space for a predefined set of
		 * end-effectors of the robot
		 * @param Eigen::VectorXd& Operational acceleration
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyAcceleration(Eigen::VectorXd& op_acc,
										  	  const Vector6d& base_pos,
										  	  const Eigen::VectorXd& joint_pos,
										  	  const Vector6d& base_vel,
										  	  const Eigen::VectorXd& joint_vel,
										  	  const Vector6d& base_acc,
										  	  const Eigen::VectorXd& joint_acc,
										  	  EndEffectorSelector effector_set,
										  	  enum Component component = Full);
		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for all
		 * end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyJdotQdot(Eigen::VectorXd& jacd_qd,
				  	  	  	  	  	  	  const Vector6d& base_pos,
				  	  	  	  	  	  	  const Eigen::VectorXd& joint_pos,
				  	  	  	  	  	  	  const Vector6d& base_vel,
				  	  	  	  	  	  	  const Eigen::VectorXd& joint_vel,
				  	  	  	  	  	  	  enum Component component = Full);

		/**
		 * @brief Computes the operational acceleration contribution from the joint velocity for a
		 * predefined set of end-effectors of the robot, i.e. Jac_d * q_d
		 * @param Eigen::VectorXd& Operational acceleration contribution from joint velocity
		 * @param const Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param EndEffectorSelector A predefined set of end-effectors
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeWholeBodyJdotQdot(Eigen::VectorXd& jacd_qd,
				  	  	  	  	  	  	  const Vector6d& base_pos,
				  	  	  	  	  	  	  const Eigen::VectorXd& joint_pos,
				  	  	  	  	  	  	  const Vector6d& base_vel,
				  	  	  	  	  	  	  const Eigen::VectorXd& joint_vel,
				  	  	  	  	  	  	  EndEffectorSelector effector_set,
				  	  	  	  	  	  	  enum Component component = Full);

		/**
		 * @brief Gets the number of active end-effectors
		 * @param EndEffectorSelector End-effector set
		 */
		int getNumberOfActiveEndEffectors(EndEffectorSelector effector_set);

		/**
		 * @brief Actives all the end-effectors of the robot
		 * @param EndEffectorSelector& End-effector set
		 */
		void activeAllEndEffector(EndEffectorSelector& effector_set);

		/** @brief Returns true if it's a floating-base robot */
		bool isFloatingBaseRobot();

		/**
		 * @brief Converts the base and joint states to a generalized joint state
		 * @param const Vector6d& Base state
		 * @param const Eigen::VectorXd& Joint state
		 * @return Eigen::VectorXd Generalized joint state
		 */
		Eigen::VectorXd toGeneralizedJointState(const Vector6d& base_state,
												   const Eigen::VectorXd& joint_state);

		/**
		 * @brief Converts the generalized joint state to base and joint states
		 * @param Vector6d& Base state
		 * @param Eigen::VectorXd& Joint state
		 * @return const Eigen::VectorXd Generalized joint state
		 */
		void fromGeneralizedJointState(Vector6d& base_state,
										   Eigen::VectorXd& joint_state,
										   const Eigen::VectorXd& generalized_state);


		private:
			/**
			 * @brief Computes the Jacobian in certain point of a specific body
			 * @param const Eigen::VectorXd& Generalized joint position
			 * @param unsigned int Body id
			 * @param const Eigen::Vector3d& 3d Position of the point
			 * @param Eigen::MatrixXd& Jacobian
			 * @param bool Update kinematic state
			 */
			void computePointJacobian(const Eigen::VectorXd& q,
				  				     	 unsigned int body_id,
				  				     	 const Eigen::Vector3d& point_position,
				  				     	 Eigen::MatrixXd& jacobian,
				  				     	 bool update_kinematics);

			/**
			 * @brief Computes the velocity in certain point of a specific body
			 * @param const Eigen::VectorXd& Generalized joint position
			 * @param const Eigen::VectorXd& Generalized joint velocity
			 * @param unsigned int Body id
			 * @param const Eigen::Vector3d& 3d Position of the point
			 * @param Eigen::MatrixXd& Jacobian
			 * @param bool Update kinematic state
			 */
			Vector6d computePointVelocity(const Eigen::VectorXd& q,
											 const Eigen::VectorXd& q_dot,
											 unsigned int body_id,
											 const Eigen::Vector3d point_position,
											 bool update_kinematics);

			/**
			 * @brief Computes the acceleration in certain point of a specific body
			 * @param const Eigen::VectorXd& Generalized joint position
			 * @param const Eigen::VectorXd& Generalized joint velocity
			 * @param const Eigen::VectorXd& Generalized joint acceleration
			 * @param unsigned int Body id
			 * @param const Eigen::Vector3d& 3d Position of the point
			 * @param Eigen::MatrixXd& Jacobian
			 * @param bool Update kinematic state
			 */
			Vector6d computePointAcceleration(const Eigen::VectorXd& q,
											 	 const Eigen::VectorXd& q_dot,
											 	 const Eigen::VectorXd& q_ddot,
											 	 unsigned int body_id,
											 	 const Eigen::Vector3d point_position,
											 	 bool update_kinematics);

			/** @brief Model of the rigid-body system */
			RigidBodyDynamics::Model robot_model_;

			/* @brief End-effector ids */
			EndEffectorID effector_id_;
};

} //@namespace model
} //@namespace dwl

#endif
