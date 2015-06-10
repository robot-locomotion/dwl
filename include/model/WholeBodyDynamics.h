#ifndef DWL_WholeBodyDynamics_H
#define DWL_WholeBodyDynamics_H

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
 * @brief WholeBodyDynamics class implements the dynamics methods for a floating-base robot
 */
class WholeBodyDynamics
{
	public:
		/** @bried Constructor function */
		WholeBodyDynamics();

		/** @brief Destructor function */
		~WholeBodyDynamics();

		/**
		 * @brief Build the model rigid-body system from an URDF file
		 * @param std::string URDF file
		 * @param Print model information
		 */
		void modelFromURDF(std::string file, bool info = false);

		/**
		 * @brief Computes the whole-body inverse dynamics using the Recursive Newton-Euler Algorithm (RNEA).
		 * An applied external force is defined for a certain body, movable or fixed body, where a fixed
		 * body is considered a fixed point of a movable one. These forces are represented as Cartesian forces
		 * applied to the body, where the first three elements are the moments and the last three elements are
		 * the linear forces. In general a point only has linear forces, but with this representation we can
		 * model the forces applied by a surface of contact in the center of pressure of it.
		 * @param rbd::Vector6d& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::EndEffectorForce External force applied to a certain body of the robot
		 */
		void computeWholeBodyInverseDynamics(rbd::Vector6d& base_wrench,
												  Eigen::VectorXd& joint_forces,
												  const rbd::Vector6d& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  const rbd::Vector6d& base_vel,
												  const Eigen::VectorXd& joint_vel,
												  const rbd::Vector6d& base_acc,
												  const Eigen::VectorXd& joint_acc,
												  const rbd::EndEffectorForce& ext_force = rbd::EndEffectorForce());

		/**
		 * @brief Computes the constrained whole-body inverse dynamics using the Recursive Newton-Euler
		 * Algorithm (RNEA). Constrained are defined by the contacts of the robot. Contacts could be defined
		 * for movable and fixed bodies, where a fixed body is considered a fixed point of a movable one.
		 * Thus, this approach allows us to compute the inverse dynamic when we have a predefined set of
		 * contacts, and without specific information of the contact forces of these contacts. Here we are
		 * assuming that the desired movement is realizable without base wrench (i.e. the hand's God).
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::EndEffectorForce External force applied to a certain body of the robot
		 */
		void computeConstrainedWholeBodyInverseDynamics(Eigen::VectorXd& joint_forces,
															  const rbd::Vector6d& base_pos,
															  const Eigen::VectorXd& joint_pos,
															  const rbd::Vector6d& base_vel,
															  const Eigen::VectorXd& joint_vel,
															  const rbd::Vector6d& base_acc,
															  const Eigen::VectorXd& joint_acc,
															  const rbd::EndEffectorSelector& contacts);


	private:
		/** @brief Model of the rigid-body system */
		RigidBodyDynamics::Model robot_model_;

		/* @brief Body ids */
		rbd::EndEffectorID body_id_;
};

} //@namespace model
} //@namespace dwl

#endif
