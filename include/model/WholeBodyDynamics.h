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
		 * @brief An abstract method for computing whole-body inverse dynamics
		 * @param rbd::Vector6d& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::Vector6d& External force applied to the robot
		 */
		void computeWholeBodyInverseDynamics(rbd::Vector6d& base_wrench,
												  Eigen::VectorXd& joint_forces,
												  const rbd::Vector6d& base_pos,
												  const Eigen::VectorXd& joint_pos,
												  const rbd::Vector6d& base_vel,
												  const Eigen::VectorXd& joint_vel,
												  const rbd::Vector6d& base_acc,
												  const Eigen::VectorXd& joint_acc,
												  const rbd::Vector6d& ext_force = rbd::Vector6d::Zero());


	private:
		/** @brief Model of the rigid-body system */
		RigidBodyDynamics::Model robot_model_;
};

} //@namespace model
} //@namespace dwl

#endif
