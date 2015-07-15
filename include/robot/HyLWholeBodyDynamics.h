#ifndef DWL__ROBOT__HYL_WHOLE_BODY_DYNAMICS__H
#define DWL__ROBOT__HYL_WHOLE_BODY_DYNAMICS__H

#include <model/RobCoGenWholeBodyDynamics.h>
#include <iit/robots/hyl/inverse_dynamics.h>
#include <iit/robots/hyl/inertia_properties.h>
#include <iit/robots/hyl/transforms.h>


namespace dwl
{

namespace robot
{

/**
 * @class This class implements methods for computing whole-body dynamics of HyL
 */
class HyLWholeBodyDynamics : public model::RobCoGenWholeBodyDynamics
{
	public:
		HyLWholeBodyDynamics();
		~HyLWholeBodyDynamics();

		/**
		 * @brief Updates the state of the HyL
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		void updateState(const rbd::Vector6d& base_pos, const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the whole-body inverse dynamics of HyL
		 * @param rbd::Vector6d& Base wrench
		 * @param Eigen::VectorXd& Joint forces
		 * @param const rbd::Vector6d& Gravity vector
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 * @param const rbd::BodyWrench External force applied to a certain body of the robot
		 */
		void computeInverseDynamics(rbd::Vector6d& base_wrench,
									Eigen::VectorXd& joint_forces,
									const rbd::Vector6d& g,
									const rbd::Vector6d& base_pos,
									const Eigen::VectorXd& joint_pos,
									const rbd::Vector6d& base_vel,
									const Eigen::VectorXd& joint_vel,
									const rbd::Vector6d& base_acc,
									const Eigen::VectorXd& joint_acc,
									const rbd::BodyWrench& ext_force = rbd::BodyWrench());

		/**
		 * @brief Propagates the states for whole-body inverse dynamics of HyL
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const rbd::Vector6d& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const rbd::Vector6d& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 */
		void propagateInverseDynamics(const rbd::Vector6d& base_pos,
									  const Eigen::VectorXd& joint_pos,
									  const rbd::Vector6d& base_vel,
									  const Eigen::VectorXd& joint_vel,
									  const rbd::Vector6d& base_acc,
									  const Eigen::VectorXd& joint_acc);

	private:
		/** @brief Inverse Dynamics of HyL */
		iit::HyL::dyn::InverseDynamics id_;

		/** @brief Motion transform of HyL */
		iit::HyL::MotionTransforms motion_tf_;

		/** @brief Force transform of HyL */
		iit::HyL::ForceTransforms force_tf_;

		/** @brief Inertia properties of HyL */
		iit::HyL::dyn::InertiaProperties inertia_;
};

} //@namespace model
} //@namespace dwl

#endif
