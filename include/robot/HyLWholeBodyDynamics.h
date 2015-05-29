#ifndef DWL_HyLWholeBodyDynamics_H
#define DWL_HyLWholeBodyDynamics_H

#include <model/WholeBodyDynamics.h>
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
class HyLWholeBodyDynamics : public model::WholeBodyDynamics
{
	public:
		HyLWholeBodyDynamics();
		~HyLWholeBodyDynamics();

		/**
		 * @brief Updates the state of the HyL
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		void updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the whole-body inverse dynamics of HyL
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
		void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench,
											 Eigen::VectorXd& joint_forces,
				 	 	 	 	 	 	 	 const iit::rbd::Vector6D& g,
											 const iit::rbd::Vector6D& base_pos,
				 	 	 	 	 	 	 	 const Eigen::VectorXd& joint_pos,
				 	 	 	 	 	 	 	 const iit::rbd::Vector6D& base_vel,
											 const Eigen::VectorXd& joint_vel,
											 const iit::rbd::Vector6D& base_acc,
											 const Eigen::VectorXd& joint_acc);

		/**
		 * @brief Propagates the states for whole-body inverse dynamics of HyL
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 * @param const iit::rbd::Vector6D& Base velocity
		 * @param const Eigen::VectorXd& Joint velocity
		 * @param const iit::rbd::Vector6D& Base acceleration
		 * @param const Eigen::VectorXd& Joint acceleration
		 */
		void propagateWholeBodyInverseDynamics(const iit::rbd::Vector6D& base_pos,
											   const Eigen::VectorXd& joint_pos,
											   const iit::rbd::Vector6D& base_vel,
											   const Eigen::VectorXd& joint_vel,
											   const iit::rbd::Vector6D& base_acc,
											   const Eigen::VectorXd& joint_acc);

	private:
		/** @brief Inverse Dynamics of HyL */
		iit::HyL::dyn::InverseDynamics id_;

		/** @brief Motion transform of HyL */
		iit::HyL::MotionTransforms motion_tf_;

		/** @brief Inertia properties of HyL */
		iit::HyL::dyn::InertiaProperties inertia_;
};

} //@namespace model
} //@namespace dwl

#endif
