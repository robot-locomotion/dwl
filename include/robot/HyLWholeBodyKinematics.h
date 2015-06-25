#ifndef DWL_HyLWholeBodyKinematics_H
#define DWL_HyLWholeBodyKinematics_H

#include <model/RobCoGenWholeBodyKinematics.h>
#include <iit/robots/hyl/jacobians.h>
#include <iit/robots/hyl/transforms.h>


namespace dwl
{

namespace robot
{

/**
 * @class This class implements methods for computing kinematics of HyL
 */
class HyLWholeBodyKinematics : public model::RobCoGenWholeBodyKinematics
{
	public:
		/** @brief Constructor function */
		HyLWholeBodyKinematics();

		/** @brief Destructor function */
		~HyLWholeBodyKinematics();

		/**
		 * @brief Updates the state of the HyL
		 * @param const rbd::Vector6d& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		void updateState(const rbd::Vector6d& base_pos,
						   const Eigen::VectorXd& joint_pos);

		/**
		 * @brief Computes the inverse kinematics for all end-effectors of the robot
		 * @param Eigen::VectorXd& Operation position of end-effectors of the robot
		 * @param enum Component There are three different important kind of jacobian such as: linear,
		 * angular and full
		 */
		void computeInverseKinematics(Eigen::VectorXd& joint_pos,
									  Eigen::VectorXd& joint_vel,
									  const Eigen::VectorXd& op_pos,
									  const Eigen::VectorXd& op_vel);

	private:
		/** @brief Jacobians of HyL */
		iit::HyL::Jacobians jacs_;

		/** @brief Homogeneous transforms of HyL */
		iit::HyL::HomogeneousTransforms hom_tf_;

		/** @brief Motion transforms of HyL */
		iit::HyL::MotionTransforms motion_tf_;
};

} //@namespace robot
} //@namespace dwl

#endif
