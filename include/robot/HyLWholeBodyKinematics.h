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
		 * @param const iit::rbd::Vector6D& Base position
		 * @param const Eigen::VectorXd& Joint position
		 */
		void updateState(const iit::rbd::Vector6D& base_pos,
						 const Eigen::VectorXd& joint_pos);

		void computeEffectorIK(Eigen::VectorXd& joint_pos,
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
