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

class HyLWholeBodyDynamics : public model::WholeBodyDynamics
{
	public:
		HyLWholeBodyDynamics();
		~HyLWholeBodyDynamics();

		void init();
		void updateState(const iit::rbd::Vector6D& base_pos, const Eigen::VectorXd& joint_pos);

		void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
															const iit::rbd::Vector6D& base_pos,
															const iit::rbd::Vector6D& base_vel,
															const Eigen::VectorXd& joint_pos,
															const Eigen::VectorXd& joint_vel);
		void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd,
															EndEffectorSelector effector_set,
															const iit::rbd::Vector6D& base_pos,
															const iit::rbd::Vector6D& base_vel,
															const Eigen::VectorXd& joint_pos,
															const Eigen::VectorXd& joint_vel);

		void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
				 	 	 	 	 	 	 	 const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_pos,
				 	 	 	 	 	 	 	 const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_acc,
				 	 	 	 	 	 	 	 const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_vel,
				 	 	 	 	 	 	 	 const Eigen::VectorXd& joint_acc);

	private:
		iit::HyL::MotionTransforms motion_tf_;
		iit::HyL::dyn::InertiaProperties inertia_;
		iit::HyL::dyn::InverseDynamics id_;
};

} //@namespace model
} //@namespace dwl

#endif
