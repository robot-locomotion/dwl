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
		void updateState(Eigen::VectorXd state);

		void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd, Eigen::VectorXd q, Eigen::VectorXd qd);
		void computeJointVelocityContributionOfAcceleration(Eigen::VectorXd& jacd_qd, EndEffectorSelector effector_set,
				Eigen::VectorXd q, Eigen::VectorXd qd);

		void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		        const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_accel,
		        const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd);

	private:
		iit::HyL::MotionTransforms motion_tf_;
		iit::HyL::dyn::InertiaProperties inertia_;
		iit::HyL::dyn::InverseDynamics id_;
};

} //@namespace model
} //@namespace dwl

#endif
