#ifndef DWL_HyLWholeBodyDynamics_H
#define DWL_HyLWholeBodyDynamics_H

#include <model/WholeBodyDynamics.h>
#include <iit/robots/hyl/inverse_dynamics.h>
#include <iit/robots/hyl/inertia_properties.h>
#include <iit/robots/hyl/transforms.h>
//#include <iit/robots/hyq/jacobians.h>
//#include <iit/robots/hyq/transforms.h>


namespace dwl
{

namespace robot
{

class HyLWholeBodyDynamics : public model::WholeBodyDynamics
{
	public:
		HyLWholeBodyDynamics();
		~HyLWholeBodyDynamics();

		void computeWholeBodyInverseDynamics(iit::rbd::Vector6D& base_wrench, Eigen::VectorXd& joint_forces,
		        const iit::rbd::Vector6D& g, const iit::rbd::Vector6D& base_vel, const iit::rbd::Vector6D& base_accel,
		        const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd);

	private:
		iit::HyL::MotionTransforms motion_tf_;
		iit::HyL::dyn::InertiaProperties inertia_;
		iit::HyL::dyn::InverseDynamics id_;//(inertia_, motion_tf_);
};

} //@namespace model
} //@namespace dwl

#endif
