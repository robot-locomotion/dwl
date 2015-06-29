#ifndef DWL_HyQWholeBodyDynamics_H
#define DWL_HyQWholeBodyDynamics_H

#include <model/WholeBodyDynamics.h>
#include <iit/robots/hyq/inverse_dynamics.h>
#include <iit/robots/hyq/inertia_properties.h>
#include <iit/robots/hyq/transforms.h>
//#include <iit/robots/hyq/jacobians.h>
//#include <iit/robots/hyq/transforms.h>


namespace dwl
{

namespace robot
{

class HyQWholeBodyDynamics : public model::WholeBodyDynamics
{
	public:
		HyQWholeBodyDynamics();
		~HyQWholeBodyDynamics();


	private:
		iit::HyQ::MotionTransforms motion_tf_;
		iit::HyQ::dyn::InertiaProperties inertia_;
		iit::HyQ::dyn::InverseDynamics id_;//(inertia_, motion_tf_);
};

} //@namespace model
} //@namespace dwl

#endif
