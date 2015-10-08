#ifndef DWL__ROBOT__HYQ_WHOLE_BODY_KINEMATICS__H
#define DWL__ROBOT__HYQ_WHOLE_BODY_KINEMATICS__H

#include <model/RobCoGenWholeBodyKinematics.h>
#include <iit/robots/hyq/jacobians.h>
#include <iit/robots/hyq/transforms.h>


namespace dwl
{

namespace robot
{

class HyQWholeBodyKinematics : public model::RobCoGenWholeBodyKinematics
{
	public:
		HyQWholeBodyKinematics();
		~HyQWholeBodyKinematics();

		void init();
		void updateState(Eigen::VectorXd state, Eigen::VectorXd state_dot);

	private:
		iit::HyQ::Jacobians jacs_;
		iit::HyQ::HomogeneousTransforms hom_tf_;
		iit::HyQ::MotionTransforms motion_tf_;
};

} //@namespace robot
} //@namespace dwl

#endif
