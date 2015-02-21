#ifndef DWL_HyQWholeBodyKinematics_H
#define DWL_HyQWholeBodyKinematics_H

#include <model/WholeBodyKinematics.h>
#include <iit/robots/hyq/jacobians.h>
#include <iit/robots/hyq/transforms.h>


namespace dwl
{

namespace robot
{

class HyQWholeBodyKinematics : public model::WholeBodyKinematics
{
	public:
		HyQWholeBodyKinematics();
		~HyQWholeBodyKinematics();

		void init();
		void updateState(Eigen::VectorXd state, Eigen::VectorXd state_dot);

	private:
		iit::HyQ::Jacobians jacs_;
		iit::HyQ::HomogeneousTransforms homogeneous_tf_;
		iit::HyQ::MotionTransforms motion_tf_;
};

} //@namespace robot
} //@namespace dwl

#endif
