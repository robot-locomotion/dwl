#ifndef DWL_HyQFloatingBaseKinematics_H
#define DWL_HyQFloatingBaseKinematics_H

#include <model/WholeBodyKinematics.h>
#include <iit/robots/hyq/jacobians.h>
#include <iit/robots/hyq/transforms.h>


namespace dwl
{

namespace robot
{

class HyLWholeBodyKinematics : public model::WholeBodyKinematics
{
	public:
		HyLWholeBodyKinematics();
		~HyLWholeBodyKinematics();

		void init();
		void updateJointState(Eigen::VectorXd position, Eigen::VectorXd velocity);

	private:
		iit::HyQ::Jacobians jacs_;
		iit::HyQ::HomogeneousTransforms tf_;

//		typedef std::map<std::string, Eigen::Matrix<double, 4, 4>*> EndEffectorTransform;
};

} //@namespace robot
} //@namespace dwl

#endif
