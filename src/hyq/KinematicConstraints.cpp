#include <hyq/KinematicConstraints.h>


namespace dwl
{

namespace hyq
{

KinematicConstraints::KinematicConstraints()
{
	name_ = "kinematic";
	is_active_constraint_ = true;
}

KinematicConstraints::~KinematicConstraints()
{

}

void KinematicConstraints::get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	printf("Getting kinematic constraints\n");
}


} //@namespace hyq
} //@namespace dwl
