#include <model/KinematicConstraint.h>


namespace dwl
{

namespace model
{

KinematicConstraint::KinematicConstraint()
{
	name_ = "kinematic";
}


KinematicConstraint::~KinematicConstraint()
{
	printf("Destroying kinematic constraint\n");
}


void KinematicConstraint::get(Eigen::VectorXd& constraint, Eigen::VectorXd state)
{
	printf("Getting kinematic constraints\n");
}

} //@namespace model
} //@namespace dwl
